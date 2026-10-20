/*
  ======================================================================
  Rover diferencial - ESP32 + L9110S + HC-SR04 (ultrasónico) + Control Web
  PlatformIO - framework arduino-esp32 core v2.x (ledcSetup/ledcAttachPin)
  ======================================================================
  Modos (máquina de estados):
    - APAGADO           : motores deshabilitados, ignora comandos de dirección.
    - MANUAL_WEB         : el rover obedece los botones del panel web.
    - AUTONOMO_OBSTACULO : el rover avanza y esquiva obstáculos usando el
                      HC-SR04, ignorando los botones de dirección (solo
                      "Detener" lo saca de este modo).

  Además hay una bandeja volcable con servo SG90, independiente de la
  máquina de estados de arriba: se dispara por botón web, por pulsador
  físico, o sola al detectar un obstáculo en modo autónomo.

  Todo el loop() es no bloqueante (sin delay), usando millis() donde hace
  falta temporizar. La única excepción acotada es pulseIn() del HC-SR04,
  que bloquea como máximo ECO_TIMEOUT_US y solo se llama cada
  INTERVALO_LECTURA_MS, no en cada vuelta del loop.
  ======================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ---------- Driver L298N: entradas IN1..IN4 ----------
// Motor A (izquierdo) en OUT1/OUT2, Motor B (derecho) en OUT3/OUT4.
// ENA y ENB deben quedar con el jumper puesto (habilitados permanentemente);
// la velocidad se controla por PWM sobre IN1..IN4.
const int MOTOR_A_ADELANTE = 26;   // -> IN1
const int MOTOR_A_REVERSA  = 25;   // -> IN2

const int MOTOR_B_ADELANTE = 27;   // -> IN3
const int MOTOR_B_REVERSA  = 14;   // -> IN4

// ---------- Parámetros PWM ----------
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 230;

const int CH_MOTOR_A_ADELANTE = 0;
const int CH_MOTOR_A_REVERSA  = 1;
const int CH_MOTOR_B_ADELANTE = 2;
const int CH_MOTOR_B_REVERSA  = 3;

// ---------- Sensor ultrasónico HC-SR04 ----------
// Alimentado a 3.3V (VCC del sensor al pin 3V3 del ESP32, no a 5V/VIN): así
// el pulso de ECHO también sale a ~3.3V y se puede conectar directo al GPIO
// sin divisor resistivo. Ver README para el detalle y el trade-off de
// alcance reducido.
const int TRIG_PIN = 32;
const int ECHO_PIN = 34; // solo entrada, ideal para leer el pulso de ECHO

const unsigned long ECO_TIMEOUT_US = 25000; // ~4m de alcance máximo, evita bloquear si no hay eco
const unsigned long INTERVALO_LECTURA_MS = 60; // no disparar el sensor en cada vuelta del loop

// ponytail: umbral fijo — ajustar según qué tan cerca querés que el rover
// reaccione a un obstáculo real en la pista.
const int UMBRAL_DISTANCIA_CM = 35;

// Dispara el HC-SR04 y mide la distancia en cm. Bloquea como máximo
// ECO_TIMEOUT_US (pulseIn con timeout), nunca de forma indefinida.
// Devuelve -1 si no hubo eco (fuera de rango o sin obstáculo).
long medirDistanciaCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duracion = pulseIn(ECHO_PIN, HIGH, ECO_TIMEOUT_US);
  if (duracion == 0) return -1;
  return duracion / 58; // fórmula estándar del HC-SR04: us / 58 = cm
}

// ---------- Bandeja volcable (servo SG90) ----------
// El servo va acostado sobre el chasis y su brazo (hornilla) sube en
// perpendicular sosteniendo la bandeja. Reposo = bandeja paralela al suelo;
// descarga = bandeja perpendicular al suelo (vuelca el contenido).
const int SERVO_PIN = 13;        // GPIO13: sin función especial al arrancar
const int PIN_BOTON_VOLCAR = 33; // pulsador físico opcional (a GND, con pull-up interno)

// Pulsos típicos del SG90 a 50 Hz. Si el servo zumba o topa al llegar al
// extremo, acortar el rango (p. ej. 600-2300); no se recomienda salir de 500-2400.
const int SERVO_PULSO_MIN_US = 500;  // ancho de pulso que corresponde a 0°
const int SERVO_PULSO_MAX_US = 2400; // ancho de pulso que corresponde a 180°

// AFINACIÓN DE NIVELACIÓN: son los dos únicos valores a tocar para calibrar.
//  - ANGULO_REPOSO: si la bandeja queda inclinada al transportar, subir o
//    bajar de a 1-2° (rango válido 0-180) hasta que quede paralela al suelo.
//  - ANGULO_DESCARGA: el servo gira de 0° hacia 90°, así que si la bandeja no
//    llega a la vertical, aumentar (92, 95...); si se pasa, disminuir (88, 85...).
// Para volver a invertir el sentido de giro: intercambiar los dos valores
// (0 <-> 180) y remontar la hornilla del lado opuesto.
const int ANGULO_REPOSO   = 0;
const int ANGULO_DESCARGA = 90;

const unsigned long TIEMPO_DESCARGA_MS = 2000; // pausa arriba para que caiga lo depositado

// Los canales LEDC 0-3 (20 kHz) de los motores usan los timers 0 y 1; en
// setup() se restringe ESP32Servo a los timers 2 y 3 para no pisarlos.
Servo servoBandeja;

enum EstadoBandeja { BANDEJA_REPOSO, BANDEJA_DESCARGANDO };
EstadoBandeja estadoBandeja = BANDEJA_REPOSO;
unsigned long inicioDescargaMs = 0;

// ---------- Velocidades ----------
const int VELOCIDAD_MANUAL    = 255;
const int VELOCIDAD_GIRO      = 255;
const int VELOCIDAD_AUTONOMO  = 230;

// ponytail: compensación fija por desbalance entre ruedas, en 0 para que
// ambas reciban la misma potencia con el L298N. Subir este valor solo si el
// rover se va a un lado al ir recto.
const int TRIM_IZQUIERDA = 10;

// ---------- WiFi AP ----------
const char *AP_SSID = "Rover-ESP32";
const char *AP_PASS = "12345678";
WebServer server(80);

// ---------- Máquina de estados ----------
enum EstadoRover { APAGADO, MANUAL_WEB, AUTONOMO_OBSTACULO };
EstadoRover estadoActual = APAGADO;

// ---------- Funciones de control individual ----------

// Motor izquierdo (conectado a la salida "Motor A" de la placa)
// velocidad positiva = adelante, negativa = reversa, 0 = libre
void motorIzquierdo(int velocidad) {
  if (velocidad > 0) {
    ledcWrite(CH_MOTOR_A_ADELANTE, velocidad);
    ledcWrite(CH_MOTOR_A_REVERSA, 0);
  } else if (velocidad < 0) {
    ledcWrite(CH_MOTOR_A_ADELANTE, 0);
    ledcWrite(CH_MOTOR_A_REVERSA, -velocidad);
  } else {
    ledcWrite(CH_MOTOR_A_ADELANTE, 0);
    ledcWrite(CH_MOTOR_A_REVERSA, 0);
  }
}

// Motor derecho (conectado a la salida "Motor B" de la placa)
void motorDerecho(int velocidad) {
  if (velocidad > 0) {
    ledcWrite(CH_MOTOR_B_ADELANTE, velocidad);
    ledcWrite(CH_MOTOR_B_REVERSA, 0);
  } else if (velocidad < 0) {
    ledcWrite(CH_MOTOR_B_ADELANTE, 0);
    ledcWrite(CH_MOTOR_B_REVERSA, -velocidad);
  } else {
    ledcWrite(CH_MOTOR_B_ADELANTE, 0);
    ledcWrite(CH_MOTOR_B_REVERSA, 0);
  }
}

// ---------- Funciones de movimiento del chasis completo ----------

// ponytail: un salto brusco de 0 a la velocidad objetivo pide un pico de
// corriente que hace caer la tensión de golpe (brownout). Se usa una rampa
// lineal desde PWM_ARRANQUE (no desde 0) hasta la velocidad objetivo a lo
// largo de RAMPA_ARRANQUE_MS, recalculada en cada llamada con millis().
// PWM_ARRANQUE supera la fricción estática de las ruedas: a PWM bajo una
// rueda no arranca sola. Subir/bajar este valor si una rueda sigue trabada.
const unsigned long RAMPA_ARRANQUE_MS = 400;
const int PWM_ARRANQUE = 200;
bool enMovimiento = false;
unsigned long inicioRampa = 0;

int conRampaDeArranque(int velocidadObjetivo) {
  if (!enMovimiento) {
    enMovimiento = true;
    inicioRampa = millis();
  }
  int inicio = min(PWM_ARRANQUE, velocidadObjetivo);
  unsigned long transcurrido = millis() - inicioRampa;
  if (transcurrido >= RAMPA_ARRANQUE_MS) {
    return velocidadObjetivo;
  }
  return inicio + (int)((long)(velocidadObjetivo - inicio) * transcurrido / RAMPA_ARRANQUE_MS);
}

void avanzar(int velocidad) {
  int v = conRampaDeArranque(velocidad);
  motorIzquierdo(min(v + TRIM_IZQUIERDA, PWM_MAX));
  motorDerecho(v);
}

void retroceder(int velocidad) {
  int v = conRampaDeArranque(velocidad);
  motorIzquierdo(-min(v + TRIM_IZQUIERDA, PWM_MAX));
  motorDerecho(-v);
}

// Frenado activo en ambos motores (las 4 entradas en HIGH)
void frenarTodo() {
  enMovimiento = false;
  ledcWrite(CH_MOTOR_A_ADELANTE, PWM_MAX);
  ledcWrite(CH_MOTOR_A_REVERSA, PWM_MAX);
  ledcWrite(CH_MOTOR_B_ADELANTE, PWM_MAX);
  ledcWrite(CH_MOTOR_B_REVERSA, PWM_MAX);
}

// Punto muerto (las 4 entradas en LOW)
void liberarTodo() {
  enMovimiento = false;
  motorIzquierdo(0);
  motorDerecho(0);
}

// Giro sobre el propio eje hacia la izquierda
void girarIzquierda(int velocidad) {
  int v = conRampaDeArranque(velocidad);
  motorIzquierdo(-v);
  motorDerecho(v);
}

// Giro sobre el propio eje hacia la derecha
void girarDerecha(int velocidad) {
  int v = conRampaDeArranque(velocidad);
  motorIzquierdo(v);
  motorDerecho(-v);
}

// ---------- Bandeja: secuencia inclinar -> pausa -> retorno ----------
// Funciona en paralelo a la máquina de estados del rover (no es un
// EstadoRover más): un volcado puede dispararse en MANUAL_WEB, en
// AUTONOMO_OBSTACULO o con el pulsador físico, sin interferir con los
// motores.

// Dispara la secuencia. No bloquea: solo manda el servo a la posición de
// descarga y arranca el cronómetro; el retorno lo hace actualizarBandeja().
// Si ya hay una descarga en curso, la orden se ignora.
void volcarBandeja() {
  if (estadoBandeja != BANDEJA_REPOSO) return;
  servoBandeja.write(ANGULO_DESCARGA);
  inicioDescargaMs = millis();
  estadoBandeja = BANDEJA_DESCARGANDO;
}

// Se llama en cada vuelta del loop: vuelve al reposo pasado el tiempo de espera.
void actualizarBandeja() {
  if (estadoBandeja == BANDEJA_DESCARGANDO &&
      millis() - inicioDescargaMs >= TIEMPO_DESCARGA_MS) {
    servoBandeja.write(ANGULO_REPOSO);
    estadoBandeja = BANDEJA_REPOSO;
  }
}

// Lee el disparador físico (flanco de bajada = pulsador presionado).
void leerBotonVolcar() {
  static bool anterior = HIGH;
  static unsigned long ultimoCambioMs = 0;
  bool actual = digitalRead(PIN_BOTON_VOLCAR);
  if (actual != anterior && millis() - ultimoCambioMs > 50) { // antirrebote
    ultimoCambioMs = millis();
    anterior = actual;
    if (actual == LOW) volcarBandeja();
  }
}

// ---------- Comando manual actual ----------
// Los botones del panel web mandan un solo request por click (no
// mantienen el botón apretado), así que guardamos el último comando y lo
// reaplicamos en cada vuelta del loop(). Esto es necesario para que la
// rampa de arranque (que avanza en base a llamadas sucesivas) progrese;
// si el comando sólo se ejecutara una vez dentro del handler HTTP, la
// rampa quedaría congelada en el valor bajísimo del primer instante,
// dejando el motor sin fuerza para girar o mantener línea recta.
char comandoManual = 's';
char comandoManualPrevio = 's';

void aplicarComandoManual() {
  if (comandoManual != comandoManualPrevio) {
    enMovimiento = false; // fuerza una rampa nueva para el comando entrante
    comandoManualPrevio = comandoManual;
  }
  switch (comandoManual) {
    case 'f': avanzar(VELOCIDAD_MANUAL); break;
    case 'b': retroceder(VELOCIDAD_MANUAL); break;
    case 'l': girarIzquierda(VELOCIDAD_GIRO); break;
    case 'r': girarDerecha(VELOCIDAD_GIRO); break;
    default:  frenarTodo(); break;
  }
}

// ---------- Temporizador de volcado en modo autónomo ----------
// A los TIEMPO_AUTONOMO_ANTES_DE_VOLCAR_MS de haber entrado en
// AUTONOMO_OBSTACULO, vuelca la bandeja una sola vez (no se repite en la
// misma sesión autónoma); el rover sigue esquivando obstáculos antes,
// durante (frenado, ver loop()) y después del volcado. handleAuto()
// reinicia inicioAutonomoMs y volcadoAutonomoHecho cada vez que se activa
// el modo.
const unsigned long TIEMPO_AUTONOMO_ANTES_DE_VOLCAR_MS = 5000;
unsigned long inicioAutonomoMs = 0;
bool volcadoAutonomoHecho = false;

// ---------- Navegación autónoma (esquiva obstáculos con el HC-SR04) ----------
// Avanza en línea recta; si detecta un obstáculo a menos de
// UMBRAL_DISTANCIA_CM, gira hacia la derecha hasta despejar el camino. La
// lectura del sensor se limita a una vez cada INTERVALO_LECTURA_MS (en vez
// de cada vuelta del loop) para no bloquear tan seguido con pulseIn().
void ejecutarNavegacionAutonoma() {
  static unsigned long ultimaLectura = 0;
  static long distanciaCm = -1;

  if (!volcadoAutonomoHecho &&
      millis() - inicioAutonomoMs >= TIEMPO_AUTONOMO_ANTES_DE_VOLCAR_MS) {
    volcarBandeja();
    volcadoAutonomoHecho = true;
  }

  if (millis() - ultimaLectura >= INTERVALO_LECTURA_MS) {
    ultimaLectura = millis();
    distanciaCm = medirDistanciaCm();
  }

  bool obstaculoCerca = distanciaCm > 0 && distanciaCm < UMBRAL_DISTANCIA_CM;

  if (obstaculoCerca) {
    girarDerecha(VELOCIDAD_GIRO);
  } else {
    avanzar(VELOCIDAD_AUTONOMO);
  }
}

// Diagnóstico: por qué arrancó el ESP32 (BROWNOUT = caída de tensión).
const char *motivoReinicio() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON (encendido normal)";
    case ESP_RST_BROWNOUT: return "BROWNOUT (caida de tension)";
    case ESP_RST_SW:       return "SW (reinicio por software)";
    case ESP_RST_PANIC:    return "PANIC (error de programa)";
    case ESP_RST_WDT: case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: return "WATCHDOG";
    default:               return "OTRO";
  }
}

// ---------- Panel web ----------
const char PAGINA_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rover ESP32</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; text-align:center; margin:0; padding:16px; }
  h1 { font-size: 1.2em; }
  .estado { margin: 8px 0 20px; font-size: 0.95em; color:#9f9; }
  .dpad { display:grid; grid-template-columns: repeat(3, 70px); grid-gap:10px; justify-content:center; margin-bottom:20px; }
  button { font-size:1.3em; padding:14px; border:none; border-radius:10px; background:#333; color:#fff; }
  button:active { background:#555; }
  .stop { background:#a33; }
  .toggles { display:flex; gap:12px; justify-content:center; flex-wrap:wrap; }
  .toggles button { min-width:150px; background:#245; }
  .toggles button.on { background:#2a5; }
  .bandeja { margin-top:16px; min-width:200px; background:#a70; }
  .bandeja.activo { background:#333; color:#999; }
</style>
</head>
<body>
  <h1>Control Rover ESP32</h1>
  <div class="estado" id="estado">Cargando...</div>

  <div class="dpad">
    <div></div>
    <button onclick="cmd('fwd')">&#9650;</button>
    <div></div>
    <button onclick="cmd('left')">&#9664;</button>
    <button class="stop" onclick="cmd('stop')">&#9632;</button>
    <button onclick="cmd('right')">&#9654;</button>
    <div></div>
    <button onclick="cmd('back')">&#9660;</button>
    <div></div>
  </div>

  <div class="toggles">
    <button id="btnPower" onclick="togglePower()">ON / OFF</button>
    <button id="btnAuto" onclick="toggleAuto()">Esquivar Obstáculos</button>
  </div>

  <button class="bandeja" id="btnBandeja" onclick="volcar()">Volcar bandeja</button>

<script>
let power = false;
let auto = false;

function cmd(d) {
  fetch('/api/dir?d=' + d);
}

function volcar() {
  const b = document.getElementById('btnBandeja');
  fetch('/api/volcar');
  b.classList.add('activo');
  b.innerText = 'Volcando...';
  setTimeout(() => { b.classList.remove('activo'); b.innerText = 'Volcar bandeja'; }, 2500);
}

function togglePower() {
  power = !power;
  fetch('/api/power?v=' + (power ? 1 : 0)).then(refrescar);
}

function toggleAuto() {
  auto = !auto;
  fetch('/api/auto?v=' + (auto ? 1 : 0)).then(refrescar);
}

function refrescar() {
  fetch('/api/status').then(r => r.json()).then(s => {
    power = s.estado !== 'APAGADO';
    auto = s.estado === 'AUTONOMO_OBSTACULO';
    document.getElementById('estado').innerText = 'Estado: ' + s.estado + ' | Distancia: ' + s.sensor + ' cm';
    document.getElementById('btnPower').className = power ? 'on' : '';
    document.getElementById('btnAuto').className = auto ? 'on' : '';
  });
}

setInterval(refrescar, 1000);
refrescar();
</script>
</body>
</html>
)HTML";

// ---------- Handlers HTTP ----------

void handleRoot() {
  server.send_P(200, "text/html", PAGINA_HTML);
}

void handleDir() {
  if (estadoActual != MANUAL_WEB) {
    server.send(200, "text/plain", "ignorado");
    return;
  }
  String d = server.arg("d");
  if (d == "fwd") comandoManual = 'f';
  else if (d == "back") comandoManual = 'b';
  else if (d == "left") comandoManual = 'l';
  else if (d == "right") comandoManual = 'r';
  else if (d == "stop") comandoManual = 's';
  server.send(200, "text/plain", "ok");
}

void handlePower() {
  bool encender = server.arg("v") == "1";
  if (encender && estadoActual == APAGADO) {
    estadoActual = MANUAL_WEB;
    comandoManual = 's';
  } else if (!encender) {
    liberarTodo();
    estadoActual = APAGADO;
    comandoManual = 's';
  }
  server.send(200, "text/plain", "ok");
}

void handleAuto() {
  bool activar = server.arg("v") == "1";
  if (activar && estadoActual == MANUAL_WEB) {
    estadoActual = AUTONOMO_OBSTACULO;
    inicioAutonomoMs = millis();
    volcadoAutonomoHecho = false;
  } else if (!activar && estadoActual == AUTONOMO_OBSTACULO) {
    frenarTodo();
    liberarTodo();
    estadoActual = MANUAL_WEB;
    comandoManual = 's';
  }
  server.send(200, "text/plain", "ok");
}

void handleVolcar() {
  volcarBandeja();
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  const char *nombre =
    estadoActual == APAGADO ? "APAGADO" :
    estadoActual == AUTONOMO_OBSTACULO ? "AUTONOMO_OBSTACULO" : "MANUAL_WEB";
  String json = String("{\"estado\":\"") + nombre + "\",\"sensor\":" + medirDistanciaCm() + "}";
  server.send(200, "application/json", json);
}

void setup() {
  // Lo primerísimo, antes de cualquier otra cosa: forzar los 4 pines del
  // L298N a LOW apenas arranca el sketch (antes de ledcAttachPin). Esto NO
  // evita que un motor gire DURANTE la subida del firmware (ahí el ESP32
  // está en el bootloader, sin ejecutar nuestro código todavía; eso se
  // arregla con una resistencia pull-down de 10k a GND en cada pin IN del
  // L298N), pero sí achica al mínimo la ventana posterior, entre que
  // termina la subida y el PWM queda configurado.
  pinMode(MOTOR_A_ADELANTE, OUTPUT);
  pinMode(MOTOR_A_REVERSA, OUTPUT);
  pinMode(MOTOR_B_ADELANTE, OUTPUT);
  pinMode(MOTOR_B_REVERSA, OUTPUT);
  digitalWrite(MOTOR_A_ADELANTE, LOW);
  digitalWrite(MOTOR_A_REVERSA, LOW);
  digitalWrite(MOTOR_B_ADELANTE, LOW);
  digitalWrite(MOTOR_B_REVERSA, LOW);

  Serial.begin(115200);
  delay(500);
  Serial.println("Iniciando rover con control web + HC-SR04...");
  Serial.print("Motivo del ultimo reinicio: ");
  Serial.println(motivoReinicio());

  ledcSetup(CH_MOTOR_A_ADELANTE, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_MOTOR_A_REVERSA, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_MOTOR_B_ADELANTE, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_MOTOR_B_REVERSA, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_A_ADELANTE, CH_MOTOR_A_ADELANTE);
  ledcAttachPin(MOTOR_A_REVERSA, CH_MOTOR_A_REVERSA);
  ledcAttachPin(MOTOR_B_ADELANTE, CH_MOTOR_B_ADELANTE);
  ledcAttachPin(MOTOR_B_REVERSA, CH_MOTOR_B_REVERSA);

  liberarTodo();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Servo SG90: usar solo los timers 2-3 (los 0-1 son de los motores), 50 Hz, pulsos 500-2400 us
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoBandeja.setPeriodHertz(50);
  servoBandeja.attach(SERVO_PIN, SERVO_PULSO_MIN_US, SERVO_PULSO_MAX_US);
  // Sin write() acá: el servo no se mueve al encender; recién actúa en la primera descarga.
  pinMode(PIN_BOTON_VOLCAR, INPUT_PULLUP);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP iniciado. IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/api/dir", handleDir);
  server.on("/api/power", handlePower);
  server.on("/api/auto", handleAuto);
  server.on("/api/volcar", handleVolcar);
  server.on("/api/status", handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();
  leerBotonVolcar();
  actualizarBandeja();

  // El servo y los motores a fondo tirando corriente al mismo tiempo es lo
  // que más hace caer la tensión del powerbank y provoca brownouts (el
  // ESP32 se resetea solo). Mientras la bandeja está volcando, se frenan
  // los motores (en cualquier modo) para no sumar los dos picos de
  // corriente; el comando pendiente se retoma solo cuando termina.
  if (estadoBandeja == BANDEJA_DESCARGANDO) {
    frenarTodo();
  } else if (estadoActual == MANUAL_WEB) {
    aplicarComandoManual();
  } else if (estadoActual == AUTONOMO_OBSTACULO) {
    ejecutarNavegacionAutonoma();
  }
}

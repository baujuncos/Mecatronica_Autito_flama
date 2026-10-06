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

  Todo el loop() es no bloqueante (sin delay), usando millis() donde hace
  falta temporizar. La única excepción acotada es pulseIn() del HC-SR04,
  que bloquea como máximo ECO_TIMEOUT_US y solo se llama cada
  INTERVALO_LECTURA_MS, no en cada vuelta del loop.
  ======================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

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
const int PWM_MAX = 255;

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

// ---------- Velocidades ----------
const int VELOCIDAD_MANUAL    = 255;
const int VELOCIDAD_GIRO      = 255;
const int VELOCIDAD_AUTONOMO  = 230;

// ponytail: compensación fija por desbalance entre ruedas, en 0 para que
// ambas reciban la misma potencia con el L298N. Subir este valor solo si el
// rover se va a un lado al ir recto.
const int TRIM_IZQUIERDA = 15;

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

// ---------- Navegación autónoma (esquiva obstáculos con el HC-SR04) ----------
// Avanza en línea recta; si detecta un obstáculo a menos de
// UMBRAL_DISTANCIA_CM, gira hacia la derecha hasta despejar el camino. La
// lectura del sensor se limita a una vez cada INTERVALO_LECTURA_MS (en vez
// de cada vuelta del loop) para no bloquear tan seguido con pulseIn().
void ejecutarNavegacionAutonoma() {
  static unsigned long ultimaLectura = 0;
  static long distanciaCm = -1;

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

<script>
let power = false;
let auto = false;

function cmd(d) {
  fetch('/api/dir?d=' + d);
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
  } else if (!activar && estadoActual == AUTONOMO_OBSTACULO) {
    frenarTodo();
    liberarTodo();
    estadoActual = MANUAL_WEB;
    comandoManual = 's';
  }
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
  Serial.begin(115200);
  delay(500);
  Serial.println("Iniciando rover con control web + HC-SR04...");

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

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP iniciado. IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/api/dir", handleDir);
  server.on("/api/power", handlePower);
  server.on("/api/auto", handleAuto);
  server.on("/api/status", handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();

  if (estadoActual == MANUAL_WEB) {
    aplicarComandoManual();
  } else if (estadoActual == AUTONOMO_OBSTACULO) {
    ejecutarNavegacionAutonoma();
  }
}

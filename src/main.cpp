/*
  ======================================================================
  Rover diferencial - ESP32 + L9110S + TCRT5000 (3 cables) + Control Web
  PlatformIO - framework arduino-esp32 core v2.x (ledcSetup/ledcAttachPin)
  ======================================================================
  Modos (máquina de estados):
    - APAGADO       : motores deshabilitados, ignora comandos de dirección.
    - MANUAL_WEB    : el rover obedece los botones del panel web.
    - AUTONOMO_LINEA: el rover sigue línea usando el TCRT5000, ignorando
                      los botones de dirección (solo "Detener" lo saca de
                      este modo).

  Todo el loop() es no bloqueante (sin delay), usando millis() donde hace
  falta temporizar.
  ======================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ---------- Pines Motor A (según serigrafía del módulo: A-1A / A-1B) ----------
const int MOTOR_A_ADELANTE = 26;   // -> A-1B
const int MOTOR_A_REVERSA  = 25;   // -> A-1A

// ---------- Pines Motor B (según serigrafía del módulo: B-1A / B-2A) ----------
const int MOTOR_B_ADELANTE = 27;   // -> B-1A
const int MOTOR_B_REVERSA  = 14;   // -> B-2A

// ---------- Parámetros PWM ----------
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;

const int CH_MOTOR_A_ADELANTE = 0;
const int CH_MOTOR_A_REVERSA  = 1;
const int CH_MOTOR_B_ADELANTE = 2;
const int CH_MOTOR_B_REVERSA  = 3;

// ---------- Sensor de línea TCRT5000 (ensamblaje discreto de 3 cables) ----------
// El sensor artesanal trae 3 cables: VCC (cable blanco largo, a 3V3 preferiblemente o 5V del ESP32),
// GND (cable blanco corto, a masa común con el ESP32 y el driver L9110S) y SEÑAL
// (el cable violeta que sale del punto medio del divisor formado por el
// fototransistor y su resistencia de polarización, con salida analógica
// 0-3.3V). SEÑAL se conecta a un pin ADC1 del ESP32 (GPIO34, solo entrada,
// no interfiere con WiFi). GPIO32 es el pin alternativo si se requiere otro.
const int TCRT_PIN = 34;

// ponytail: umbral fijo, no calibración automática — ajustar este valor
// probando con analogRead() sobre la superficie real (fondo claro vs línea
// oscura); la lectura puede subir o bajar según cómo se haya armado el
// divisor resistivo del sensor.
const int UMBRAL_TCRT = 2000; // 0-4095 (ADC de 12 bits)

// ---------- Bandeja volcable (servo SG90) ----------
// El servo va acostado sobre el chasis y su brazo (hornilla) sube en
// perpendicular sosteniendo la bandeja. Reposo = bandeja paralela al suelo;
// descarga = bandeja perpendicular al suelo (vuelca el contenido).
const int SERVO_PIN = 13;            // GPIO13 (lado izquierdo): sin función especial al arrancar
const int PIN_BOTON_VOLCAR = 33;     // GPIO33 (lado izquierdo); disparador físico opcional (a GND, con pull-up interno)

// Pulsos típicos del SG90 a 50 Hz. Si el servo zumba o topa al llegar al
// extremo, acortar el rango (p. ej. 600-2300); no se recomienda salir de 500-2400.
const int SERVO_PULSO_MIN_US = 500;  // ancho de pulso que corresponde a 0°
const int SERVO_PULSO_MAX_US = 2400; // ancho de pulso que corresponde a 180°

// AFINACIÓN DE NIVELACIÓN: son los dos únicos valores a tocar para calibrar.
//  - ANGULO_REPOSO: si la bandeja queda inclinada al transportar, subir o
//    bajar de a 1-2° (rango válido 0-180) hasta que quede paralela al suelo.
//    Si la hornilla quedó montada con un pequeño desfase, este valor lo
//    compensa sin desarmar nada.
//  - ANGULO_DESCARGA: el servo gira de 180° hacia 90°, así que si la bandeja no
//    llega a la vertical, disminuir (88, 85...); si se pasa, aumentar (92, 95...).
// Para invertir el sentido de giro: intercambiar los dos valores (0 <-> 180)
// y remontar la hornilla del lado opuesto.
const int ANGULO_REPOSO   = 180;
const int ANGULO_DESCARGA = 90;

const unsigned long TIEMPO_DESCARGA_MS = 2000; // pausa arriba para que caiga lo depositado

// Los timers LEDC 0 y 1 los usan los PWM de los motores (canales 0-3, 20 kHz).
// En setup() se restringe ESP32Servo a los timers 2 y 3 para no pisarlos.
Servo servoBandeja;

enum EstadoBandeja { BANDEJA_REPOSO, BANDEJA_DESCARGANDO };
EstadoBandeja estadoBandeja = BANDEJA_REPOSO;
unsigned long inicioDescargaMs = 0;

// ---------- Velocidades ----------
const int VELOCIDAD_MANUAL    = 200;
const int VELOCIDAD_GIRO      = 180;
const int VELOCIDAD_AUTONOMO  = 150;

// ponytail: compensación fija por desbalance mecánico entre ruedas (la
// izquierda gira más lento que la derecha a igual PWM). Solo se aplica al
// ir recto (avanzar/retroceder), no a los giros. Subir/bajar este valor a
// mano hasta que el rover ande derecho.
const int TRIM_IZQUIERDA = 27;

// ---------- WiFi AP ----------
const char *AP_SSID = "Rover-ESP32";
const char *AP_PASS = "12345678";
WebServer server(80);

// ---------- Máquina de estados ----------
enum EstadoRover { APAGADO, MANUAL_WEB, AUTONOMO_LINEA };
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

void avanzar(int velocidad) {
  motorIzquierdo(min(velocidad + TRIM_IZQUIERDA, PWM_MAX));
  motorDerecho(velocidad);
}

void retroceder(int velocidad) {
  motorIzquierdo(-min(velocidad + TRIM_IZQUIERDA, PWM_MAX));
  motorDerecho(-velocidad);
}

// Frenado activo en ambos motores (las 4 entradas en HIGH)
void frenarTodo() {
  ledcWrite(CH_MOTOR_A_ADELANTE, PWM_MAX);
  ledcWrite(CH_MOTOR_A_REVERSA, PWM_MAX);
  ledcWrite(CH_MOTOR_B_ADELANTE, PWM_MAX);
  ledcWrite(CH_MOTOR_B_REVERSA, PWM_MAX);
}

// Punto muerto (las 4 entradas en LOW)
void liberarTodo() {
  motorIzquierdo(0);
  motorDerecho(0);
}

// Giro sobre el propio eje hacia la izquierda
void girarIzquierda(int velocidad) {
  motorIzquierdo(-velocidad);
  motorDerecho(velocidad);
}

// Giro sobre el propio eje hacia la derecha
void girarDerecha(int velocidad) {
  motorIzquierdo(velocidad);
  motorDerecho(-velocidad);
}

// ---------- Bandeja: secuencia inclinar -> pausa -> retorno ----------

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

// ---------- Navegación autónoma (un solo sensor TCRT5000) ----------
// Arranca con la línea negra debajo del sensor. Con un único sensor
// centrado no se puede saber hacia qué lado se curvó la línea, así que la
// estrategia simple es: sensor sobre la línea -> avanzar recto; sensor
// perdió la línea (ve el fondo blanco) -> pivotar (buscar) hacia la
// derecha hasta recuperarla. Se re-evalúa en cada vuelta del loop, sin
// delay(), así que la corrección es inmediata. Sirve para curvas suaves;
// en esquinas muy puntiagudas el pivot puede no alcanzar a recuperar la
// línea a tiempo (ver README).
void ejecutarNavegacionAutonoma() {
  int lectura = analogRead(TCRT_PIN);
  bool sobreLinea = lectura > UMBRAL_TCRT;

  if (sobreLinea) {
    avanzar(VELOCIDAD_AUTONOMO);
  } else {
    girarDerecha(VELOCIDAD_GIRO);
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
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<meta name="theme-color" content="#0f1420">
<title>Rover ESP32</title>
<style>
  :root {
    --bg:#0f1420; --card:#181f30; --btn:#232c42; --btn-hi:#2f3b58;
    --txt:#e8ecf5; --dim:#8b95ad; --ok:#3ddc97; --warn:#ffb648; --off:#5b667f;
    --accent:#4c8dff; --stop:#ff5d6c;
  }
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  body { margin:0; background:var(--bg); color:var(--txt); font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
         min-height:100vh; display:flex; justify-content:center; padding:16px; }
  main { width:100%; max-width:420px; display:flex; flex-direction:column; gap:14px; }
  header { display:flex; align-items:center; justify-content:space-between; padding:2px 4px; }
  h1 { font-size:1.15rem; margin:0; letter-spacing:.3px; }
  .conn { display:flex; align-items:center; gap:7px; font-size:.8rem; color:var(--dim); }
  .dot { width:9px; height:9px; border-radius:50%; background:var(--stop); }
  .dot.on { background:var(--ok); box-shadow:0 0 8px var(--ok); }
  .card { background:var(--card); border-radius:18px; padding:16px; }
  .label { font-size:.72rem; text-transform:uppercase; letter-spacing:1px; color:var(--dim); margin-bottom:10px; }
  .status { display:flex; align-items:center; justify-content:space-between; gap:12px; }
  .badge { padding:6px 14px; border-radius:999px; font-weight:700; font-size:.85rem; background:var(--off); color:#fff; }
  .badge.manual { background:var(--accent); }
  .badge.auto { background:var(--ok); color:#06281a; }
  .sensor { text-align:right; }
  .sensor b { font-size:1.5rem; font-variant-numeric:tabular-nums; }
  .sensor small { display:block; color:var(--dim); font-size:.7rem; }
  .bar { height:6px; background:var(--btn); border-radius:3px; margin-top:12px; overflow:hidden; }
  .bar i { display:block; height:100%; width:0; background:var(--accent); transition:width .3s; }
  .dpad { display:grid; grid-template-columns:repeat(3,1fr); gap:10px; max-width:270px; margin:0 auto; }
  .dpad button { aspect-ratio:1; font-size:1.6rem; }
  .dpad.bloq { opacity:.35; pointer-events:none; }
  button { border:none; border-radius:16px; background:var(--btn); color:var(--txt); font:inherit; font-weight:600;
           transition:transform .08s, background .15s; touch-action:manipulation; }
  button:active { transform:scale(.94); background:var(--btn-hi); }
  .stop { background:var(--stop); color:#fff; }
  .stop:active { background:#e04858; }
  .row { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
  .row button { padding:16px 8px; font-size:.95rem; display:flex; flex-direction:column; align-items:center; gap:4px; }
  .row button span { font-size:1.4rem; line-height:1; }
  .row button.on { background:var(--ok); color:#06281a; }
  .row button.off { opacity:.4; pointer-events:none; }
  .bandeja { width:100%; padding:16px; font-size:1rem; background:var(--warn); color:#3a2500; }
  .bandeja:active { background:#e6a13f; }
  .bandeja.activo { background:var(--btn); color:var(--dim); pointer-events:none; }
  footer { text-align:center; font-size:.7rem; color:var(--dim); }
</style>
</head>
<body>
<main>
  <header>
    <h1>Rover ESP32</h1>
    <div class="conn"><span class="dot" id="dot"></span><span id="conn">Conectando...</span></div>
  </header>

  <section class="card">
    <div class="label">Estado</div>
    <div class="status">
      <span class="badge" id="badge">--</span>
      <div class="sensor"><b id="sensor">--</b><small>sensor de línea</small></div>
    </div>
    <div class="bar"><i id="barra"></i></div>
  </section>

  <section class="card">
    <div class="label">Movimiento</div>
    <div class="dpad bloq" id="dpad">
      <div></div><button onclick="cmd('fwd')">&#9650;</button><div></div>
      <button onclick="cmd('left')">&#9664;</button>
      <button class="stop" onclick="cmd('stop')">&#9632;</button>
      <button onclick="cmd('right')">&#9654;</button>
      <div></div><button onclick="cmd('back')">&#9660;</button><div></div>
    </div>
  </section>

  <section class="row">
    <button id="btnPower" onclick="togglePower()"><span>&#9211;</span>Encender</button>
    <button id="btnAuto" class="off" onclick="toggleAuto()"><span>&#10138;</span>Seguir línea</button>
  </section>

  <button class="bandeja" id="btnBandeja" onclick="volcar()">Volcar bandeja</button>

  <footer id="pie"></footer>
</main>

<script>
let estado = 'APAGADO';
const $ = id => document.getElementById(id);

function cmd(d) { fetch('/api/dir?d=' + d); }

function togglePower() {
  fetch('/api/power?v=' + (estado === 'APAGADO' ? 1 : 0)).then(refrescar);
}

function toggleAuto() {
  fetch('/api/auto?v=' + (estado === 'AUTONOMO_LINEA' ? 0 : 1)).then(refrescar);
}

function volcar() {
  const b = $('btnBandeja');
  fetch('/api/volcar');
  b.classList.add('activo');
  b.innerText = 'Volcando...';
  setTimeout(() => { b.classList.remove('activo'); b.innerText = 'Volcar bandeja'; }, 2500);
}

function refrescar() {
  fetch('/api/status').then(r => r.json()).then(s => {
    estado = s.estado;
    const apagado = estado === 'APAGADO', auto = estado === 'AUTONOMO_LINEA';

    $('dot').className = 'dot on';
    $('conn').innerText = 'Conectado';

    const badge = $('badge');
    badge.innerText = apagado ? 'Apagado' : auto ? 'Autónomo' : 'Manual';
    badge.className = 'badge' + (apagado ? '' : auto ? ' auto' : ' manual');

    $('sensor').innerText = s.sensor;
    $('barra').style.width = (s.sensor / 4095 * 100) + '%';

    $('dpad').className = 'dpad' + (estado === 'MANUAL_WEB' ? '' : ' bloq');

    const p = $('btnPower');
    p.className = apagado ? '' : 'on';
    p.innerHTML = '<span>&#9211;</span>' + (apagado ? 'Encender' : 'Apagar');

    const a = $('btnAuto');
    a.className = apagado ? 'off' : auto ? 'on' : '';
    a.innerHTML = '<span>&#10138;</span>' + (auto ? 'Detener línea' : 'Seguir línea');

    $('pie').innerText = 'Último inicio: ' + s.reinicio;
  }).catch(() => {
    $('dot').className = 'dot';
    $('conn').innerText = 'Sin conexión';
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
  if (d == "fwd") avanzar(VELOCIDAD_MANUAL);
  else if (d == "back") retroceder(VELOCIDAD_MANUAL);
  else if (d == "left") girarIzquierda(VELOCIDAD_GIRO);
  else if (d == "right") girarDerecha(VELOCIDAD_GIRO);
  else if (d == "stop") frenarTodo();
  server.send(200, "text/plain", "ok");
}

void handlePower() {
  bool encender = server.arg("v") == "1";
  if (encender && estadoActual == APAGADO) {
    estadoActual = MANUAL_WEB;
  } else if (!encender) {
    liberarTodo();
    estadoActual = APAGADO;
  }
  server.send(200, "text/plain", "ok");
}

void handleAuto() {
  bool activar = server.arg("v") == "1";
  if (activar && estadoActual == MANUAL_WEB) {
    estadoActual = AUTONOMO_LINEA;
  } else if (!activar && estadoActual == AUTONOMO_LINEA) {
    frenarTodo();
    liberarTodo();
    estadoActual = MANUAL_WEB;
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
    estadoActual == AUTONOMO_LINEA ? "AUTONOMO_LINEA" : "MANUAL_WEB";
  String json = String("{\"estado\":\"") + nombre + "\",\"sensor\":" + analogRead(TCRT_PIN) + ",\"reinicio\":\"" + motivoReinicio() + "\"}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Iniciando rover con control web + TCRT5000...");
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

  analogReadResolution(12); // ESP32: 0-4095
  pinMode(TCRT_PIN, INPUT);

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

  if (estadoActual == AUTONOMO_LINEA) {
    ejecutarNavegacionAutonoma();
  }
}

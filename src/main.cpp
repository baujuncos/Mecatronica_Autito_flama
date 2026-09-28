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

// ---------- Navegación autónoma (un solo sensor TCRT5000) ----------
// Con un único sensor centrado no se puede saber de qué lado quedó la
// línea, así que la estrategia simple es: superficie clara -> avanzar
// recto; línea oscura detectada -> pivotar (buscar) hacia la derecha
// hasta recuperar la superficie clara. Se re-evalúa en cada vuelta del
// loop, sin delay(), así que la corrección es inmediata.
void ejecutarNavegacionAutonoma() {
  int lectura = analogRead(TCRT_PIN);
  bool superficieClara = lectura < UMBRAL_TCRT;

  if (superficieClara) {
    avanzar(VELOCIDAD_AUTONOMO);
  } else {
    girarDerecha(VELOCIDAD_GIRO);
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
    <button id="btnAuto" onclick="toggleAuto()">Modo Autónomo</button>
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
    auto = s.estado === 'AUTONOMO_LINEA';
    document.getElementById('estado').innerText = 'Estado: ' + s.estado + ' | Sensor: ' + s.sensor;
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

void handleStatus() {
  const char *nombre =
    estadoActual == APAGADO ? "APAGADO" :
    estadoActual == AUTONOMO_LINEA ? "AUTONOMO_LINEA" : "MANUAL_WEB";
  String json = String("{\"estado\":\"") + nombre + "\",\"sensor\":" + analogRead(TCRT_PIN) + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Iniciando rover con control web + TCRT5000...");

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

  if (estadoActual == AUTONOMO_LINEA) {
    ejecutarNavegacionAutonoma();
  }
}

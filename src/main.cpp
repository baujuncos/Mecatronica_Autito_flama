/*
  ======================================================================
  Control de 2 motores DC (chasis diferencial) con módulo dual L9110S
  ESP32 + PlatformIO - Framework arduino-esp32 core v3.x
  ======================================================================
  Secuencia de prueba:
    1. Ambas ruedas ADELANTE, rampa de 0% a 100%
    2. Detiene 2 segundos
    3. Ambas ruedas en REVERSA, rampa de 0% a 100%
    4. Frenado final
 
  Nota: este código mueve ambos motores IGUAL (adelante/atrás en línea recta).
  Para girar (diferencial), hay que dar velocidades distintas a cada rueda
  -> ver funciones girarIzquierda() / girarDerecha() al final, listas para usar.
  ======================================================================
*/

#include <Arduino.h>

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

// El core de arduino-esp32 instalado (2.0.17) usa LEDC por canal, no por pin.
const int CH_MOTOR_A_ADELANTE = 0;
const int CH_MOTOR_A_REVERSA  = 1;
const int CH_MOTOR_B_ADELANTE = 2;
const int CH_MOTOR_B_REVERSA  = 3;

// ---------- Funciones de control individual ----------

// Motor izquierdo (conectado a la salida "Motor A" de la placa)
// velocidad positiva = adelante, negativa = reversa, 0 = libre
void motorIzquierdo(int velocidad) {
  if (velocidad > 0) {
    ledcWrite(CH_MOTOR_A_ADELANTE, velocidad);
    ledcWrite(CH_MOTOR_A_REVERSA, 0);
  } else if (velocidad < 0) {
    ledcWrite(CH_MOTOR_A_ADELANTE, 0);
    ledcWrite(CH_MOTOR_A_REVERSA, -velocidad);  // se pasa el valor absoluto
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
  motorIzquierdo(velocidad);
  motorDerecho(velocidad);
}

void retroceder(int velocidad) {
  motorIzquierdo(-velocidad);
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
 
// Giro sobre el propio eje hacia la izquierda (rueda izq. atrás, rueda der. adelante)
void girarIzquierda(int velocidad) {
  motorIzquierdo(-velocidad);
  motorDerecho(velocidad);
}
 
// Giro sobre el propio eje hacia la derecha (rueda izq. adelante, rueda der. atrás)
void girarDerecha(int velocidad) {
  motorIzquierdo(velocidad);
  motorDerecho(-velocidad);
}
 
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Iniciando control de 2 motores L9110S...");
 
  // Configuramos los 4 canales PWM y los asociamos a sus pines
  ledcSetup(CH_MOTOR_A_ADELANTE, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_MOTOR_A_REVERSA, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_MOTOR_B_ADELANTE, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_MOTOR_B_REVERSA, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_A_ADELANTE, CH_MOTOR_A_ADELANTE);
  ledcAttachPin(MOTOR_A_REVERSA, CH_MOTOR_A_REVERSA);
  ledcAttachPin(MOTOR_B_ADELANTE, CH_MOTOR_B_ADELANTE);
  ledcAttachPin(MOTOR_B_REVERSA, CH_MOTOR_B_REVERSA);
 
  liberarTodo();
}
 
void loop() {
  // ---------- 1. ADELANTE: rampa 0% -> 100% ----------
  Serial.println("Avanzando, aumentando velocidad...");
  for (int duty = 0; duty <= PWM_MAX; duty++) {
    avanzar(duty);
    delay(15);
  }
 
  // ---------- 2. DETENER 2 segundos ----------
  Serial.println("Deteniendo 2 segundos...");
  frenarTodo();
  delay(2000);
 
  // ---------- 3. REVERSA: rampa 0% -> 100% ----------
  Serial.println("Retrocediendo, aumentando velocidad...");
  for (int duty = 0; duty <= PWM_MAX; duty++) {
    retroceder(duty);
    delay(15);
  }
 
  // ---------- 4. Frenado final ----------
  Serial.println("Ciclo completo. Frenando.");
  frenarTodo();
 
  while (true) {
    delay(1000);
  }
}
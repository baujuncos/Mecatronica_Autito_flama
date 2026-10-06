# Mecatronica_Autito_flama

Rover diferencial con ESP32, driver dual L9110S, sensor ultrasónico HC-SR04
(detección de obstáculos) y control remoto vía panel web (WiFi SoftAP).
Proyecto para la materia **Mecatrónica**.

## Estado del proyecto

- [x] Control de 2 motores DC (chasis diferencial) vía L9110S, con PWM (LEDC).
- [x] Sensor ultrasónico HC-SR04 para detección/esquive de obstáculos.
- [x] Panel web (WiFi AP + WebServer) con controles manuales, ON/OFF y modo autónomo.
- [x] Máquina de estados: `APAGADO`, `MANUAL_WEB`, `AUTONOMO_OBSTACULO` (loop no bloqueante; única excepción acotada: `pulseIn()` del HC-SR04, con timeout de 25ms y muestreado cada 60ms, no en cada vuelta del loop).
- [ ] **Pendiente**: integración del servomotor con la bandeja/soporte impreso en
      impresora 3D (mecanismo aún no diseñado/impreso). Se abordará en una
      siguiente etapa del proyecto.

> **Historial**: la primera versión del proyecto usaba un sensor de línea
> TCRT5000 (3 cables) para seguir una cinta negra. Se reemplazó por el
> HC-SR04, cambiando el modo autónomo de "seguir línea" a "esquivar
> obstáculos".

## Esquema de conexiones

Masa común: **todos los GND** (ESP32, driver L9110S, sensor HC-SR04 y
batería/fuente de los motores) deben unirse en un mismo punto de referencia.

### ESP32 <-> Driver L9110S

| ESP32 (GPIO) | L9110S       | Función           |
|---------------|--------------|-------------------|
| GPIO26        | A-1B         | Motor A adelante  |
| GPIO25        | A-1A         | Motor A reversa   |
| GPIO27        | B-1A         | Motor B adelante  |
| GPIO14        | B-2A         | Motor B reversa   |
| GND           | GND          | Masa común        |

### L9110S <-> Motores y alimentación

| L9110S     | Destino                                   |
|------------|--------------------------------------------|
| VCC        | Positivo de batería/fuente de motores (NO el 3V3 del ESP32) |
| GND        | Negativo de batería/fuente **y** GND del ESP32 (masa común) |
| A-1A/A-1B  | Motor izquierdo (2 cables)                 |
| B-1A/B-2A  | Motor derecho (2 cables)                   |

### ESP32 <-> Sensor ultrasónico HC-SR04

| HC-SR04 | Conexión                                  | Nota |
|---------|---------------------------------------------|------|
| VCC     | **3V3** del ESP32 (no 5V/VIN)                | Ver nota de alimentación abajo |
| GND     | GND (masa común)                             | |
| TRIG    | GPIO32                                       | Salida del ESP32 hacia el sensor |
| ECHO    | GPIO34, directo, sin divisor                 | Seguro porque el sensor corre a 3.3V |

**Nota de alimentación**: el HC-SR04 está especificado para 4.5-5.5V, pero
alimentado a 3.3V (VCC del sensor al pin 3V3 del ESP32) funciona en la
práctica en la mayoría de los módulos, y como corre a 3.3V, el pulso de
ECHO también sale a 3.3V — se puede conectar directo al GPIO sin divisor
resistivo ni componentes extra. La contra es que el alcance máximo baja de
~4m a ~1.5-2m aprox., lecturas algo menos estables. Para este proyecto
(detectar un obstáculo a 15cm) sobra margen. Si notan lecturas erráticas o
intermitentes una vez armado, la solución de respaldo es alimentar a 5V
(VIN) y agregar un divisor resistivo en ECHO (1kΩ en serie + 2kΩ a GND)
antes del GPIO.

GPIO34 se eligió para ECHO porque es un pin **solo-entrada** de ADC1 (no
necesita salida). GPIO32 para TRIG es un pin de uso general sin restricciones.

`UMBRAL_DISTANCIA_CM` (en `main.cpp`, valor por defecto 15cm) define a qué
distancia el rover considera que hay un obstáculo y empieza a esquivarlo;
ajustar según el tamaño de la pista y la velocidad de reacción deseada.

## Panel de control web

1. El ESP32 crea una red WiFi propia: SSID `Rover-ESP32`, contraseña `12345678`.
2. Conectarse a esa red desde el celular o PC y abrir `http://192.168.4.1/`.
3. Controles disponibles: Adelante / Atrás / Izquierda / Derecha / Frenar,
   botón ON/OFF (habilita/deshabilita motores) y botón "Esquivar Obstáculos"
   (modo autónomo con el HC-SR04; al detenerlo vuelve a modo manual). El
   panel muestra la distancia medida en cm en tiempo real.

## Compilar y subir (PlatformIO)

```
pio run -t upload
pio device monitor
```

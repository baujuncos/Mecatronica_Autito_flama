# Mecatronica_Autito_flama

Rover diferencial con ESP32, driver dual L9110S, sensor de línea TCRT5000
(ensamblaje discreto de 3 cables) y control remoto vía panel web (WiFi
SoftAP). Proyecto para la materia **Mecatrónica**.

## Estado del proyecto

- [x] Control de 2 motores DC (chasis diferencial) vía L9110S, con PWM (LEDC).
- [x] Sensor de línea TCRT5000 (3 cables) leído por ADC.
- [x] Panel web (WiFi AP + WebServer) con controles manuales, ON/OFF y modo autónomo.
- [x] Máquina de estados: `APAGADO`, `MANUAL_WEB`, `AUTONOMO_LINEA` (loop 100% no bloqueante).
- [ ] **Pendiente**: integración del servomotor con la bandeja/soporte impreso en
      impresora 3D (mecanismo aún no diseñado/impreso). Se abordará en una
      siguiente etapa del proyecto.

## Esquema de conexiones

Masa común: **todos los GND** (ESP32, driver L9110S, sensor TCRT5000 y
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

### ESP32 <-> Sensor TCRT5000 (ensamblaje discreto, 3 cables)

El sensor está armado a mano: un TCRT5000 soldado junto con su resistencia
de polarización, formando un divisor de tensión. De ahí salen solo 3 cables:

| Cable del sensor | Identificación                                             | Conexión ESP32 |
|-------------------|------------------------------------------------------------|----------------|
| VCC               | Va a una pata del TCRT5000 (alimentación del emisor/fototransistor) | 3V3 (o 5V si el divisor fue diseñado para eso) |
| GND               | Va al otro extremo de la resistencia de polarización        | GND (masa común) |
| Señal             | Sale del punto medio entre el fototransistor y la resistencia (nodo intermedio del divisor) | GPIO34 (ADC1, solo entrada) |

**Cómo identificarlos con un multímetro** (sensor desenergizado): mide
continuidad/resistencia entre pares de cables. El par que muestra la
resistencia de polarización fija (ej. 10kΩ) son VCC-GND. El cable restante,
que cae en el punto medio, es la Señal.

GPIO34 se eligió porque es un pin **solo-entrada** de ADC1 (no comparte
recursos con el WiFi, a diferencia del ADC2). GPIO32 es la alternativa
válida si se prefiere otro pin.

`UMBRAL_TCRT` (en `main.cpp`) debe calibrarse imprimiendo `analogRead()`
por Serial sobre la superficie real del circuito (fondo claro vs. línea
oscura), ya que el valor sube o baja según cómo se haya armado el divisor.

## Panel de control web

1. El ESP32 crea una red WiFi propia: SSID `Rover-ESP32`, contraseña `12345678`.
2. Conectarse a esa red desde el celular o PC y abrir `http://192.168.4.1/`.
3. Controles disponibles: Adelante / Atrás / Izquierda / Derecha / Frenar,
   botón ON/OFF (habilita/deshabilita motores) y botón de Modo Autónomo
   (sigue-línea con el TCRT5000; al detenerlo vuelve a modo manual).

## Compilar y subir (PlatformIO)

```
pio run -t upload
pio device monitor
```

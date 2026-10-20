# Mecatronica_Autito_flama

Rover diferencial con ESP32, driver doble puente H L298N, sensor ultrasónico
HC-SR04 (detección de obstáculos), bandeja volcable con servo SG90 y control
remoto vía panel web (WiFi SoftAP). Alimentado con **dos powerbanks de 5V**:
uno para el ESP32/servo/sensor y otro solo para los motores, con GND común
entre ambos. Proyecto para la materia **Mecatrónica**.

## Estado del proyecto

- [X] Control de 2 motores DC (chasis diferencial) vía L298N, con PWM (LEDC).
- [X] Sensor ultrasónico HC-SR04 para detección/esquive de obstáculos.
- [X] Bandeja volcable con servo SG90: botón web, pulsador físico, y disparo
  automático a los 5s de entrar en modo autónomo.
- [X] Panel web (WiFi AP + WebServer) con controles manuales, ON/OFF, modo
  autónomo y volcado de bandeja.
- [X] Máquina de estados: `APAGADO`, `MANUAL_WEB`, `AUTONOMO_OBSTACULO` (loop no bloqueante; única excepción acotada: `pulseIn()` del HC-SR04, con timeout de 25ms y muestreado cada 60ms, no en cada vuelta del loop). La bandeja es independiente de esta máquina de estados: puede volcar en cualquiera de los tres estados.
- [X] Resistencias pull-down (10kΩ a GND) en los 4 pines del L298N (GPIO26/25/27/14), para que ningún motor gire solo mientras el ESP32 está en el bootloader subiendo el firmware.

> **Historial**: la primera versión del proyecto usaba un sensor de línea
> TCRT5000 (3 cables) para seguir una cinta negra. Se reemplazó por el
> HC-SR04, cambiando el modo autónomo de "seguir línea" a "esquivar
> obstáculos". Más adelante se cambió el driver de motores L9110S por un
> L298N, para reducir la caída de tensión al arrancar/girar, y se agregó la
> bandeja volcable con servo SG90.

## Esquema de conexiones

Masa común: **todos los GND** (ESP32, driver L298N, servo, sensor HC-SR04 y
los dos powerbanks) deben unirse en un mismo punto de referencia, aunque
cada powerbank alimente partes distintas del circuito. Sin este GND común,
las señales que manda el ESP32 al L298N no tienen una referencia válida.

### Alimentación: dos powerbanks de 5V

| Powerbank | Alimenta | Conexión |
| --------- | -------- | -------- |
| A (control) | ESP32 (por su cable USB) | — |
| A (control) | Servo SG90 | Un segundo cable USB de esta misma fuente: rojo (5V) al VCC del servo, negro (GND) al GND del servo. No usar el pin 5V/3V3 del ESP32 para esto. |
| B (motores) | L298N | Un cable USB de esta fuente: rojo (5V) al pin +12V/VS del L298N, negro (GND) al GND del L298N. |

El GND del powerbank B (o el GND del L298N) se une con un cable al GND del
powerbank A (o el GND del ESP32). Separar así la fuente de los motores
(consumo con picos fuertes) de la del ESP32 (consumo estable) evita que los
picos de corriente de los motores reseteen el ESP32 por brownout (ver
sección de diagnóstico más abajo). No soluciona la falta de potencia de los
motores (ver nota de alimentación de motores abajo): eso depende del
voltaje y del driver, no de cuántas fuentes se usen.

### ESP32 <-> Driver L298N

| ESP32 (GPIO) | L298N | Función         |
| ------------ | ----- | ---------------- |
| GPIO26       | IN1   | Motor A adelante |
| GPIO25       | IN2   | Motor A reversa  |
| GPIO27       | IN3   | Motor B adelante |
| GPIO14       | IN4   | Motor B reversa  |
| GND          | GND   | Masa común      |

Los jumpers de **ENA** y **ENB** del módulo van puestos (puentes siempre
habilitados): la velocidad se controla por PWM directo sobre IN1..IN4, no
sobre ENA/ENB.

### L298N <-> Motores y alimentación

| L298N     | Destino                                                        |
| --------- | -------------------------------------------------------------- |
| +12V / VS | Positivo del powerbank B (motores). NO el 3V3 ni el 5V del ESP32 |
| GND       | Negativo del powerbank B **y** GND común con el resto del circuito |
| OUT1/OUT2 | Motor izquierdo (2 cables)                                     |
| OUT3/OUT4 | Motor derecho (2 cables)                                       |

**Nota de alimentación**: aunque los motores ya tienen su propio powerbank,
sigue siendo una fuente de 5V. El L298N pierde cerca de 2V entre VS y los
motores, así que éstos reciben bastante menos de 5V reales. Por eso
`PWM_ARRANQUE` y las constantes `VELOCIDAD_*` en `main.cpp` están al máximo
o cerca del máximo (ver sección de calibración más abajo): es la forma de
compensar por software la falta de potencia de esta fuente. Separar los
powerbanks evita que esos picos reseteen el ESP32, pero si se quiere más
par en las ruedas hace falta subir el voltaje de la fuente de motores
(7.4-9V) o cambiar a un driver de menor caída (TB6612FNG); ahí sí se pueden
bajar los valores de `VELOCIDAD_*` y el rover va a responder mejor.

### ESP32 <-> Sensor ultrasónico HC-SR04

| HC-SR04 | Conexión                           | Nota                                 |
| ------- | ----------------------------------- | ------------------------------------ |
| VCC     | **3V3** del ESP32 (no 5V/VIN) | Ver nota de alimentación abajo      |
| GND     | GND (masa común)                   |                                      |
| TRIG    | GPIO32                              | Salida del ESP32 hacia el sensor     |
| ECHO    | GPIO34, directo, sin divisor        | Seguro porque el sensor corre a 3.3V |

**Nota de alimentación**: el HC-SR04 está especificado para 4.5-5.5V, pero
alimentado a 3.3V (VCC del sensor al pin 3V3 del ESP32) funciona en la
práctica en la mayoría de los módulos, y como corre a 3.3V, el pulso de
ECHO también sale a 3.3V — se puede conectar directo al GPIO sin divisor
resistivo ni componentes extra. La contra es que el alcance máximo baja de
~4m a ~1.5-2m aprox., lecturas algo menos estables. Para este proyecto
(detectar un obstáculo a 35cm) sobra margen. Si notan lecturas erráticas o
intermitentes una vez armado, la solución de respaldo es alimentar a 5V
(VIN) y agregar un divisor resistivo en ECHO (1kΩ en serie + 2kΩ a GND)
antes del GPIO.

GPIO34 se eligió para ECHO porque es un pin **solo-entrada** de ADC1 (no
necesita salida). GPIO32 para TRIG es un pin de uso general sin restricciones.

`UMBRAL_DISTANCIA_CM` (en `main.cpp`, valor por defecto 35cm) define a qué
distancia el rover considera que hay un obstáculo y empieza a esquivarlo;
ajustar según el tamaño de la pista y la velocidad de reacción deseada.

### ESP32 <-> Bandeja volcable (servo SG90)

| Pin                 | Conexión | Nota                                                    |
| ------------------- | -------- | -------------------------------------------------------- |
| GPIO13               | Señal del servo | Sin función especial al arrancar                    |
| GPIO33               | Pulsador físico (opcional) | A GND; usa el pull-up interno del ESP32 (`INPUT_PULLUP`) |
| 5V del powerbank A   | VCC del servo | Ver tabla de alimentación más arriba, no usar el 3V3/5V del ESP32 |
| GND común            | GND del servo | |

El servo va acostado sobre el chasis; su hornilla (brazo) sube en
perpendicular sosteniendo la bandeja. Reposo = bandeja paralela al suelo;
descarga = bandeja volcada.

**Disparadores del volcado** (cualquiera de los tres, en cualquier estado del rover):
- Botón "Volcar bandeja" en el panel web.
- Pulsador físico en GPIO33.
- Automático: a los 5 segundos de activar el modo autónomo (`TIEMPO_AUTONOMO_ANTES_DE_VOLCAR_MS` en `main.cpp`), vuelca una sola vez por sesión autónoma y el rover sigue esquivando obstáculos antes y después.

Mientras la bandeja está volcando (2 segundos, `TIEMPO_DESCARGA_MS`), el
rover frena los motores por completo, en cualquier modo. Es a propósito: el
servo y los motores tirando corriente al mismo tiempo es lo que más hace
caer la tensión y puede resetear el ESP32 (brownout).

**Calibración del servo** (en `main.cpp`, dos únicos valores a tocar):
- `ANGULO_REPOSO`: ajustar de a 1-2° hasta que la bandeja quede paralela al suelo en reposo.
- `ANGULO_DESCARGA`: ajustar hasta que la bandeja llegue bien a la vertical al volcar.
- Si el servo vuelca para el lado equivocado, **no alcanza con cambiar estos valores**: hay que remontar físicamente la hornilla del servo girada 180° en el eje (ver comentario en el código), porque el lado hacia el que vuelca depende de para dónde apunta la hornilla montada, no del ángulo que se le mande.

## Calibración de motores (L298N + powerbank de motores)

Al driver le cuesta vencer la fricción estática de las ruedas al arrancar,
sobre todo la rueda derecha. Constantes en `main.cpp` para ajustar si hace falta:

- `PWM_ARRANQUE`: PWM mínimo con el que arranca la rampa de aceleración (en
  vez de arrancar desde 0). Subirlo si alguna rueda sigue sin arrancar sola.
- `VELOCIDAD_MANUAL`, `VELOCIDAD_GIRO`, `VELOCIDAD_AUTONOMO`: velocidades
  objetivo de cada modo. Están altas (algunas al máximo de 255) para
  compensar la caída de tensión del L298N.
- `TRIM_IZQUIERDA`: compensación de potencia entre ruedas al ir recto;
  subir/bajar de a poco si el rover se va para un lado al avanzar derecho.

## Diagnóstico de reinicios (brownout)

Al arrancar, el ESP32 imprime por Serial (115200 baudios) el motivo del
último reinicio, por ejemplo:

```
Iniciando rover con control web + HC-SR04...
Motivo del ultimo reinicio: BROWNOUT (caida de tension)
```

Si dice `BROWNOUT`, el ESP32 se reseteó solo por una caída de tensión (lo
más probable: motores y/o servo pidiendo más corriente de la que la fuente
puede entregar en ese instante). Si dice `POWERON`, fue un encendido normal.
Para revisarlo en vivo, dejar `pio device monitor` abierto mientras el
rover anda con sus powerbanks.

## Panel de control web

1. El ESP32 crea una red WiFi propia: SSID `Rover-ESP32`, contraseña `12345678`.
2. Conectarse a esa red desde el celular o PC y abrir `http://192.168.4.1/`.
3. Controles disponibles: Adelante / Atrás / Izquierda / Derecha / Frenar,
   botón ON/OFF (habilita/deshabilita motores), botón "Esquivar Obstáculos"
   (modo autónomo con el HC-SR04; al detenerlo vuelve a modo manual) y botón
   "Volcar bandeja" (dispara el servo en cualquier estado). El panel muestra
   la distancia medida en cm en tiempo real.

## Compilar y subir (PlatformIO)

```
pio run -t upload
pio device monitor
```

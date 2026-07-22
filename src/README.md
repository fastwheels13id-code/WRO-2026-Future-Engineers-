# Control Software

# Code Architecture & Firmware Explanation

## 1. Perception Layer: ESP32-CAM (`esp32_vision.ino`)

The perception subsystem is responsible for acquiring raw visual data from the track environment, transforming color spaces, and detecting target track obstacles using horizontal spatial division.

> **Important Note on System Architecture:**  
> The provided vision firmware currently handles local frame processing and outputs diagnostics directly over `Serial` (9600 baud) for debugging purposes. **Serial communication (TX/RX) between the ESP32-CAM and the central LAFVIN R3 controller has not yet been integrated into this version.** Future revisions will transmit compact UART byte tokens instead of human-readable debug messages.

### Key Technical Subsystems

#### Camera Configuration & Acquisition

Initializes the OV2640 sensor using a **20 MHz XCLK** clock with a **CIF resolution (400 × 296 pixels)**. Images are captured in `PIXFORMAT_JPEG` format to optimize DMA buffer transfers.

#### Frame Decompression & RGB565 Parsing

Raw JPEG frames (`camera_fb_t`) are decompressed into a temporary **16-bit RGB565** buffer using `jpg2rgb565()`. Color channels are extracted through bitwise operations:

```math
R = ((pixel \gg 11) \& 0x1F) \ll 3
```

```math
G = ((pixel \gg 5) \& 0x3F) \ll 2
```

```math
B = (pixel \& 0x1F) \ll 3
```

#### Color Transformation (`rgbToHsv`)

RGB values are normalized to the interval

```math
[0,1]
```

and converted into the HSV (Hue, Saturation, Value) color space. Separating **Hue** from **Saturation** and **Value** improves robustness against illumination changes.

#### Performance Optimization (`PIXEL_STEP`)

To maintain an acceptable frame rate, the image is sampled every **2 pixels** (`PIXEL_STEP = 2`) while ignoring the outer borders:

```math
10 \le X \le Width - 10
```

```math
10 \le Y \le Height - 10
```

#### Color Classification & Centroid Calculation

Pixels whose Hue falls inside predefined ranges (Red, Green, Magenta, Blue and Orange) increment color counters and accumulate their coordinates.

Whenever the number of detected pixels exceeds

```math
threshold = 130
```

the centroid is computed as

```math
Center_X = \frac{\sum X}{N}
```

where:

- \( \sum X \) = accumulated horizontal coordinates
- \( N \) = number of valid pixels

#### Horizontal Zone Classification (`obtenerPosicion`)

The 400-pixel horizontal image is divided into three equal regions:

```math
400 / 3 \approx 133 \text{ pixels}
```

The detected object is classified as:

- `IZQUIERDA` (Left)
- `CENTRO` (Center)
- `DERECHA` (Right)

depending on the horizontal centroid location.

---

# 2. Locomotion & Actuation Layer: Central Controller (`motor_servo_control.ino`)

This module manages the vehicle's steering servo and traction motor, providing high-level abstractions for vehicle motion.

## Hardware Mapping & Constants

### Steering Servo (Pin 9)

Controlled through the `Servo.h` library.

- `ANGULO_RECTO = 76°` → Straight alignment
- `ANGULO_DER = 110°` → Maximum safe right steering angle
- `ANGULO_IZQ = 35°` → Maximum safe left steering angle

### DC Traction Motor (Pins 4 & 5)

Controlled through an H-Bridge driver.

- `PIN_DIR_MOTOR_A = 4` → Direction control (`LOW` = Forward, `HIGH` = Reverse)
- `PIN_PWM_MOTOR_A = 5` → PWM speed control (0–255)
- `VEL_RAPIDA = 190` → Cruise speed
- `VEL_NORMAL = 150` → Maneuvering speed

## Functional Breakdown

### Low-Level Drivers

- `moverDireccion(int angulo)`  
  Sends the requested steering angle directly to the MG996R servo.

- `controlarMotorA(boolean direccion, int velocidad)`  
  Configures motor direction and outputs the corresponding PWM signal.

### High-Level Motion Commands

- `avanzarRecto()`
- `girarIzquierda()`
- `girarDerecha()`

These functions provide predefined steering positions.

Motor commands include:

- `motorA_Adelante(vel)`
- `motorA_Atras(vel)`
- `detenerMotorA()`

which encapsulate both direction and PWM speed control.

### Test Routine (`rutinaPruebaConMotor`)

Provides a simple sequence to validate:

- Steering alignment
- Servo response
- Motor operation
- Power stability

before integrating higher-level autonomous control algorithms.

---

# Explicación del Código y Arquitectura del Sistema

## 1. Capa de Percepción: ESP32-CAM (`esp32_vision.ino`)

El subsistema de percepción se encarga de capturar imágenes del entorno, transformar el espacio de color y detectar obstáculos mediante segmentación horizontal.

> **Nota importante sobre la arquitectura del sistema:**  
> El firmware actual procesa cada fotograma localmente y envía información de depuración mediante `Serial` a **9600 baudios**. **La comunicación UART (TX/RX) con el controlador central LAFVIN R3 aún no está implementada.** En futuras versiones se transmitirán tokens compactos en lugar de texto.

### Subcomponentes Técnicos

#### Configuración y Captura de Cámara

Inicializa el sensor OV2640 utilizando un reloj **XCLK de 20 MHz** con resolución **CIF (400 × 296 píxeles)**. Las imágenes se capturan en formato `PIXFORMAT_JPEG` para optimizar el uso del DMA.

#### Descompresión y Lectura RGB565

Las imágenes JPEG (`camera_fb_t`) se convierten a un búfer temporal **RGB565 de 16 bits** mediante `jpg2rgb565()`.

Los componentes de color se obtienen mediante operaciones de desplazamiento de bits:

```math
R = ((pixel \gg 11) \& 0x1F) \ll 3
```

```math
G = ((pixel \gg 5) \& 0x3F) \ll 2
```

```math
B = (pixel \& 0x1F) \ll 3
```

#### Conversión de Espacio de Color (`rgbToHsv`)

Los valores RGB se normalizan al intervalo

```math
[0,1]
```

y posteriormente se convierten al espacio HSV (Tono, Saturación y Valor), lo que mejora la robustez frente a cambios de iluminación.

#### Optimización (`PIXEL_STEP`)

Para incrementar el rendimiento se procesa únicamente uno de cada dos píxeles:

```math
PIXEL\_STEP = 2
```

ignorando además un margen de 10 píxeles alrededor de la imagen.

#### Clasificación de Colores y Centroide

Los píxeles pertenecientes a los rangos de Rojo, Verde, Magenta, Azul y Naranja incrementan contadores y acumulan sus coordenadas.

Si la cantidad de píxeles detectados supera

```math
threshold = 130
```

el centroide horizontal se calcula como

```math
Center_X = \frac{\sum X}{N}
```

#### Zonificación Horizontal (`obtenerPosicion`)

La imagen de **400 píxeles** de ancho se divide en tres sectores aproximadamente iguales:

```math
400 / 3 \approx 133
```

Dependiendo de la posición del centroide, el objeto se clasifica como:

- `IZQUIERDA`
- `CENTRO`
- `DERECHA`

---

## 2. Capa de Locomoción y Actuación: Controlador Central (`motor_servo_control.ino`)

Este módulo controla el servomotor de dirección y el motor de tracción mediante funciones de alto nivel.

### Hardware

#### Servomotor (Pin 9)

- `ANGULO_RECTO = 76°`
- `ANGULO_DER = 110°`
- `ANGULO_IZQ = 35°`

#### Motor DC (Pines 4 y 5)

- `PIN_DIR_MOTOR_A = 4`
- `PIN_PWM_MOTOR_A = 5`
- `VEL_RAPIDA = 190`
- `VEL_NORMAL = 150`

### Funciones

#### Control de Bajo Nivel

- `moverDireccion(int angulo)`
- `controlarMotorA(boolean direccion, int velocidad)`

#### Control de Alto Nivel

- `avanzarRecto()`
- `girarIzquierda()`
- `girarDerecha()`
- `motorA_Adelante(vel)`
- `motorA_Atras(vel)`
- `detenerMotorA()`

#### Rutina de Prueba

`rutinaPruebaConMotor()` permite verificar:

- funcionamiento del servo,
- funcionamiento del motor,
- alineación mecánica,
- estabilidad de alimentación,

antes de incorporar algoritmos de control autónomo.

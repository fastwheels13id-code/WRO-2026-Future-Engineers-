Control software
====
# Code Architecture & Firmware Explanation

## 1. Perception Layer: ESP32-CAM (`esp32_vision.ino`)

The perception subsystem is responsible for acquiring raw visual data from the track environment, transforming color spaces, and detecting target track obstacles using horizontal spatial division.

> **Important Note on System Architecture:** 
> The provided vision firmware currently handles local frame processing and outputs diagnostics directly over `Serial` (at `9600` baud) for debugging purposes. **Serial transmission (TX/RX) logic between the ESP32-CAM and the central LAFVIN R3 controller is not yet integrated in this code version.** Future updates will stream compact byte tokens over UART instead of human-readable text logs.

### Key Technical Subsystems

*   **Camera Configuration & Acquisition:** 
    Initializes the OV2640 sensor over a 20 MHz XCLK baseline clock utilizing a CIF resolution array ($400 \times 296$ pixels). Images are captured in `PIXFORMAT_JPEG` format to optimize Internal DMA buffer transfers.
*   **Frame Decompression & RGB565 Parsing:**
    Raw JPEG frames (`camera_fb_t`) are decompressed into a temporary 16-bit `RGB565` buffer using `jpg2rgb565()`. Each pixel’s bit-depth is extracted using bitwise shifts:
    $$\text{Red} = ((pixel \gg 11) \ \& \ 0x1F) \ll 3$$
    $$\text{Green} = ((pixel \gg 5) \ \& \ 0x3F) \ll 2$$
    $$\text{Blue} = (pixel \ \& \ 0x1F) \ll 3$$
*   **Color Transformation (`rgbToHsv`):**
    RGB values are normalized to $[0.0, 1.0]$ and converted to the HSV (Hue, Saturation, Value) space. Decoupling Hue (chromatic identity) from Saturation and Value ensures immunity against changing track light levels.
*   **Performance Optimization (`PIXEL_STEP`):**
    To avoid severe frame rate drop-offs, nested loops parse the image frame stepping by `PIXEL_STEP = 2` pixels, skipping boundary margins ($X, Y \in [10, \text{Limit} - 10]$).
*   **Color Classification & Centroid Calculation:**
    Pixels matching specific Hue ranges (Red, Green, Magenta, Blue, Orange) increment density counters and accumulate absolute spatial coordinates ($X, Y$). If a color density exceeds `threshold = 130`, the algorithm computes its spatial geometric center:
    $$\text{Center X} = \frac{\text{Sum of X Coordinates}}{\text{Total Valid Pixels}}$$
*   **Horizontal Zone Classification (`obtenerPosicion`):**
    The $400\text{px}$ horizontal plane is split into three equal sectors ($\approx 133\text{px}$ each). Depending on where $\text{Center X}$ lands, the object is classified as **`IZQUIERDA`** (Left), **`CENTRO`** (Center), or **`DERECHA`** (Right).

---

## 2. Locomotion & Actuation Layer: Central Controller (`motor_servo_control.ino`)

This module manages low-level hardware actuation for the vehicle's traction motor and front steering geometry, providing clean abstractions for directional maneuvers.

### Hardware Mapping & Constants

*   **Steering Servo (Pin 9):** Driven via the `Servo.h` library.
    *   `ANGULO_RECTO = 76°`: Calculated hardware center alignment for straight driving.
    *   `ANGULO_DER = 110°`: Maximum safe right steering limit.
    *   `ANGULO_IZQ = 35°`: Maximum safe left steering limit.
*   **DC Traction Motor (Pins 4 & 5):** Driven via an H-bridge interface.
    *   `PIN_DIR_MOTOR_A = 4`: Digital pin controlling H-Bridge current direction (`LOW` = Forward, `HIGH` = Reverse).
    *   `PIN_PWM_MOTOR_A = 5`: PWM-capable pin regulating motor speed ($0$ to $255$).
    *   `VEL_RAPIDA = 190`: High-speed cruise setting.
    *   `VEL_NORMAL = 150`: Reduced speed maneuvering setting.

### Functional Breakdown

1.  **Low-Level Hardware Drivers:**
    *   `moverDireccion(int angulo)`: Directly writes targeted pulse widths to the MG996R steering servo.
    *   `controlarMotorA(boolean direccion, int velocidad)`: Sets digital direction state and outputs hardware PWM duty cycles to the traction motor driver.
2.  **High-Level Behavioral Abstractions:**
    *   `avanzarRecto()`, `girarIzquierda()`, `girarDerecha()`: Immediate preset calls to reposition steering geometry.
    *   `motorA_Adelante(vel)`, `motorA_Atras(vel)`, `detenerMotorA()`: High-level traction commands encapsulating direction flags and PWM levels.
3.  **Testing Routines (`rutinaPruebaConMotor`):**
    Provides a non-blocking sequence to validate mechanical alignment, power draw stability, and steering responsiveness before integrating closed-loop control loops.

---

# Explicación del Código y Arquitectura del Sistema

## 1. Capa de Percepción: ESP32-CAM (`esp32_vision.ino`)

El subsistema de percepción se encarga de capturar las imágenes del entorno de la pista, transformar el espacio de color y detectar los obstáculos mediante segmentación espacial horizontal.

> **Nota Importante sobre la Arquitectura del Sistema:** 
> El código de visión proporcionado procesa los fotogramas de forma local e imprime diagnósticos directamente en el Monitor Serial (`9600` baudios) para depuración. **La lógica de transmisión serial (TX/RX) entre el ESP32-CAM y el controlador central LAFVIN R3 aún no está contemplada en esta versión del código.** En futuras actualizaciones, se enviarán tokens compactos por UART en lugar de texto legible.

### Subcomponentes Técnicos Clave

*   **Configuración y Captura de Cámara:** 
    Inicializa el sensor OV2640 con una frecuencia base XCLK de 20 MHz a una resolución CIF ($400 \times 296$ píxeles). Las imágenes se obtienen en formato `PIXFORMAT_JPEG` para optimizar las transferencias del búfer por DMA.
*   **Descompresión y Lectura RGB565:**
    Los fotogramas comprimidos (`camera_fb_t`) se descomprimen en un búfer temporal de 16 bits `RGB565` mediante `jpg2rgb565()`. Los colores de cada píxel se extraen usando operaciones de desplazamiento de bits:
    $$\text{Rojo} = ((pixel \gg 11) \ \& \ 0x1F) \ll 3$$
    $$\text{Verde} = ((pixel \gg 5) \ \& \ 0x3F) \ll 2$$
    $$\text{Azul} = (pixel \ \& \ 0x1F) \ll 3$$
*   **Conversión de Espacio de Color (`rgbToHsv`):**
    Normaliza los valores RGB al rango $[0.0, 1.0]$ y los transforma al espacio HSV (Tono, Saturación, Valor). Desacoplar el Tono de la Saturación y el Valor asegura inmunidad ante cambios en la iluminación de la pista.
*   **Optimización del Procesamiento (`PIXEL_STEP`):**
    Para mantener un alto procesamiento de fotogramas por segundo, se utilizan bucles anidados con un salto de `PIXEL_STEP = 2` píxeles, ignorando los márgenes de los bordes ($X, Y \in [10, \text{Límite} - 10]$).
*   **Clasificación y Centroide:**
    Los píxeles que coinciden con los rangos de Tono (Rojo, Verde, Magenta, Azul, Naranja) incrementan contadores de densidad y acumulan coordenadas ($X, Y$). Si la densidad supera el umbral `threshold = 130`, se calcula el centro geométrico:
    $$\text{Centro X} = \frac{\text{Suma de Coordenadas X}}{\text{Total de Píxeles Válidos}}$$
*   **Zonificación Horizontal (`obtenerPosicion`):**
    El plano horizontal de $400\text{px}$ se divide en tres sectores iguales ($\approx 133\text{px}$ cada uno). Según la posición de $\text{Centro X}$, el objeto se clasifica como **`IZQUIERDA`**, **`CENTRO`** o **`DERECHA`**.

---

## 2. Capa de Locomoción y Actuación: Controlador Central (`motor_servo_control.ino`)

Este módulo controla los actuadores del vehículo (motor de tracción y servomotor de dirección delantera), proporcionando funciones claras para maniobras de movimiento.

### Mapeo de Hardware y Constantes

*   **Servomotor de Dirección (Pin 9):** Controlado a través de la librería `Servo.h`.
    *   `ANGULO_RECTO = 76°`: Ángulo centrado para avance en línea recta.
    *   `ANGULO_DER = 110°`: Límite máximo seguro de giro a la derecha.
    *   `ANGULO_IZQ = 35°`: Límite máximo seguro de giro a la izquierda.
*   **Motor DC de Tracción (Pines 4 y 5):** Controlado mediante interfaz Puente H.
    *   `PIN_DIR_MOTOR_A = 4`: Pin digital para dirección de corriente (`LOW` = Adelante, `HIGH` = Reversa).
    *   `PIN_PWM_MOTOR_A = 5`: Pin PWM para el control de velocidad ($0$ a $255$).
    *   `VEL_RAPIDA = 190`: Velocidad máxima de crucero.
    *   `VEL_NORMAL = 150`: Velocidad reducida para maniobras.

### Funciones Principales

1.  **Controladores de Bajo Nivel:**
    *   `moverDireccion(int angulo)`: Envía pulsos directos al servomotor MG996R.
    *   `controlarMotorA(boolean direccion, int velocidad)`: Establece el sentido de marcha y envía la señal PWM al driver del motor.
2.  **Abstracciones de Alto Nivel:**
    *   `avanzarRecto()`, `girarIzquierda()`, `girarDerecha()`: Llamadas directas para ajustar la posición del servo.
    *   `motorA_Adelante(vel)`, `motorA_Atras(vel)`, `detenerMotorA()`: Comandos de tracción que encapsulan dirección y velocidad PWM.
3.  **Rutina de Prueba (`rutinaPruebaConMotor`):**
    Secuencia simple para validar la alineación mecánica, la estabilidad del voltaje y la respuesta de la dirección antes de implementar bucles de control cerrado.

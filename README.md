# Section in English

# WRO-2026-Future-Engineers-Fast Wheels ID

# Autonomous Vehicle Engineering Documentation

## 1. Introduction & Engineering Philosophy

This project presents the development of an autonomous robotic vehicle designed to navigate a structured environment, track specific pathways, and dynamically avoid obstacles. Instead of relying on heavy edge-computing processors, expensive single-board computers, or external cloud dependencies, our engineering philosophy focused on structural efficiency: building a distributed embedded system capable of real-time perception and immediate electromechanical response.

To achieve this, we decoupled the architecture into two dedicated hardware processing nodes. A specialized camera module handles high-speed matrix processing, bitwise filtering, and computer vision, while a robust central microcontroller governs mechanical orchestration, active inertial heading correction via a dedicated gyroscope, hardware actuation, and dynamic behavioral loops. By isolating perception from locomotion and heading control—and transitioning away from restrictive shield configurations to a fully custom-routed LAFVIN R3 pin layout—we achieved higher operational flexibility, minimized processing latency, and ensured that safety overrides are executed with millisecond-level precision using localized hardware logic. This approach proves that complex autonomous behaviors can be accomplished efficiently on microcontrollers through optimized firmware design and strict resource management.

---

## 2. System Architecture & Component Integration

Our system was designed around a dual-controller layout, dividing perception and physical execution into two distinct firmware layers interacting seamlessly via continuous asynchronous serial communication:

*   **Perception Controller (ESP32-CAM):** Operates as the visual core of the vehicle. It manages high-speed frame buffer acquisition through the onboard OV2640 camera sensor and coordinates its integrated status LED for synchronization signals and localized flash illumination.
*   **Locomotion & Central Controller (LAFVIN R3 System):** Functions as the primary brain. It processes real-time payload data sent by the vision system, evaluates inertial feedback, and coordinates the vehicle's actuators through a dedicated, flexible hardware ecosystem:
    *   **LAFVIN R3 Development Board:** Serves as the central logic unit (compatible with the ATmega328P footprint). We transitioned directly to the LAFVIN R3 layout instead of using the rigid Keyestudio shield, granting us full pin allocation freedom to route I2C, PWM, and digital I/O without hardware pin conflicts.
    *   **GY-521 Gyroscope / IMU Module (MPU-6050):** Communicates via the I2C bus (SDA/SCL) to track the vehicle's real-time yaw heading rate and angular deviation. Its primary function is active trajectory correction: if the chassis curves or drifts off-course due to surface friction, uneven wheel traction, or mechanical play, the GY-521 detects the angular tilt and dynamically steers the vehicle back onto its baseline path.
    *   **H-Bridge Motor Driver & Actuation Drivers:** Connected directly via the LAFVIN R3's freed-up digital and PWM pins to handle current regulation and variable speed execution.
    *   **MG996R High-Torque Servomotor:** Directly driven via a dedicated PWM pin on the LAFVIN R3 to actuate the front steering geometry, ensuring precise steering angles and maintaining steering alignment under mechanical stress.
    *   **DC Traction Motor:** Connected to the motor driver terminals to govern speed ramping, direction switching, and instant electronic braking via pulse-width modulation (PWM).
    *   **2x HC-SR04 Ultrasonic Sensors:** Configured via unreserved digital input/output pins to map the immediate physical perimeter, identifying walls or unexpected path hazards through ultrasonic wave time-of-flight calculations.
    *   **Power & Regulation Subsystem:** Fueled by a high-drain battery bank consisting of rechargeable 18650 lithium-ion batteries (3.7V each), isolated and stabilized through an HW-131 power supply module to prevent voltage drops during motor spikes.

---

## 3. How It Works: Algorithms & Decision-Making Framework

### Computer Vision Pipeline (Color Segmentation)

The ESP32-CAM implements a cyclic, non-blocking computer vision algorithm engineered to isolate specific color paths within the HSV (Hue, Saturation, Value) color space. The step-by-step pipeline operates through the following stages:

1.  **System Initialization:** Establishes serial debugging parameters at a high baud rate and configures the camera sensor registers utilizing a custom Common Intermediate Format (CIF) baseline resolution of 400 x 296 pixels (instantiated via `const int CAM_WIDTH = 400;` and `const int CAM_HEIGHT = 296;`). It locks specific sensor properties including white balance, gain control, and exposure limits to guarantee visual consistency.
2.  **Frame Acquisition:** Inside the main execution loop, the firmware queries the OV2640 sensor component, requesting a fresh raw image matrix. The sensor returns a compressed JPEG stream to optimize internal DMA transfers.
3.  **Decompression & Color Translation:** Because a raw compressed JPEG stream cannot be parsed pixel-by-pixel, the algorithm utilizes an internal decompression library to translate the stream into an uncompressed RGB565 bitmap array, allocating direct bitwise access to the color channel of every coordinate.
4.  **Grid Optimization Scanning:** To prevent severe frame drops and minimize loop processing latency, the algorithm parses the pixel matrix using nested loops (outer loops control rows on the Y-axis, inner loops control columns on the X-axis). Instead of analyzing all 118,400 pixels, it implements an optimization constant (`PIXEL_STEP = 2`), effectively checking every second pixel to maintain an excellent processing speed while preserving structural accuracy.
5.  **Bit Extraction & HSV Transformation:** For each sampled pixel, the individual Red, Green, and Blue bit-depth intensities are extracted from the 16-bit RGB565 structure. These variables are mathematically transformed into the HSV color space. Converting data into Hue (H), Saturation (S), and Value (V) decouples the actual color type from its brightness component. This ensures that color tracking remains highly immune to natural ambient shadows, track glares, or changing illumination across the venue.
6.  **Dynamic Noise Mitigation:** Before evaluating a pixel's color identity, the firmware passes the HSV metrics through a strict validation filter. Pixels falling below a minimum saturation baseline (washed-out tones or grays) or below a minimum value boundary (deep shadows and dark regions) are immediately discarded as visual noise.
7.  **Color Profiling & Classification:** Pixels that pass the noise filter have their exact Hue angle compared against precise, pre-programmed angular boundaries. These boundaries define the exact spectrum for our target track markers, including Red, Green, and Magenta boundaries.
8.  **Spatial Data Accumulation:** When a pixel successfully matches a color profile, the system increments a dedicated density counter for that specific color. Simultaneously, it adds the absolute X coordinate and Y coordinate of that pixel to running summation variables designated for spatial analysis.
9.  **Threshold Validation:** Once the matrix traversal concludes, the system evaluates the cumulative density counters. If the total number of detected pixels for a specific color does not exceed a minimum validation threshold (`threshold`), the entire cluster is dismissed as background artifact noise, preventing false positive triggers.
10. **Centroid Calculation & Horizontal Zone Mapping:** For any color cluster that passes validation, the system computes its exact geometric center. The spatial center is calculated by dividing the sum of the X coordinates by the total valid pixel count, and the sum of the Y coordinates by the same total count:

    $$\text{Center X} = \frac{\text{Sum of X coordinates}}{\text{Total Valid Pixels}}$$

    $$\text{Center Y} = \frac{\text{Sum of Y coordinates}}{\text{Total Valid Pixels}}$$

    The 400-pixel wide horizontal scanning plane is structurally split into three distinct virtual sectors: Left, Center, and Right. By evaluating where the Center X value falls within these boundaries, the vehicle knows exactly where the color marker is positioned relative to its chassis, transmitting this token immediately over the serial interface.
11. **Memory Release & Recycled Execution:** To avoid critical heap fragmentation or memory overflows on the microchip, the active frame buffer is fully cleared, and the allocated memory is freed. The pipeline loops back seamlessly to capture the next incoming frame buffer in real time.

### Locomotive Control & Heading Correction (The Enhanced 3-Question Framework)

Simultaneously, the LAFVIN R3 processes the incoming serial tracking strings alongside gyroscope metrics and local ultrasonic distance data through a high-speed execution cycle known as the **Enhanced Three-Question Framework**:

*   **Question 1: "What is my current sensor & inertial state?"**
    The vehicle samples environmental and kinetic telemetry continuously across three operational vectors:
    *   *Visual Vector:* Streams color tokens from the ESP32-CAM via the serial buffer.
    *   *Spatial Vector:* Measures lateral distances to track walls using the two HC-SR04 ultrasonic sensors connected to dedicated LAFVIN R3 digital pins.
    *   *Inertial Vector:* Reads Z-axis angular velocity and yaw orientation from the GY-521 (MPU-6050) module over I2C. When entering a straight segment, the controller locks a target reference angle ($\theta_{\text{target}}$).
*   **Question 2: "Is there a hazard, boundary, or trajectory deviation?"**
    The controller continuously checks metrics against safety thresholds and heading alignment:
    *   *Obstacle Hazard:* Triggered if the ultrasonic metrics drop below the critical safety margin or if a prohibited boundary marker (e.g., solid Red token) is detected.
    *   *Trajectory Drift / Deviation:* Evaluated during forward motion. If the chassis curves or strays off its heading due to mechanical slop, uneven traction, or surface bumps, the GY-521 registers the angular error ($\Delta\theta = \theta_{\text{current}} - \theta_{\text{target}}$).
*   **Question 3: "What action must I take?"**
    Depending on the operational flag, the firmware adjusts motor PWM and servo positioning:
    *   *Active Gyro Deviation Correction:* If a curved path or drift error ($\Delta\theta$) is detected by the GY-521, the LAFVIN R3 calculates a proportional correction command. The MG996R steering servo executes instant counter-steering to correct the vehicle's position and restore straight-line trajectory.
    *   *Evasion / Maneuver Branch:* Upon detecting a wall or obstacle hazard, the system halts forward traction, calculates the necessary turn angle using the GY-521 feedback, turns the steering servo to maximum safe deflection, and executes an evasion protocol before re-locking the straight heading baseline.

---

## 4. Implementation Journey: How We Built It

The realization of this vehicle followed a rigorous engineering process, divided into three core development phases:

*   **Phase 1: Pin Optimization & Power Management:** We redesigned the physical chassis hardware layout by switching to the LAFVIN R3 board without using the original Keyestudio shield. This modification unlocked pin routing flexibility, allowing us to connect the GY-521, ultrasonic sensors, and actuators without pin contention. Power stabilization was achieved by integrating an HW-131 regulator board with high-drain 18650 lithium cells, isolating the logic power rail from the heavy inductive loads generated by the DC motors and MG996R servo.
*   **Phase 2: Vision Optimization & GY-521 Gyro Calibration:** Developing the perception layer required balancing accuracy against strict processing limitations on the ESP32-CAM, which we solved using a custom CIF resolution matrix (400 x 296) and RGB565 layout with `PIXEL_STEP = 2`. Simultaneously, on the LAFVIN R3, we written an automatic zero-rate calibration routine for the GY-521 module at startup. This routine eliminates gyro offset drift and ensures exact angular detection whenever the vehicle needs to correct unwanted curvature or deviation.
*   **Phase 3: Closed-Loop Heading & Evasion Testing:** The final phase focused on fine-tuning the communication, inertial correction loop, and behavioral response systems. We routed the serial TX/RX pins from the ESP32-CAM to the LAFVIN R3 and established stable I2C lines for the GY-521. Track tests allowed us to calibrate the proportional feedback loop so that any unwanted chassis deviation detected by the GY-521 triggers immediate steering correction to keep the vehicle centered.

---

## 5. How to Compile and Upload the Code

Both software profiles are compiled and flashed utilizing the official **Arduino IDE** development environment:

### Firmware Flashing (Perception - ESP32-CAM)

1. Open the computer vision sketch file (`.ino`) in the Arduino IDE.
2. Navigate to **Tools > Board** and choose **ESP32 Wrover Module**. Then, configure **Tools > Partition Scheme** to **Huge APP (3MB No OTA/1MB SPIFFS)** to ensure the compiled vision libraries fit comfortably into the flash memory.
3. Connect the camera module via an external FTDI programmer or its dedicated daughterboard, map the active COM port under **Tools > Port**, and bridge the `GPIO 0` pin to `GND` using a jumper wire to unlock flashing mode before hitting the reset button.
4. Click the **Upload** arrow icon. Once the process reaches `100%`, disconnect the jumper wire from `GPIO 0` and reset the module to start execution.

### Firmware Flashing (Locomotion & GY-521 Control - LAFVIN R3)

1. Open the main control sketch file (`.ino`) in the Arduino IDE.
2. Navigate to **Tools > Board** and select **Arduino Uno** (the baseline architectural profile required for the LAFVIN R3 board).
3. Connect the board to your computer using a standard USB Type-B cable. **Crucial Safety Step:** Make sure the external 18650 power switch is turned **OFF** during programming to avoid dangerous power feedback loops to your computer.
4. Select the matching active serial port under **Tools > Port**, click the **Upload** arrow icon, and wait for confirmation. Once completed, disconnect the programming cable and switch the main circuit power source **ON** for autonomous tracking.

---

## 6. Acknowledgments & Final Reflections

We would like to express our deepest gratitude to our institution and teacher Iliana Caballero, advisors, and peers who supported us throughout this engineering journey. Special thanks to our team members for their dedication during long debugging sessions, calibrating sensors, tuning GY-521 gyroscope parameters, and refining the structural code of this vehicle.

We also extend our sincere appreciation to the organizers of this international competition for providing an exceptional platform that challenges students to push the boundaries of embedded hardware programming, electronics, and autonomous mobile robotics. This project stands as a testament to teamwork, technical perseverance, and practical engineering.


# Sección en Español

# WRO-2026-Future-Engineers-Fast Wheels ID

# Documentación de Ingeniería del Vehículo Autónomo

## 1. Introducción y Filosofía de Ingeniería

Este proyecto presenta el desarrollo de un vehículo robótico autónomo diseñado para navegar en un entorno estructurado, seguir trayectorias específicas y esquivar obstáculos de forma dinámica. En lugar de depender de procesadores pesados de computación en el borde (edge computing), computadoras de placa única costosas o dependencias externas en la nube, nuestra filosofía de ingeniería se centró en la eficiencia estructural: construir un sistema embebido distribuido capaz de percepción en tiempo real y respuesta electromecánica inmediata.

Para lograr esto, desacoplamos la arquitectura en dos nodos de procesamiento de hardware dedicados. Un módulo de cámara especializado se encarga del procesamiento matricial a alta velocidad, filtrado bit a bit y visión por computadora, mientras que un microcontrolador central robusto gobierna la orquestación mecánica, la corrección activa de rumbo inercial mediante un giroscopio dedicado, la actuación del sistema y los bucles de comportamiento dinámico. Al aislar la percepción de la locomoción y el control de orientación—y al migrar de configuraciones rígidas de shields a una distribución de pines totalmente personalizada en la placa LAFVIN R3—logramos una mayor flexibilidad operativa, minimizamos la latencia de procesamiento y garantizamos que las interrupciones de seguridad se ejecuten con precisión de milisegundos utilizando lógica de hardware local. Este enfoque demuestra que es posible lograr comportamientos autónomos complejos de manera eficiente en microcontroladores mediante un diseño de firmware optimizado y una estricta gestión de recursos.

---

## 2. Arquitectura del Sistema e Integración de Componentes

Nuestro sistema fue diseñado en torno a una disposición de doble controlador, dividiendo la percepción y la ejecución física en dos capas de firmware distintas que interactúan de manera fluida a través de comunicación serial asíncrona continua:

*   **Controlador de Percepción (ESP32-CAM):** Funciona como el núcleo visual del vehículo. Gestiona la adquisición del búfer de fotogramas a alta velocidad a través del sensor de cámara integrado OV2640 y coordina su LED de estado para señales de sincronización e iluminación de flash localizada.
*   **Controlador de Locomoción y Central (Sistema LAFVIN R3):** Funciona como el cerebro principal. Procesa los datos en tiempo real enviados por el sistema de visión, evalúa la retroalimentación inercial y coordina los actuadores del vehículo a través de un ecosistema de hardware dedicado y flexible:
    *   **Placa de Desarrollo LAFVIN R3:** Sirve como la unidad lógica central (compatible con la arquitectura del ATmega328P). Migramos directamente a la distribución de la LAFVIN R3 en lugar de utilizar la shield de Keyestudio, lo que nos otorgó total libertad en la asignación de pines para enrutar I2C, PWM y entradas/salidas digitales sin conflictos de pines.
    *   **Módulo Giroscopio / IMU GY-521 (MPU-6050):** Se comunica a través del bus I2C (SDA/SCL) para rastrear la tasa de rotación en el eje z (yaw) y la desviación angular del vehículo en tiempo real. Su función principal es la corrección activa de trayectoria: si el chasis se curva o se desvía de su rumbo debido a la fricción de la superficie, tracción desequilibrada en las ruedas o holgura mecánica, el GY-521 detecta la inclinación angular y redirige dinámicamente el vehículo hacia su trayectoria base.
    *   **Controlador de Motores Puente H y Actuadores:** Conectados directamente a través de los pines digitales y PWM liberados de la LAFVIN R3 para gestionar la regulación de corriente y la velocidad variable.
    *   **Servomotor de Alto Torque MG996R:** Controlado directamente mediante un pin PWM dedicado en la LAFVIN R3 para accionar la geometría del sistema de dirección delantera, garantizando ángulos de giro precisos y manteniendo la alineación bajo estrés mecánico.
    *   **Motor DC de Tracción:** Conectado a las terminales del controlador de motores para gobernar el rampa de velocidad, el cambio de sentido de marcha y el frenado electrónico instantáneo mediante modulación por ancho de pulsos (PWM).
    *   **2x Sensores Ultrasónicos HC-SR04:** Configurados a través de pines de entrada/salida digital no reservados para mapear el perímetro físico inmediato, identificando paredes u obstáculos inesperados en la pista mediante cálculos del tiempo de vuelo de ondas ultrasónicas.
    *   **Subsistema de Alimentación y Regulación:** Alimentado por un paquete de baterías de alta descarga compuesto por celdas recargables de ión de litio 18650 (3.7V cada una), aisladas y estabilizadas a través de un módulo de fuente de alimentación HW-131 para evitar caídas de voltaje durante picos de consumo de los motores.

---

## 3. Funcionamiento: Algoritmos y Marco de Toma de Decisiones

### Pipeline de Visión por Computadora (Segmentación de Color)

El ESP32-CAM implementa un algoritmo de visión por computadora cíclico y no bloqueante, diseñado para aislar trayectorias de color específicas dentro del espacio de color HSV (Hue, Saturation, Value). El pipeline paso a paso opera en las siguientes etapas:

1.  **Inicialización del Sistema:** Establece los parámetros de depuración serial a una tasa de baudios alta y configura los registros del sensor de la cámara utilizando una resolución base personalizada CIF (Common Intermediate Format) de 400 x 296 píxeles (instanciada mediante `const int CAM_WIDTH = 400;` y `const int CAM_HEIGHT = 296;`). Bloquea propiedades específicas del sensor como balance de blancos, control de ganancia y límites de exposición para garantizar consistencia visual.
2.  **Adquisición del Fotograma:** Dentro del bucle principal de ejecución, el firmware consulta el componente del sensor OV2640 solicitando una nueva matriz de imagen. El sensor devuelve un flujo JPEG comprimido para optimizar las transferencias internas por DMA.
3.  **Descompresión y Traducción de Color:** Debido a que un flujo JPEG comprimido no se puede analizar píxel por píxel, el algoritmo utiliza una librería de descompresión interna para traducir el flujo a una matriz de mapa de bits RGB565 sin comprimir, otorgando acceso directo a nivel de bits al canal de color de cada coordenada.
4.  **Escaneo Optimizado por Malla:** Para evitar caídas severas en la tasa de fotogramas y minimizar la latencia en el bucle de procesamiento, el algoritmo analiza la matriz de píxeles mediante bucles anidados (los bucles externos controlan las filas en el eje Y, y los internos las columnas en el eje X). En lugar de analizar los 118,400 píxeles, implementa una constante de optimización (`PIXEL_STEP = 2`), verificando efectivamente uno de cada dos píxeles para mantener una velocidad de procesamiento excelente conservando la precisión estructural.
5.  **Extracción de Bits y Transformación a HSV:** Para cada píxel muestreado, se extraen las intensidades individuales de Red, Green y Blue de la estructura RGB565 de 16 bits. Estas variables se transforman matemáticamente al espacio de color HSV. Convertir los datos a Tono (H), Saturación (S) y Valor (V) desacopla el color real de su componente de brillo, asegurando que el rastreo sea altamente inmune a sombras ambientales, reflejos en la pista o cambios de iluminación en el recinto.
6.  **Mitigación Dinámica de Ruido:** Antes de evaluar la identidad de color de un píxel, el firmware pasa las métricas HSV por un estricto filtro de validación. Los píxeles que caen por debajo de un umbral mínimo de saturación (tonos lavados o grises) o por debajo de un umbral mínimo de valor (sombras profundas y regiones oscuras) se descartan inmediatamente como ruido visual.
7.  **Perfilado y Clasificación de Color:** Los píxeles que superan el filtro de ruido comparan su ángulo exacto de Tono (Hue) contra límites angulares programados previamente. Estos límites definen el espectro exacto para nuestras marcas en la pista, incluyendo bordes de color Rojo, Verde y Magenta.
8.  **Acumulación de Datos Espaciales:** Cuando un píxel coincide con un perfil de color, el sistema incrementa un contador de densidad dedicado a ese color. Simultáneamente, suma las coordenadas absolutas X e Y de ese píxel a variables de acumulación diseñadas para el análisis espacial.
9.  **Validación por Umbral:** Al concluir el recorrido de la matriz, el sistema evalúa los contadores de densidad acumulados. Si el número total de píxeles detectados para un color no supera un umbral mínimo de validación (`threshold`), todo el clúster se descarta como ruido de fondo, evitando falsos positivos.
10. **Cálculo de Centroide y Mapeo de Zonas Horizontales:** Para cualquier clúster de color que supere la validación, el sistema calcula su centro geométrico exacto dividiendo la suma de las coordenadas X entre el total de píxeles válidos, y la suma de las coordenadas Y entre el mismo total:

    $$\text{Centro X} = \frac{\text{Suma de coordenadas X}}{\text{Total de Píxeles Válidos}}$$

    $$\text{Centro Y} = \frac{\text{Suma de coordenadas Y}}{\text{Total de Píxeles Válidos}}$$

    El plano de escaneo horizontal de 400 píxeles de ancho se divide estructuralmente en tres sectores virtuales: Izquierda, Centro y Derecha. Al evaluar en qué sector cae el valor del Centro X, el vehículo conoce exactamente dónde está ubicada la marca de color con respecto a su chasis, transmitiendo esta información inmediatamente por la interfaz serial.
11. **Liberación de Memoria y Ejecución Reciclada:** Para evitar la fragmentación crítica de la memoria heap o desbordamientos en el microchip, el búfer del fotograma activo se limpia por completo y la memoria asignada se libera. El pipeline vuelve a iniciar en bucle para capturar el siguiente fotograma en tiempo real.

### Control de Locomoción y Corrección de Rumbo (El Marco Mejorado de las 3 Preguntas)

Simultáneamente, la LAFVIN R3 procesa las cadenas de datos seriales provenientes de la cámara, junto con las métricas del giroscopio y las distancias ultrasónicas locales a través de un ciclo de ejecución de alta velocidad denominado el **Marco Mejorado de las Tres Preguntas**:

*   **Pregunta 1: "¿Cuál es mi estado actual de sensores e inercia?"**
    El vehículo muestrea la telemetría ambiental y kinética continuamente a través de tres vectores operativos:
    *   *Vector Visual:* Recibe los tokens de color del ESP32-CAM vía búfer serial.
    *   *Vector Espacial:* Mide distancias laterales a las paredes de la pista mediante los dos sensores ultrasónicos HC-SR04 conectados a pines digitales dedicados en la LAFVIN R3.
    *   *Vector Inercial:* Lee la velocidad angular en el eje Z y la orientación yaw desde el módulo GY-521 (MPU-6050) vía I2C. Al ingresar a un tramo recto, el controlador establece y bloquea un ángulo de referencia objetivo ($\theta_{\text{target}}$).
*   **Pregunta 2: "¿Existe algún peligro, límite o desviación en la trayectoria?"**
    El controlador evalúa continuamente las métricas frente a umbrales de seguridad y alineación de rumbo:
    *   *Peligro de Obstáculo:* Se activa si la distancia ultrasónica cae por debajo del margen de seguridad crítico o si se detecta un token de límite prohibido (por ejemplo, token Rojo sólido).
    *   *Desviación o Curvatura de Trayectoria:* Se evalúa durante el desplazamiento hacia adelante. Si el chasis se curva o se desvía de su rumbo debido a holguras mecánicas, tracción desequilibrada o irregularidades en la pista, el GY-521 registra el error angular ($\Delta\theta = \theta_{\text{current}} - \theta_{\text{target}}$).
*   **Pregunta 3: "¿Qué acción debo tomar?"**
    Dependiendo de la bandera operativa, el firmware ajusta el PWM de los motores y la posición del servo:
    *   *Corrección Activa por Giroscopio:* Si el GY-521 detecta una trayectoria curva o un error de desviación ($\Delta\theta$), la LAFVIN R3 calcula un comando de corrección proporcional. El servomotor MG996R ejecuta un contra-giro instantáneo para corregir la posición del vehículo y restaurar la trayectoria en línea recta.
    *   *Rama de Evasión / Maniobra:* Al detectar una pared u obstáculo, el sistema detiene la tracción, calcula el ángulo de giro necesario mediante la retroalimentación del GY-521, gira el servo a su deflexión máxima segura y ejecuta un protocolo de evasión antes de volver a fijar el rumbo recto base.

---

## 4. Proceso de Implementación: Cómo lo Construimos

La realización de este vehículo siguió un riguroso proceso de ingeniería dividido en tres fases principales de desarrollo:

*   **Fase 1: Optimización de Pines y Gestión de Energía:** Rediseñamos la distribución física del chasis cambiando a la placa LAFVIN R3 sin utilizar la shield original de Keyestudio. Esta modificación nos otorgó flexibilidad en el enrutamiento de pines, permitiéndonos conectar el GY-521, los sensores ultrasónicos y los actuadores sin conflictos de pines. La estabilización de energía se logró integrando una placa reguladora HW-131 con celdas de litio 18650 de alta descarga, aislando la línea de lógica contra los picos inductivos generados por los motores DC y el servo MG996R.
*   **Fase 2: Optimización de Visión y Calibración del GY-521:** Desarrollar la capa de percepción requirió equilibrar la precisión con los estrictos límites de procesamiento del ESP32-CAM, lo cual resolvimos utilizando una resolución CIF personalizada (400 x 296) y formato RGB565 con `PIXEL_STEP = 2`. Simultáneamente, en la LAFVIN R3 programamos una rutina de calibración automática de offset a cero para el módulo GY-521 al momento del encendido. Esta rutina elimina la deriva (drift) del giroscopio y asegura una detección angular exacta cuando el vehículo necesita corregir desvíos o curvaturas no deseadas.
*   **Fase 3: Bucle Cerrado de Orientación y Pruebas de Evasión:** La fase final se centró en ajustar la comunicación, el bucle de corrección inercial y los sistemas de respuesta conductual. Conectamos los pines TX/RX del ESP32-CAM a la LAFVIN R3 y establecimos líneas I2C estables para el GY-521. Las pruebas en pista nos permitieron calibrar el bucle de retroalimentación proporcional, garantizando que cualquier desviación del chasis detectada por el GY-521 active una corrección de dirección inmediata para mantener el vehículo centrado.

---

## 5. Cómo Compilar y Cargar el Código

Ambos perfiles de software se compilan y cargan utilizando el entorno de desarrollo oficial **Arduino IDE**:

### Carga de Firmware (Percepción - ESP32-CAM)

1. Abre el archivo de código de visión por computadora (`.ino`) en el Arduino IDE.
2. Ve a **Herramientas > Placa** y selecciona **ESP32 Wrover Module**. Luego, en **Herramientas > Partition Scheme**, selecciona **Huge APP (3MB No OTA/1MB SPIFFS)** para asegurar que las librerías de visión quepan en la memoria flash.
3. Conecta el módulo de la cámara mediante un programador FTDI externo o su placa adaptadora, selecciona el puerto COM activo en **Herramientas > Puerto**, y conecta mediante un cable jumper el pin `GPIO 0` a `GND` para activar el modo de programación antes de presionar el botón de reinicio (reset).
4. Haz clic en el ícono de **Cargar** (flecha). Cuando el proceso alcance el `100%`, desconecta el jumper de `GPIO 0` y reinicia el módulo para iniciar la ejecución.

### Carga de Firmware (Locomoción y Control GY-521 - LAFVIN R3)

1. Abre el archivo de código de control principal (`.ino`) en el Arduino IDE.
2. Ve a **Herramientas > Placa** y selecciona **Arduino Uno** (el perfil de arquitectura base requerido para la placa LAFVIN R3).
3. Conecta la placa a tu computadora utilizando un cable USB Tipo B estándar. **Paso de Seguridad Crucial:** Asegúrate de que el interruptor de alimentación externa de las baterías 18650 esté **APAGADO** durante la programación para evitar retornos de corriente peligrosos hacia tu computadora.
4. Selecciona el puerto serial correspondiente en **Herramientas > Puerto**, haz clic en el ícono de **Cargar** y espera la confirmación. Una vez finalizado, desconecta el cable de programación y **ENCIENDE** la fuente de alimentación principal para la navegación autónoma.

---

## 6. Agradecimientos y Reflexiones Finales

Queremos expresar nuestro más sincero agradecimiento a nuestra institución, a nuestra profesora Iliana Caballero, a nuestros asesores y compañeros que nos apoyaron a lo largo de este viaje de ingeniería. Un reconocimiento especial a los miembros de nuestro equipo por su dedicación durante las largas sesiones de depuración, calibración de sensores, ajuste de parámetros del giroscopio GY-521 y optimización del código del vehículo.

Asimismo, extendemos nuestra gratitud a los organizadores de esta competencia internacional por brindar una plataforma excepcional que desafía a los estudiantes a superar los límites en programación de hardware embebido, electrónica y robótica móvil autónoma. Este proyecto se erige como un testimonio de trabajo en equipo, perseverancia técnica e ingeniería práctica.

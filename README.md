# Section in English
# WRO-2026-Future-Engineers-Fast Wheels ID

# Autonomous Vehicle Engineering Documentation

## 1. Introduction & Engineering Philosophy

This project presents the development of an autonomous robotic vehicle designed to navigate a structured environment, track specific pathways, and dynamically avoid obstacles. Instead of relying on heavy edge-computing processors, expensive single-board computers, or external cloud dependencies, our engineering philosophy focused on structural efficiency: building a distributed embedded system capable of real-time perception and immediate electromechanical response. 

To achieve this, we decoupled the architecture into two dedicated hardware processing nodes. A specialized camera module handles high-speed matrix processing, bitwise filtering, and computer vision, while a robust central microcontroller governs mechanical orchestration, hardware shield actuation, and dynamic behavioral loops. By isolating perception from locomotion, we achieved high operational stability, minimized processing latency, and ensured that safety overrides are executed with millisecond-level precision using localized hardware logic. This approach proves that complex autonomous behaviors can be accomplished efficiently on microcontrollers through optimized firmware design and strict resource management.

---

## 2. System Architecture & Component Integration

Our system was designed around a dual-controller layout, dividing perception and physical execution into two distinct firmware layers interacting seamlessly via continuous asynchronous serial communication:

*   **Perception Controller (ESP32-CAM):** Operates as the visual core of the vehicle. It manages high-speed frame buffer acquisition through the onboard OV2640 camera sensor and coordinates its integrated status LED for synchronization signals and localized flash illumination.
*   **Locomotion & Central Controller (Keyestudio Arduino R3):** Functions as the primary brain. It processes real-time payload data sent by the vision system and coordinates the vehicle's actuators through a dedicated hardware ecosystem:
    *   **Keyestudio K0435 Motor Shield:** Mounted directly over the Arduino R3 to handle current regulation, protecting the microcontroller's logic gates while delivering raw power to the motors via integrated H-bridge drivers.
    *   **MG996R High-Torque Servomotor:** Wired directly to the K0435 shield to actuate the front steering geometry, ensuring precise steering angles and maintaining steering alignment under mechanical stress.
    *   **DC Traction Motor:** Driven by the shield's terminal blocks to govern speed ramping, direction switching, and instant electronic braking via pulse-width modulation (PWM).
    *   **2x HC-SR04 Ultrasonic Sensors:** Configured via digital input/output pins to map the immediate physical perimeter, identifying walls or unexpected path hazards through ultrasonic wave time-of-flight calculations.
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
    *   Center X = (Sum of X coordinates) / Total Valid Pixels
    *   Center Y = (Sum of Y coordinates) / Total Valid Pixels
    The 400-pixel wide horizontal scanning plane is structurally split into three distinct virtual sectors: Left, Center, and Right. By evaluating where the Center X value falls within these boundaries, the vehicle knows exactly where the color marker is positioned relative to its chassis, transmitting this token immediately over the serial interface.
11. **Memory Release & Recycled Execution:** To avoid critical heap fragmentation or memory overflows on the microchip, the active frame buffer is fully cleared, and the allocated memory is freed. The pipeline loops back seamlessly to capture the next incoming frame buffer in real time.

### Locomotive Control (The 3-Question Framework)
Simultaneously, the Keyestudio Arduino R3 processes the incoming serial tracking strings alongside local sensor data through a structured, high-speed execution cycle known as the Three-Question Framework:

*   **Question 1: "What is my current sensor state?"**
    The vehicle samples its environmental telemetry continuously from multiple operational vectors. The visual vector provides color tokens streamed from the ESP32-CAM via the serial buffer, while the spatial vector calculates distance metrics to physical side-walls by pulsing the two HC-SR04 ultrasonic sensors.
*   **Question 2: "Is there a hazard or path deviation?"**
    The controller matches incoming metrics against safety thresholds. Under standard operation, detecting a gray track color combined with open ultrasonic distances confirms path clearance. However, if the visual string indicates a prohibited boundary marker (such as a solid Red token), or if the ultrasonic sensors detect a wall distance dropping below our critical safety margin, a high-priority override flag is raised.
*   **Question 3: "What action must I take?"**
    Depending on the state flag, the firmware issues commands to the K0435 motor shield:
    *   *Path is Clear Branch:* The DC traction motor maintains steady forward velocity via a stable PWM output, and the MG996R steering servo dynamically adjusts its centering angle to keep the vehicle aligned with the path.
    *   *Hazard/Obstacle Triggered Branch:* The vehicle immediately executes an embedded evasion protocol. The K0435 shield cuts current to the DC motor to brake instantly. The steering servo shifts to its maximum safe deflection angle away from the obstacle. The DC motor polarity is reversed to back away safely from the boundary line, and the vehicle drives around the hazard before returning control to the standard line-tracking routine.

---

## 4. Implementation Journey: How We Built It

The realization of this vehicle followed a rigorous engineering process, divided into three core development phases:

*   **Phase 1: Power Management & Mechanical Calibration:** We began by assembling the physical chassis layout and stacking the Keyestudio K0435 Motor Shield onto the Arduino R3. Power stabilization was our first major engineering bottleneck; initial testing revealed that when the DC motors started or when the high-torque MG996R steering servo adjusted its heading under load, they created severe inductive voltage spikes. These spikes caused the Arduino to experience brownouts and resets. We resolved this by integrating an HW-131 regulator board with high-drain 18650 lithium cells, effectively separating the logic power rail from the heavy inductive loads.
*   **Phase 2: Vision Optimization & Resolution Tuning:** Developing the perception layer required balancing accuracy against strict processing limitations. Standard image profiles caused complete memory overflows or dropped frames on the ESP32-CAM chip. We overcame this constraint by implementing our custom CIF resolution matrix (400 x 296) and forcing an RGB565 conversion layout. Integrating the `PIXEL_STEP = 2` optimization loop allowed the camera module to perform the full HSV transformation, noise filtering, and centroid mapping smoothly, maintaining a high frame rate without thermal throttling or heap memory depletion.
*   **Phase 3: System Integration & Distributed Testing:** The final phase focused on tuning the communication and behavioral response systems. We wired the serial TX/RX pins from the camera board to the Arduino R3, implementing a clear payload protocol to transmit tracking states. We spent hours conducting track tests to calibrate the exact distance thresholds for the HC-SR04 ultrasonic sensors and tuning the steering servo's mechanical endpoints. This extensive calibration ensured that the vehicle executes smooth, reliable evasion maneuvers without losing track bounds or colliding with perimeter walls.

---

## 5. How to Compile and Upload the Code

Both software profiles are compiled and flashed utilizing the official Arduino IDE development environment:

### Firmware Flashing (Perception - ESP32-CAM)
1. Open the computer vision sketch file (`.ino`) in the Arduino IDE.
2. Navigate to **Tools > Board** and choose **ESP32 Wrover Module**. Then, configure **Tools > Partition Scheme** to **Huge APP (3MB No OTA/1MB SPIFFS)** to ensure the compiled vision libraries fit comfortably into the flash memory.
3. Connect the camera module via an external FTDI programmer or its dedicated daughterboard, map the active COM port under **Tools > Port**, and bridge the `GPIO 0` pin to `GND` using a jumper wire to unlock flashing mode before hitting the reset button.
4. Click the **Upload** arrow icon. Once the process reaches `100%`, disconnect the jumper wire from `GPIO 0` and reset the module to start execution.

### Firmware Flashing (Locomotion - Keyestudio Arduino R3)
1. Open the motion control sketch file (`.ino`) in the Arduino IDE.
2. Navigate to **Tools > Board** and select **Arduino Uno** (the baseline architectural profile required for the Keyestudio R3 board).
3. Connect the board to your computer using a standard USB Type-B cable. **Crucial Safety Step:** Make sure the external 18650 power switch located on the K0435 shield is turned **OFF** during programming to avoid dangerous power feedback loops to your computer.
4. Select the matching active serial port under **Tools > Port**, click the **Upload** arrow icon, and wait for confirmation. Once completed, disconnect the programming cable and switch the main circuit power source **ON** for autonomous tracking.

---

## 6. Acknowledgments & Final Reflections

We would like to express our deepest gratitude to our institution, advisors, and peers who supported us throughout this engineering journey. Special thanks to our team members for their dedication during long debugging sessions, calibrating sensors, and refining the structural code of this vehicle. 

We also extend our sincere appreciation to the organizers of this international competition for providing an exceptional platform that challenges students to push the boundaries of embedded hardware programming, electronics, and autonomous mobile robotics. This project stands as a testament to teamwork, technical perseverance, and practical engineering.

# Sección Español

# WRO-2026-Future-Engineers-Fast Wheels ID

# Documentación de Ingeniería del Vehículo Autónomo

## 1. Introducción y Filosofía de Ingeniería

Este proyecto presenta el desarrollo de un vehículo robótico autónomo diseñado para navegar en un entorno estructurado, rastrear trayectorias específicas y evitar obstáculos de forma dinámica. En lugar de depender de procesadores de alto consumo en el borde (*edge-computing*), computadoras monoplaca costosas o dependencias en la nube, nuestra filosofía de ingeniería se centró en la eficiencia estructural: construir un sistema embebido distribuido capaz de realizar percepción en tiempo real y respuesta electromecánica inmediata. 

Para lograr esto, desacoplamos la arquitectura en dos nodos de procesamiento de hardware dedicados. Un módulo de cámara especializado maneja el procesamiento matricial de alta velocidad, el filtrado a nivel de bits y la visión por computadora; mientras que un microcontrolador central robusto gobierna la orquestación mecánica, la actuación del *shield* de hardware y los bucles de comportamiento dinámico. Al aislar la percepción de la locomoción, alcanzamos una alta estabilidad operativa, minimizamos la latencia de procesamiento y garantizamos que las interrupciones de seguridad se ejecuten con precisión de milisegundos mediante lógica de hardware local. Este enfoque demuestra que se pueden lograr comportamientos autónomos complejos de manera eficiente en microcontroladores mediante un diseño de firmware optimizado y una gestión estricta de recursos.

---

## 2. Arquitectura del Sistema e Integración de Componentes

Nuestro sistema se diseñó en torno a una disposición de controlador dual, dividiendo la percepción y la ejecución física en dos capas de firmware distintas que interactúan sin problemas mediante comunicación serie asíncrona continua:

*   **Controlador de Percepción (ESP32-CAM):** Funciona como el núcleo visual del vehículo. Gestiona la adquisición de búfer de fotogramas a alta velocidad a través del sensor de cámara integrado OV2640 y coordina su LED de estado para señales de sincronización e iluminación flash localizada.
*   **Controlador Central y de Locomoción (Keyestudio Arduino R3):** Funciona como el cerebro principal. Procesa los datos de carga útil en tiempo real enviados por el sistema de visión y coordina los actuadores del vehículo a través de un ecosistema de hardware dedicado:
    *   **Keyestudio K0435 Motor Shield:** Montado directamente sobre el Arduino R3 para manejar la regulación de corriente, protegiendo las compuertas lógicas del microcontrolador mientras entrega energía directa a los motores mediante controladores de puente H integrados.
    *   **Servomotor de Alto Torque MG996R:** Conectado directamente al *shield* K0435 para accionar la geometría de dirección delantera, asegurando ángulos de giro precisos y manteniendo la alineación de la dirección bajo estrés mecánico.
    *   **Motor DC de Tracción:** Impulsado por los bloques de terminales del *shield* para gobernar la rampa de velocidad, el cambio de dirección y el frenado electrónico instantáneo mediante modulación por ancho de pulsos (PWM).
    *   **2x Sensores Ultrasónicos HC-SR04:** Configurados a través de pines de entrada/salida digital para mapear el perímetro físico inmediato, identificando paredes o peligros inesperados en la ruta mediante cálculos de tiempo de vuelo de ondas ultrasónicas.
    *   **Subsistema de Alimentación y Regulación:** Alimentado por un banco de baterías de alta descarga compuesto por celdas recargables de iones de litio 18650 (3.7V cada una), aisladas y estabilizadas a través de un módulo de fuente de alimentación HW-131 para evitar caídas de voltaje durante los picos de consumo de los motores.

---

## 3. Cómo Funciona: Algoritmos y Marco de Toma de Decisiones

### Flujo de Visión por Computadora (Segmentación de Color)
El ESP32-CAM implementa un algoritmo de visión por computadora cíclico y no bloqueante, diseñado para aislar rutas de color específicas dentro del espacio de color HSV (Tono, Saturación, Valor). El flujo paso a paso opera a través de las siguientes etapas:

1.  **Inicialización del Sistema:** Establece los parámetros de depuración serie a una alta velocidad de baudios y configura los registros del sensor de la cámara utilizando una resolución base en formato CIF (400 x 296 píxeles), instanciada mediante `const int CAM_WIDTH = 400;` y `const int CAM_HEIGHT = 296;`. Bloquea propiedades específicas del sensor, incluyendo el balance de blancos, el control de ganancia y los límites de exposición para garantizar la consistencia visual.
2.  **Adquisición de Fotogramas:** Dentro del bucle principal de ejecución, el firmware consulta el componente del sensor OV2640 solicitando una nueva matriz de imagen. El sensor devuelve un flujo JPEG comprimido para optimizar las transferencias DMA internas.
3.  **Descompresión y Traducción de Color:** Dado que un flujo JPEG comprimido no se puede analizar píxel por píxel, el algoritmo utiliza una librería de descompresión interna para traducir el flujo a un arreglo mapa de bits RGB565 sin comprimir, otorgando acceso directo a nivel de bits al canal de color de cada coordenada.
4.  **Escaneo de Optimización por Rejilla:** Para evitar caídas severas de fotogramas y minimizar la latencia de procesamiento del bucle, el algoritmo analiza la matriz de píxeles mediante bucles anidados (los bucles externos controlan las filas en el eje Y; los internos, las columnas en el eje X). En lugar de analizar los 118,400 píxeles, implementa una constante de optimización (`PIXEL_STEP = 2`), evaluando efectivamente uno de cada dos píxeles para mantener una excelente velocidad de procesamiento conservando la precisión estructural.
5.  **Extracción de Bits y Transformación a HSV:** Para cada píxel muestreado, se extraen las intensidades individuales de profundidad de bits de Rojo, Verde y Azul de la estructura RGB565 de 16 bits. Estas variables se transforman matemáticamente al espacio de color HSV. Convertir los datos a Tono (*Hue*), Saturación (*Saturation*) y Valor (*Value*) desacopla el tipo de color real de su componente de brillo. Esto garantiza que el seguimiento de color permanezca altamente inmune a sombras ambientales, reflejos en la pista o cambios de iluminación en el recinto.
6.  **Mitigación Dinámica de Ruido:** Antes de evaluar la identidad de color de un píxel, el firmware pasa las métricas HSV a través de un filtro de validación estricto. Los píxeles que caen por debajo de un umbral mínimo de saturación (tonos lavados o grises) o por debajo de un límite mínimo de valor (sombras profundas y regiones oscuras) se descartan inmediatamente como ruido visual.
7.  **Perfilado y Clasificación de Color:** Los píxeles que superan el filtro de ruido comparan su ángulo exacto de Tono (*Hue*) con límites angulares precisos preprogramados. Estos límites definen el espectro exacto para los marcadores de nuestra pista objetivo, incluyendo los márgenes de Rojo, Verde y Magenta.
8.  **Acumulación de Datos Espaciales:** Cuando un píxel coincide exitosamente con un perfil de color, el sistema incrementa un contador de densidad dedicado para ese color específico. Simultáneamente, suma la coordenada absoluta X y la coordenada Y de ese píxel a variables de sumatoria destinadas al análisis espacial.
9.  **Validación de Umbral:** Una vez que concluye el recorrido de la matriz, el sistema evalúa los contadores de densidad acumulados. Si el número total de píxeles detectados para un color específico no supera un umbral mínimo de validación (`threshold`), todo el clúster se descarta como artefacto de ruido de fondo, evitando falsos positivos.
10. **Cálculo de Centroide y Mapeo de Zonas Horizontales:** Para cualquier clúster de color que supere la validación, el sistema calcula su centro geométrico exacto. El centro espacial se calcula dividiendo la suma de las coordenadas X entre el total de píxeles válidos, y la suma de las coordenadas Y entre la misma cantidad total:

    $$\text{Centro X} = \frac{\text{Suma de coordenadas X}}{\text{Total de píxeles válidos}}$$

    $$\text{Centro Y} = \frac{\text{Suma de coordenadas Y}}{\text{Total de píxeles válidos}}$$

    El plano de escaneo horizontal de 400 píxeles de ancho se divide estructuralmente en tres sectores virtuales distintos: Izquierda, Centro y Derecha. Al evaluar en qué sector cae el valor del Centro X, el vehículo conoce exactamente dónde se ubica el marcador de color con respecto a su chasis, transmitiendo este *token* de inmediato a través de la interfaz serie.
11. **Liberación de Memoria y Ejecución Reciclada:** Para evitar la fragmentación crítica de la memoria *heap* o desbordamientos en el microchip, el búfer de fotogramas activo se limpia por completo y la memoria asignada se libera. El flujo vuelve al inicio sin problemas para capturar el siguiente fotograma entrante en tiempo real.

### Control de Locomoción (El Marco de las 3 Preguntas)
Simultáneamente, el Keyestudio Arduino R3 procesa las cadenas de rastreo entrantes por puerto serie junto con los datos de los sensores locales mediante un ciclo de ejecución de alta velocidad conocido como el **Marco de las Tres Preguntas**:

*   **Pregunta 1: "¿Cuál es el estado actual de mis sensores?"**
    El vehículo muestrea su telemetría ambiental continuamente desde múltiples vectores operativos. El vector visual proporciona *tokens* de color transmitidos desde el ESP32-CAM a través del búfer serie, mientras que el vector espacial calcula las métricas de distancia a las paredes laterales pulsando los dos sensores ultrasónicos HC-SR04.
*   **Pregunta 2: "¿Existe un peligro o desviación en la ruta?"**
    El controlador evalúa las métricas entrantes contra los umbrales de seguridad. En operación estándar, la detección de un color de pista gris combinado con distancias ultrasónicas despejadas confirma el libre paso. Sin embargo, si la cadena visual indica un marcador de límite prohibido (como un *token* Rojo sólido), o si los sensores ultrasónicos detectan que la distancia a una pared cae por debajo de nuestro margen crítico de seguridad, se activa una bandera de interrupción de alta prioridad.
*   **Pregunta 3: "¿Qué acción debo tomar?"**
    Dependiendo de la bandera de estado, el firmware emite comandos al *shield* de motores K0435:
    *   *Rama de Ruta Despejada:* El motor DC de tracción mantiene una velocidad constante hacia adelante a través de una salida PWM estable, y el servo de dirección MG996R ajusta dinámicamente su ángulo de centrado para mantener el vehículo alineado con el camino.
    *   *Rama de Peligro/Obstáculo Detectado:* El vehículo ejecuta inmediatamente un protocolo de evasión embebido. El *shield* K0435 corta la corriente al motor DC para frenar al instante. El servo de dirección gira a su ángulo máximo de deflexión segura en dirección opuesta al obstáculo. La polaridad del motor DC se invierte para retroceder de forma segura desde la línea límite, y el vehículo rodea el peligro antes de devolver el control a la rutina estándar de seguimiento de línea.

---

## 4. Trayectoria de Implementación: Cómo lo Construimos

La realización de este vehículo siguió un riguroso proceso de ingeniería dividido en tres fases principales de desarrollo:

*   **Fase 1: Gestión de Potencia y Calibración Mecánica:** Comenzamos ensamblando el chasis físico y acoplando el *shield* de motores Keyestudio K0435 sobre el Arduino R3. La estabilización de la alimentación fue nuestro primer gran cuello de botella; las pruebas iniciales revelaron que cuando los motores DC arrancaban o cuando el servomotor MG996R ajustaba su dirección bajo carga, se generaban fuertes picos de voltaje inductivo. Estos picos causaban caídas de tensión (*brownouts*) y reinicios en el Arduino. Lo resolvimos integrando una tarjeta reguladora HW-131 con celdas de litio 18650 de alta descarga, separando efectivamente la línea de alimentación lógica de las cargas inductivas pesadas.
*   **Fase 2: Optimización de Visión y Ajuste de Resolución:** Desarrollar la capa de percepción requirió balancear la precisión con las estrictas limitaciones de procesamiento. Los perfiles de imagen estándar causaban desbordamientos de memoria completos o pérdida de fotogramas en el chip ESP32-CAM. Superamos esta restricción implementando nuestra matriz de resolución CIF personalizada (400 x 296) y forzando un formato de conversión RGB565. La integración del bucle de optimización `PIXEL_STEP = 2` permitió que el módulo de la cámara realizara la transformación HSV completa, el filtrado de ruido y el mapeo de centroides con fluidez, manteniendo una alta tasa de fotogramas sin estrangulamiento térmico ni agotamiento de memoria *heap*.
*   **Fase 3: Integración del Sistema y Pruebas Distribuidas:** La fase final se centró en sintonizar la comunicación y los sistemas de respuesta de comportamiento. Conectamos los pines de transmisión y recepción (TX/RX) de la tarjeta de la cámara al Arduino R3, implementando un protocolo de carga útil claro para transmitir los estados de seguimiento. Dedicamos horas a pruebas en pista para calibrar los umbrales de distancia exactos de los sensores ultrasónicos HC-SR04 y ajustar los topes mecánicos del servo de dirección. Esta extensa calibración garantizó que el vehículo ejecute maniobras de evasión suaves y confiables sin perder los límites de la pista ni colisionar con las paredes perimetrales.

---

## 5. Cómo Compilar y Cargar el Código

Ambos perfiles de software se compilan y cargan utilizando el entorno de desarrollo oficial **Arduino IDE**:

### Carga del Firmware (Percepción - ESP32-CAM)
1. Abre el archivo de código de visión por computadora (`.ino`) en el Arduino IDE.
2. Ve a **Herramientas > Placa** y selecciona **ESP32 Wrover Module**. Luego, configura **Herramientas > Esquema de Partición** en **Huge APP (3MB No OTA/1MB SPIFFS)** para asegurar que las librerías de visión compiladas quepan cómodamente en la memoria flash.
3. Conecta el módulo de la cámara mediante un programador FTDI externo o su placa adaptadora dedicada, selecciona el puerto COM activo en **Herramientas > Puerto**, y conecta el pin `GPIO 0` a `GND` usando un cable puente (*jumper*) para desbloquear el modo de programación antes de presionar el botón de reinicio (*reset*).
4. Haz clic en el ícono de la flecha de **Subir**. Una vez que el proceso alcance el `100%`, desconecta el cable puente del `GPIO 0` y reinicia el módulo para iniciar la ejecución.

### Carga del Firmware (Locomoción - Keyestudio Arduino R3)
1. Abre el archivo de código de control de movimiento (`.ino`) en el Arduino IDE.
2. Ve a **Herramientas > Placa** y selecciona **Arduino Uno** (el perfil arquitectónico base requerido para la tarjeta Keyestudio R3).
3. Conecta la tarjeta a tu computadora usando un cable USB Tipo B estándar. **Paso de Seguridad Crucial:** Asegúrate de que el interruptor de encendido externo de las baterías 18650 ubicado en el *shield* K0435 esté **APAGADO** durante la programación para evitar retornos de corriente peligrosos a tu computadora.
4. Selecciona el puerto serie activo correspondiente en **Herramientas > Puerto**, haz clic en el ícono de la flecha de **Subir** y espera la confirmación. Una vez completado, desconecta el cable de programación y enciende la fuente de alimentación principal del circuito para el seguimiento autónomo.

---

## 6. Agradecimientos y Reflexiones Finales

Queremos expresar nuestro más profundo agradecimiento a nuestra institución, asesores y compañeros que nos apoyaron a lo largo de este viaje de ingeniería. Un agradecimiento especial a los miembros de nuestro equipo por su dedicación durante las largas sesiones de depuración, calibrando sensores y refinando el código estructural de este vehículo.

También extendemos nuestro sincero reconocimiento a los organizadores de esta competencia internacional por brindar una plataforma excepcional que desafía a los estudiantes a superar los límites de la programación de hardware embebido, la electrónica y la robótica móvil autónoma. Este proyecto se erige como un testimonio del trabajo en equipo, la perseverancia técnica y la ingeniería práctica.

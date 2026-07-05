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

#include <Servo.h>

// --- CONSTANTES DE PINES ---
const int PIN_SERVO      = 9;
const int PIN_DIR_MOTOR_A = 4;  // Pin de dirección para Motor A
const int PIN_PWM_MOTOR_A = 5;  // Pin de velocidad (PWM) para Motor A

// --- CONSTANTES DE CONFIGURACIÓN ---
// Dirección (Servo)
const int ANGULO_RECTO = 76;
const int ANGULO_DER   = 110;
const int ANGULO_IZQ   = 35;

// Velocidades del motor (0 a 255)
const int VEL_RAPIDA  = 190;
const int VEL_NORMAL  = 150;
const int VEL_STOP    = 0;

// --- OBJETOS ---
Servo direccionServo;

void setup() {
  // Configuración del servo
  direccionServo.attach(PIN_SERVO);
  moverDireccion(ANGULO_RECTO); 
  
  // Configuración de pines del motor Keyestudio
  pinMode(PIN_DIR_MOTOR_A, OUTPUT);
  pinMode(PIN_PWM_MOTOR_A, OUTPUT);
  detenerMotorA(); // Empezar parados por seguridad
}

void loop() {
  rutinaPruebaConMotor();
}

// ========================================================
// --- FUNCIONES DE BAJO NIVEL (Control directo de Hardware) ---
// ========================================================

// Control del Servo
void moverDireccion(int angulo) {
  direccionServo.write(angulo);
}

// Control total del Motor A (Dirección y Velocidad)
void controlarMotorA(boolean direccion, int velocidad) {
  digitalWrite(PIN_DIR_MOTOR_A, direccion);
  analogWrite(PIN_PWM_MOTOR_A, velocidad);
}


// ========================================================
// --- FUNCIONES DE ALTO NIVEL (Acciones lógicas) ---
// ========================================================

// Movimientos del motor
void motorA_Adelante(int velocidad) {
  controlarMotorA(LOW, velocidad);
}

void motorA_Atras(int velocidad) {
  controlarMotorA(HIGH, velocidad);
}

void detenerMotorA() {
  controlarMotorA(HIGH, VEL_STOP);
}

// Movimientos de la dirección (Servo)
void avanzarRecto() { moverDireccion(ANGULO_RECTO); }
void girarIzquierda() { moverDireccion(ANGULO_IZQ); }
void girarDerecha() { moverDireccion(ANGULO_DER); }


// ========================================================
// --- RUTINAS DE PRUEBA ---
// ========================================================
void rutinaPruebaConMotor() {
  // 1. Avanzar recto durante 2 segundos
  avanzarRecto();
  motorA_Adelante(VEL_RAPIDA);
  delay(2000);
 
  //girarIzquierda();
  //delay(500);

  detenerMotorA();
  delay(1000);

 



   //motorA_Atras(VEL_RAPIDA);
  //delay(3000);
  //detenerMotorA();
  //delay(5000);
  
 // 2. Girar a la izquierda sin detener el motor (curva abierta)
  // 2. Girar a la izquierda sin detener el motor (curva abierta)
  //girarIzquierda();
  //delay(2000);
  
  // 3. Detenerse por completo y enderezar ruedas
  //detenerMotorA();
  //avanzarRecto();
  //delay(2000);
  
  // 4. Retroceder girando a la derecha
  //girarDerecha();
  //motorA_Atras(VEL_NORMAL);
  //delay(2000);
  
  // 5. Frenar antes de reiniciar el loop
  //detenerMotorA();
  //delay(3000);
}
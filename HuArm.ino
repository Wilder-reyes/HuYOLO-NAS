#include <ESP32Servo.h> // Librería compatible con ESP32

// Declaración de funciones
void validatePosition(float &Px, float &Py, float &Pz);
void calculateAngles(float Px, float Py, float Pz);
void moveServos(float q1_target, float q2_target, float q3_target, float q4_target);
void gripObject();
void releaseObject();
void moveToRestPosition();

// Declaración de servomotores
Servo servoQ1;
Servo servoQ2;
Servo servoQ3;
Servo servoQ4;
Servo gripperServo;

// Pines para los servos
const int servoQ1Pin = 32;
const int servoQ2Pin = 13;
const int servoQ3Pin = 12;
const int servoQ4Pin = 14;
const int gripperPin = 27;

// Variables para la posición del brazo y contenedor
float Px, Py, Pz;
float Px_container, Py_container, Pz_container;
float q1, q2, q3, q4;

// Variables de la garra
const int gripperOpenAngle = 90; // Ángulo para abrir la garra
const int gripperCloseAngle = 0; // Ángulo para cerrar la garra

// Configuración de movimiento
const int moveDelay = 20; // Tiempo entre cada paso en milisegundos
const int stepSize = 2;   // Tamaño del incremento/decremento de ángulo

void setup() {
  // Inicializar servos
  servoQ1.attach(servoQ1Pin, 500, 2400); // Ancho de pulso para SG-5010
  servoQ2.attach(servoQ2Pin, 500, 2400);
  servoQ3.attach(servoQ3Pin, 500, 2400);
  servoQ4.attach(servoQ4Pin, 500, 2400);
  gripperServo.attach(gripperPin, 500, 2400);

  // Configuración de comunicación serial
  Serial.begin(115200);
  Serial.println("Esperando valores de Px, Py, Pz y contenedor...");
}

void loop() {
  // Leer valores de Px, Py, Pz y posición del contenedor desde el puerto serial
  if (Serial.available() > 0) {
    // Leer posición del objeto
    Px = Serial.parseFloat();
    Py = Serial.parseFloat();
    Pz = Serial.parseFloat();

    // Leer posición del contenedor
    Px_container = Serial.parseFloat();
    Py_container = Serial.parseFloat();
    Pz_container = Serial.parseFloat();

    Serial.print("Recibido - Px: "); Serial.print(Px);
    Serial.print(", Py: "); Serial.print(Py);
    Serial.print(", Pz: "); Serial.println(Pz);
    Serial.print("Contenedor - Px: "); Serial.print(Px_container);
    Serial.print(", Py: "); Serial.print(Py_container);
    Serial.print(", Pz: "); Serial.println(Pz_container);

    // Validar si la posición está dentro del alcance
    validatePosition(Px, Py, Pz);
    validatePosition(Px_container, Py_container, Pz_container);

    // Calcular ángulos para la posición del objeto
    calculateAngles(Px, Py, Pz);

    // Mover servos al objeto
    moveServos(q1, q2, q3, q4);

    // Control de la garra: cerrar, mover al contenedor, y abrir
    gripObject();
    calculateAngles(Px_container, Py_container, Pz_container);
    moveServos(q1, q2, q3, q4);
    releaseObject();

    // Volver a la posición de reposo
    moveToRestPosition();
  }
}

void validatePosition(float &Px, float &Py, float &Pz) {
  float distance = sqrt(Px * Px + Py * Py + Pz * Pz);
  if (distance > 2.1) { // El alcance máximo del brazo es 2.1 unidades
    Serial.println("Error: La posición está fuera del alcance del brazo. Ajustando...");
    Px = 2.1 * Px / distance;
    Py = 2.1 * Py / distance;
    Pz = 2.1 * Pz / distance;
    Serial.print("Nueva posición ajustada - Px: "); Serial.print(Px);
    Serial.print(", Py: "); Serial.print(Py);
    Serial.print(", Pz: "); Serial.println(Pz);
  }
}

void calculateAngles(float Px, float Py, float Pz) {
  // Calcular q1
  q1 = atan2(Py, Px) * 180.0 / PI;
  if (q1 < 0) q1 += 180.0; // Ajustar para valores positivos

  // Calcular q2
  float PxPyMagnitude = sqrt(Px * Px + Py * Py); // Magnitud en el plano XY
  if (PxPyMagnitude == 0) {
    Serial.println("Error: División por cero al calcular q2.");
    q2 = 0;
  } else {
    q2 = atan2(Pz - 0.8, PxPyMagnitude) * 180.0 / PI;
    if (q2 < 0) q2 += 180.0; // Ajustar para valores positivos
  }

  // Calcular q3
  float sqrtPxPy = sqrt(Px * Px + Py * Py);
  float q3Numerator = sqrtPxPy - (0.7 * cos(q2 * PI / 180.0));
  float q3Denominator = 0.8;

  if (q3Denominator == 0 || abs(q3Numerator / q3Denominator) > 1) {
    Serial.println("Error: Valor fuera de rango o división por cero al calcular q3.");
    q3 = 0;
  } else {
    q3 = acos(q3Numerator / q3Denominator) * 180.0 / PI;
    if (q3 < 0) q3 += 180.0; // Ajustar para valores positivos
  }

  // Calcular q4
  float q4Numerator = Pz - 0.8 - (0.7 * sin(q2 * PI / 180.0)) - (0.8 * sin((q2 + q3) * PI / 180.0));
  float q4Denominator = 0.6;

  if (q4Denominator != 0) {
    float value = q4Numerator / q4Denominator;
    if (value > 1.0) value = 1.0; // Límite superior
    if (value < -1.0) value = -1.0; // Límite inferior
    q4 = acos(value) * 180.0 / PI;
    if (q4 < 0) q4 += 180.0; // Ajustar para valores positivos
  } else {
    Serial.println("Error: División por cero al calcular q4.");
    q4 = 0;
  }

  Serial.print("Ángulos calculados - Q1: "); Serial.print(q1);
  Serial.print(", Q2: "); Serial.print(q2);
  Serial.print(", Q3: "); Serial.print(q3);
  Serial.print(", Q4: "); Serial.println(q4);
}

void moveServos(float q1_target, float q2_target, float q3_target, float q4_target) {
  int q1_current = servoQ1.read();
  int q2_current = servoQ2.read();
  int q3_current = servoQ3.read();
  int q4_current = servoQ4.read();

  while (q1_current != q1_target || q2_current != q2_target || q3_current != q3_target || q4_current != q4_target) {
    if (q1_current < q1_target) q1_current += stepSize;
    else if (q1_current > q1_target) q1_current -= stepSize;

    if (q2_current < q2_target) q2_current += stepSize;
    else if (q2_current > q2_target) q2_current -= stepSize;

    if (q3_current < q3_target) q3_current += stepSize;
    else if (q3_current > q3_target) q3_current -= stepSize;

    if (q4_current < q4_target) q4_current += stepSize;
    else if (q4_current > q4_target) q4_current -= stepSize;

    servoQ1.write(constrain(q1_current, 0, 180));
    servoQ2.write(constrain(q2_current, 0, 180));
    servoQ3.write(constrain(q3_current, 0, 180));
    servoQ4.write(constrain(q4_current, 0, 180));

    delay(moveDelay);
  }

  Serial.println("Servos movidos a las posiciones calculadas.");
}

void gripObject() {
  gripperServo.write(gripperCloseAngle);
  Serial.println("Garra cerrada para tomar el objeto.");
  delay(1000);
}

void releaseObject() {
  gripperServo.write(gripperOpenAngle);
  Serial.println("Garra abierta para soltar el objeto.");
  delay(1000);
}

void moveToRestPosition() {
  Serial.println("Moviendo a la posición de reposo...");
  moveServos(90, 90, 90, 90);
  Serial.println("Robot en posición de reposo.");
}

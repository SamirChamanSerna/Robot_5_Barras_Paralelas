#include <Servo.h>

// Definir los pines para los servos
#define SERVO1_PIN 8  // Pin digital para el servo 1
#define SERVO2_PIN 7  // Pin digital para el servo 2

// Longitudes de los brazos del robot (en unidades arbitrarias; en este caso, mm)
#define L0 31.125 // Distancia entre un servo al origen
#define L1 62.5   // Longitud del primer brazo
#define L2 93.75  // Longitud del segundo brazo

// Crear objetos Servo
Servo servo1; // Objeto para el servo 1
Servo servo2; // Objeto para el servo 2

void setup() {
  // Inicializar los servos
  servo1.attach(SERVO1_PIN);  // Asignar el pin al servo 1
  servo2.attach(SERVO2_PIN);  // Asignar el pin al servo 2

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  // Mueve el efector final en un patrón rectangular

  // Movimiento horizontal de izquierda a derecha
  for (int i = -36; i < 37; i++) {
    moverEfectorFinal(i, 75);
    delay(10);  // Pequeña pausa para visualizar el movimiento
  }

  // Movimiento vertical hacia arriba
  for (int j = 75; j < 120; j++) {
    moverEfectorFinal(36, j);
    delay(10);
  }

  // Movimiento horizontal de derecha a izquierda
  for (int k = 36; k > -37; k--) {
    moverEfectorFinal(k, 119);
    delay(10);
  }

  // Movimiento vertical hacia abajo
  for (int l = 120; l > 75; l--) {
    moverEfectorFinal(-36, l);
    delay(10);
  }
}

// Función para mover el efector final a las coordenadas (x, y)
void moverEfectorFinal(float x, float y) {
  // Calcular los ángulos de los servos
  float shoulder1, shoulder2;
  calcularAngulos(x, y, shoulder1, shoulder2);

  // Convertir los ángulos a grados
  float shoulder1Degrees = shoulder1 * 180.0 / PI;
  float shoulder2Degrees = shoulder2 * 180.0 / PI;

  // Mover los servos a las posiciones calculadas
  servo1.write(shoulder1Degrees);
  servo2.write(shoulder2Degrees);

   // Imprimir los ángulos de los servos por el puerto serial (para depuración)
  Serial.print("Servo1 = ");
  Serial.print(shoulder1Degrees);
  Serial.print("\t Servo2 = ");
  Serial.println(shoulder2Degrees);
}

// Función para calcular los ángulos de los servos a partir de las coordenadas (x, y)
void calcularAngulos(float x, float y, float& shoulder1, float& shoulder2) {
  // Calcular el ángulo beta1 (ángulo desde el hombro izquierdo al efector final)
  float beta1 = atan2(y, (L0 + x));

  // Calcular el ángulo beta2 (ángulo desde el hombro derecho al efector final)
  float beta2 = atan2(y, (L0 - x));

  // Pre-cálculos para el ángulo alfa (usando la ley de cosenos)
  float alpha1_calc = (L1 * L1 + ((L0 + x) * (L0 + x) + y * y) - L2 * L2) / (2 * L1 * sqrt((L0 + x) * (L0 + x) + y * y));
  float alpha2_calc = (L1 * L1 + ((L0 - x) * (L0 - x) + y * y) - L2 * L2) / (2 * L1 * sqrt((L0 - x) * (L0 - x) + y * y));

  // Si los cálculos son > 1, la función acos fallará (punto inalcanzable)
  if (alpha1_calc > 1 || alpha2_calc > 1) {
    Serial.println("Coordenadas inalcanzables");
    return;
  }

  // Calcular el ángulo alpha1 (ángulo del hombro izquierdo)
  float alpha1 = acos(alpha1_calc);

  // Calcular el ángulo alpha2 (ángulo del hombro derecho)
  float alpha2 = acos(alpha2_calc);

  // Calcular el ángulo final del hombro izquierdo (shoulder1)
  shoulder1 = beta1 + alpha1;

  // Calcular el ángulo final del hombro derecho (shoulder2)
  shoulder2 = PI - beta2 - alpha2;
}
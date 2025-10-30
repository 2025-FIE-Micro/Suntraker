#include <AccelStepper.h>

// PINES DE MOTOR PAP
#define PIN_ENABLE 8 // Habilitación compartida para ambos motores

#define PAP_DIR_X 5
#define PAP_STEP_X 2

#define PAP_DIR_Z 7
#define PAP_STEP_Z 4

// Configuración de los motores
AccelStepper motor_horizontal(1, PAP_STEP_X, PAP_DIR_X);
AccelStepper motor_vertical(1, PAP_STEP_Z, PAP_DIR_Z);

void setup() 
{
  // Configuración de pines
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW); // LOW para activar los motores

  // Configuración del motor horizontal
  motor_horizontal.setMaxSpeed(1000);   // Velocidad máxima (STEPS por segundo)
  motor_horizontal.setAcceleration(500); // Aceleración (STEPS/seg^2)
  motor_horizontal.setCurrentPosition(0); // Posición inicial

  // Configuración del motor vertical
  motor_vertical.setMaxSpeed(1000);      // Velocidad máxima (sin movimiento real, pero necesaria)
  motor_vertical.setAcceleration(500);  // Aceleración
  motor_vertical.setCurrentPosition(0); // Posición inicial
  motor_vertical.moveTo(0);             // Mantener la posición actual (bloqueado)

  // Iniciar monitor serie
  Serial.begin(9600);
  Serial.println("Iniciando prueba de motores...");

  // Configurar el movimiento inicial del motor horizontal
  motor_horizontal.moveTo(-6400); // 3200 pasos = 1 vuelta completa si tienes 200 pasos/rev y un microstepping de 16
}

void loop() 
{
  // Mantener el motor vertical bloqueado
  motor_vertical.run(); // Aunque no se mueve, mantiene la posición
  motor_vertical.stop();

  // Ejecutar movimientos del motor horizontal
  motor_horizontal.run();

  // Cambiar dirección del motor horizontal al llegar al destino
  if (motor_horizontal.distanceToGo() == 0) {
    Serial.println("Cambiando dirección del motor horizontal...");
    motor_horizontal.moveTo(-motor_horizontal.currentPosition()); // Ir en la dirección opuesta
  }
}

#include <AccelStepper.h>

//HABILITAR MODULO
#define ENABLE_PIN 8

//PINES DE CONTROL - DES-COMENTAR UNO Y COMENTAR EL OTRO
//DIRECCION Z
#define PAP_DIR_Z 7
#define PAP_STEP_Z 4

//DIRECCION X
#define PAP_DIR_X 5
#define PAP_STEP_X 2

AccelStepper motor_x(1, PAP_STEP_X, PAP_DIR_X);
AccelStepper motor_z(1, PAP_STEP_Z, PAP_DIR_Z);

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // LOW habilita el motor en la mayoría de los controladores

  motor_x.setMaxSpeed(1000);
  motor_x.setAcceleration(200);
  motor_x.setCurrentPosition(0); // Posición inicial
  
  motor_z.setMaxSpeed(1000);
  motor_z.setAcceleration(200);
  motor_z.setCurrentPosition(0); // Posición inicial

  motor_x.move(3200);
}

void loop() 
{
  motor_z.stop();
  motor_x.run();
}

#include <AccelStepper.h>

// PINES DE MOTOR PAP
#define PIN_ENABLE 8
#define PAP_DIR_X 5
#define PAP_STEP_X 2
#define PAP_DIR_Z 7
#define PAP_STEP_Z 4

// PINES DE LOS SENSORES LDR
#define ldr_W A0
#define ldr_S A1
#define ldr_N A2
#define ldr_E A3

// CONSTANTES
#define VEL_MAX 2000
#define ACC_MAX 1000

#define MAX_Y 1000

#define ERROR_MINIMO_X 30
#define ERROR_MINIMO_Y 30

// VARS DE LECTURA DE LOS SENSORES LDR
int valor_ldr_W = 0, valor_ldr_S = 0, valor_ldr_N = 0, valor_ldr_E = 0;

// Error y pasos a mover
int error_horizontal = 0, error_vertical = 0;
int pasos_horizontal = 0, pasos_vertical = 0;

// GANANCIAS 
#define Kp 10       // Ajuste de ganancia proporcional
#define Ki 0.65      // Ajuste de ganancia integral

// Integral acumulada del error 
float integral_error_horizontal = 0, integral_error_vertical = 0;

float delta_t = 0;
unsigned long tiempo_anterior = 0; // Para calcular ∆t

#define INTERVALO_MOSTRAR 500 // Intervalo de 500 ms
unsigned long tiempo_inicio = 0;  // Para mostrar la informacion en el serial

// Crear objetos de AccelStepper para los motores X y Z
AccelStepper motor_x(1, PAP_STEP_X, PAP_DIR_X);
AccelStepper motor_z(1, PAP_STEP_Z, PAP_DIR_Z);


void setup() {
  // Habilitar motores
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW); // LOW para activar los motores

  // Iniciar monitor serie
  Serial.begin(9600);
  Serial.println("\nIniciando seguidor solar con control proporcional...");

  // Configurar las velocidades y aceleraciones de los motores
  motor_x.setMaxSpeed(VEL_MAX); // Reduce la velocidad
  motor_x.setAcceleration(ACC_MAX); // Reduce la aceleración
  motor_x.setCurrentPosition(0); // Posición inicial

  motor_z.setMaxSpeed(VEL_MAX);
  motor_z.setAcceleration(ACC_MAX);
  motor_z.setCurrentPosition(0); // Posición inicial

  motor_x.move(0);
  motor_z.move(0);

  tiempo_inicio = millis();
}


void loop() 
{
  unsigned long tiempo_actual = millis();
  float delta_t = (tiempo_actual - tiempo_anterior) / 1000.0; // Convertir ms a segundos
  tiempo_anterior = tiempo_actual;

  valor_ldr_W = analogRead(ldr_W);
  valor_ldr_S = analogRead(ldr_S);
  valor_ldr_N = analogRead(ldr_N);
  valor_ldr_E = analogRead(ldr_E);
  
  // Calcular errores
  error_horizontal = valor_ldr_E - valor_ldr_W; // Error entre Este y Oeste
  error_vertical = valor_ldr_N - valor_ldr_S;   // Error entre Norte y Sur

  // Transformamos el error mediante un controlador PI
  controladorPI(error_horizontal, error_vertical, delta_t);

  // Mostramos los valores de los ldr, los errores y los pasos
  mostrarDatos();

  // Indicamos los movimientos a realizar 
  ajustarDireccionHorizontal(error_horizontal, pasos_horizontal);
  ajustarDireccionVertical(error_vertical, pasos_vertical);
  
  // Continuamente mueve el motor hasta que se haya completado el movimiento
  while (motor_x.isRunning() || motor_z.isRunning()) {
    motor_x.run();  // Mantenemos el motor en movimiento
    motor_z.run();  // Mantenemos el motor Z en movimiento
  }
}

void controladorPI(int error_horizontal, int error_vertical, float delta_t)
{
  // Aplicar margen muerto a los errores
  if (abs(error_horizontal) < ERROR_MINIMO_X) 
  {
    error_horizontal = 0;
    integral_error_horizontal = 0;
  }
  if (abs(error_vertical) < ERROR_MINIMO_Y) 
  {
    error_vertical = 0;
    integral_error_vertical = 0;
  }

  // Actualizar integrales
  integral_error_horizontal += error_horizontal * delta_t;
  integral_error_vertical += error_vertical * delta_t;

  // Calcular pasos basados en el error y la constante proporcional
  pasos_horizontal = Kp * error_horizontal + Ki * integral_error_horizontal;

  pasos_vertical = Kp * error_vertical + Ki * integral_error_vertical;
  pasos_vertical = constrain(Kp * error_vertical, -MAX_Y, MAX_Y);
}


// Mostramos los datos en cada INTERVALO
void mostrarDatos()
{  
  if (millis() - tiempo_inicio >= INTERVALO_MOSTRAR) 
  {
    tiempo_inicio = millis(); // Actualizamos el tiempo inicial

    mostrarLDR();
    mostrarErrores();
    mostrarPasos();
  }
}


// Ajustar dirección horizontal (motor X)
void ajustarDireccionHorizontal(int error, int pasos) 
{
  if (error > 0) {// Error positivo: ESTE es mayor que OESTE
                  // Mover hacia el este
    //Serial.println("Moviendo hacia el OESTE...");
    motor_x.move(-pasos);  // Mover motor X en la cantidad calculada
  } else if (error < 0) { // Error negativo: mover hacia el oeste
    //Serial.println("Moviendo hacia el ESTE...");
    motor_x.move(-pasos);  // Mover motor X en la cantidad calculada en dirección contraria
  }
}


// Ajustar dirección vertical (motor Z)
void ajustarDireccionVertical(int error, int pasos) 
{
  if (error > 0) { // Error positivo: NORTE mayor a SUR 
                   // mover hacia el norte
    //Serial.println("Moviendo hacia el SUR...");
    motor_z.move(-pasos);  // Mover motor Z en la cantidad calculada
  } else if (error < 0) { // Error negativo: mover hacia el sur
    //Serial.println("Moviendo hacia el NORTE...");
    motor_z.move(-pasos);  // Mover motor Z en la cantidad calculada en dirección contraria
  }
}


// Mostrar los LDRS en el monitor serial
void mostrarLDR()
{
  Serial.println("\n-----------------------------------------------------------------");
  Serial.print("Sensores -");
  Serial.print("LDR_W: "); Serial.print(valor_ldr_W);
  Serial.print("  | LDR_S: "); Serial.print(valor_ldr_S);
  Serial.print("  | LDR_N: "); Serial.print(valor_ldr_N);
  Serial.print("  | LDR_E: "); Serial.println(valor_ldr_E);  
}


// Mostrar los errores en el monitor serial
void mostrarErrores()
{
  Serial.print("Error Horizontal: "); Serial.print(error_horizontal);
  Serial.print("  | Error Vertical: "); Serial.println(error_vertical);

  Serial.print("Error INTEGRATIVO  Horizontal: "); Serial.print(integral_error_horizontal);
  Serial.print("  | Error INTEGRATIVO Vertical: "); Serial.println(integral_error_horizontal);
}


// Mostrar los pasos en el monitor serial
void mostrarPasos()
{
  Serial.print("Pasos Horizontal: "); Serial.print(pasos_horizontal);
  Serial.print("  | Pasos Vertical: "); Serial.println(pasos_vertical);
}
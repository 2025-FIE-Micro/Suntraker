// PINES DE MOTOR PAP
#define PIN_ENABLE 8

#define PAP_DIR_X 5
#define PAP_STEP_X 2

#define PAP_DIR_Z 7
#define PAP_STEP_Z 4

#define tiempo_min 250    //tiempo maximo de pulso alto y bajo
#define tiempo_max 700    //tiempo minimo de pulso alto y bajo

const int velocidad = tiempo_min;       // En realidad, son tiempos, pero la vel era inversamente prop a esto

const int cantidadPasos_x = 1000;
const int cantidadPasos_z = 1000;


void setup() 
{
  Serial.begin(9600);

  // Habilitar motores
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW); // LOW para activar los motores

  pinMode(PAP_DIR_X, OUTPUT);
  pinMode(PAP_DIR_Z, OUTPUT);
  pinMode(PAP_STEP_X , OUTPUT);
  pinMode(PAP_STEP_Z , OUTPUT);

  //mueve el equipo de manera vertical
  //  moverMotor(PAP_DIR_Z, PAP_STEP_Z, cantidadPasos_z);

  //mueve el equipo de manera mixta
  //  moverMotor(PAP_DIR_X, PAP_STEP_X, cantidadPasos_x);

  /*mueve el equipo de manera horizontal
  //  moverMotores(cantidadPasos_x); //DERECHA
  //  delay(1000);
  //  moverMotores(-1 * cantidadPasos_x); //IZQUIERDA
  */
}


void loop() {
  if (Serial.available()) {
    // Leer el carácter inicial (x o z)
    char comando = Serial.read();

    // Esperar el valor de los pasos después del punto
    int pasos = Serial.parseInt();

    if (pasos != 0) {
      if (comando == 'x') {
        Serial.print("Moviendo motores (x) ");
        Serial.print(pasos);
        Serial.println(" pasos.");
        moverMotores(pasos); // Llamada para el caso 'x'
      } else if (comando == 'z') {
        Serial.print("Moviendo motor Z ");
        Serial.print(pasos);
        Serial.println(" pasos.");
        moverMotor(PAP_DIR_Z, PAP_STEP_Z, pasos); // Llamada para el caso 'z'
      } else {
        Serial.println("Comando no reconocido. Usa 'x.(pasos)' o 'z.(pasos)'.");
      }
      delay(1000);
    } else {
      Serial.println("Por favor, introduce un número válido.");
    }
  }
}


// Función para mover un motor paso a paso
void moverMotor(int pinDir, int pinStep, int pasos)
{
  if(pasos<0) 
  {
    digitalWrite(pinDir, LOW);
    pasos = -pasos;
  }
  else digitalWrite(pinDir, HIGH);

  for (int i = 0; i < pasos; i++) 
  {
    digitalWrite(pinStep, HIGH);  // Pulso alto
    delayMicroseconds(velocidad);               // Ajustar para velocidad
    digitalWrite(pinStep, LOW);   // Pulso bajo
    delayMicroseconds(velocidad);
  }
}


// Función para los motor paso a paso, giro puramente Horizontal
void moverMotores(int pasos)
{
  if(pasos<0)
  {

    digitalWrite(PAP_DIR_X, LOW);
    digitalWrite(PAP_DIR_Z, LOW);
    pasos = -pasos;
  }
  else
  {
    digitalWrite(PAP_DIR_X, HIGH);
    digitalWrite(PAP_DIR_Z, HIGH);
  }

  for (int i = 0; i < pasos; i++) 
  {
    digitalWrite(PAP_STEP_Z, HIGH);
    digitalWrite(PAP_STEP_X, HIGH);
    
    delayMicroseconds(velocidad);      // Ajustar para velocidad

    digitalWrite(PAP_STEP_Z, LOW);
    digitalWrite(PAP_STEP_X, LOW);
    delayMicroseconds(velocidad);
  }
}
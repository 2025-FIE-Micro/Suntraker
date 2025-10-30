//PRUEBA DE LDRS SEGUIDOR SOLAR 

// PINES DE LOS SENSORES LDR
#define ldr_W A0
#define ldr_S A1
#define ldr_N A2
#define ldr_E A3

float error_horizontal = 0, error_vertical = 0;

#define ERROR_MINIMO_X 0  //10
#define ERROR_MINIMO_Z 0  //10

#define NUM_LECTURAS 10


void setup() 
{
  Serial.begin(115200);
  Serial.println("Verificacion de sensores LDR...");
}


void loop() 
{
  Serial.println("-----------------------------------------------------------------");

  int valor_ldr_W = leerSuavizado(ldr_W);
  int valor_ldr_S = leerSuavizado(ldr_S);
  int valor_ldr_N = leerSuavizado(ldr_N);
  int valor_ldr_E = leerSuavizado(ldr_E);

  Serial.println("Sensores.");
  Serial.print("LDR_W: ");
  Serial.println(valor_ldr_W);
  Serial.print("LDR_E: ");
  Serial.println(valor_ldr_E);
  Serial.print("LDR_S: ");
  Serial.println(valor_ldr_S);
  Serial.print("LDR_N: ");
  Serial.println(valor_ldr_N);

  error_horizontal = valor_ldr_E - valor_ldr_W;  // Error entre Este y Oeste

  error_vertical = valor_ldr_N - valor_ldr_S;  // Error entre Norte y Sur

  Serial.print("Error Horizontal: ");
  Serial.println(error_horizontal);
  Serial.print("Error Vertical: ");
  Serial.println(error_vertical);

  delay(500); // Pausa para evitar saturar el monitor serie
}


int leerSuavizado(int pin) {
    long suma = 0;
    for (int i = 0; i < NUM_LECTURAS; i++) {
        suma += analogRead(pin);
        delay(5); // Pequeña espera entre lecturas
    }
    return suma / NUM_LECTURAS;
}


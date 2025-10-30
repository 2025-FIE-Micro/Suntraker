// TRABAJO INTEGRADOR TC 2024
// CODIGO DE ARDUINO PARA SOLAR TRACKER

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
#define PASOS 50              // Número de pasos a mover en cada ajuste
#define TIEMPO_DE_ESPERA 200  // Tiempo de espera entre ajustes de dirección (en milisegundos)
#define LDR_MIN 0             // Valor mínimo del mapeo de los LDR
#define LDR_MAX 1023          // Valor máximo del mapeo de los LDR

// VARS DE LECTURA DE LOS SENSORES LDR
int valor_ldr_W = 0;
int valor_ldr_S = 0;
int valor_ldr_N = 0;
int valor_ldr_E = 0;

void setup() {
  // Configuración de pines de motores
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PAP_DIR_X, OUTPUT);
  pinMode(PAP_STEP_X, OUTPUT);
  pinMode(PAP_DIR_Z, OUTPUT);
  pinMode(PAP_STEP_Z, OUTPUT);

  // Habilitar motores
  digitalWrite(PIN_ENABLE, LOW); // LOW para activar los motores

  // Iniciar monitor serie
  Serial.begin(9600);
  Serial.println("Iniciando seguidor solar...");
}

void loop() {
  // Leer valores de los sensores LDR y mapear a un rango útil
  valor_ldr_W = map(analogRead(ldr_W), LDR_MIN, LDR_MAX, 100, 0);
  valor_ldr_S = map(analogRead(ldr_S), LDR_MIN, LDR_MAX, 100, 0);
  valor_ldr_N = map(analogRead(ldr_N), LDR_MIN, LDR_MAX, 100, 0);

  //Posible lectura incorrecta.
  valor_ldr_E = map(analogRead(ldr_E), LDR_MIN, LDR_MAX, 100, 0);

  // Mostrar valores de los LDR en la consola
  mostrarLDRS(valor_ldr_N, valor_ldr_S, valor_ldr_E, valor_ldr_W);

  // Comparar valores para ajustar dirección
  ajustarDireccionHorizontal(valor_ldr_W, valor_ldr_E);
  ajustarDireccionVertical(valor_ldr_N, valor_ldr_S);

  // Pausa para evitar movimientos rápidos
  delay(TIEMPO_DE_ESPERA);
}

// Mostrar valores de los sensores en consola
void mostrarLDRS(int N, int S, int E, int W) {
  Serial.print("LDR_W: "); Serial.print(W);
  Serial.print(" | LDR_S: "); Serial.print(S);
  Serial.print(" | LDR_N: "); Serial.print(N);
  Serial.print(" | LDR_E: "); Serial.println(E);
}

// Ajustar dirección horizontal (motor X)
void ajustarDireccionHorizontal(int ldrW, int ldrE) {
  if (ldrW > ldrE + 5) { // Si la luz es mayor en el oeste
    Serial.println("Moviendo hacia el oeste...");
    moverMotor(PAP_DIR_X, PAP_STEP_X, PASOS, true); // Mover hacia el oeste
  } else if (ldrE > ldrW + 5) { // Si la luz es mayor en el este
    Serial.println("Moviendo hacia el este...");
    moverMotor(PAP_DIR_X, PAP_STEP_X, PASOS, false); // Mover hacia el este
  }
}

// Ajustar dirección vertical (motor Z)
void ajustarDireccionVertical(int ldrN, int ldrS) {
  if (ldrN > ldrS + 5) { // Si la luz es mayor en el norte
    Serial.println("Moviendo hacia el norte...");
    moverMotor(PAP_DIR_Z, PAP_STEP_Z, PASOS, true); // Mover hacia el norte
  } else if (ldrS > ldrN + 5) { // Si la luz es mayor en el sur
    Serial.println("Moviendo hacia el sur...");
    moverMotor(PAP_DIR_Z, PAP_STEP_Z, PASOS, false); // Mover hacia el sur
  }
}



// Función para mover un motor paso a paso
void moverMotor(int pinDir, int pinStep, int pasos, bool direccion) {
  digitalWrite(pinDir, direccion ? HIGH : LOW); // Configurar dirección
  for (int i = 0; i < pasos; i++) {
    digitalWrite(pinStep, HIGH); // Pulso alto
    delayMicroseconds(4000);    // Ajustar para velocidad
    digitalWrite(pinStep, LOW); // Pulso bajo
    delayMicroseconds(4000);
  }
}

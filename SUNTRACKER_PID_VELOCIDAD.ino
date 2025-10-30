// ------------------- CONSTANTES -------------------
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

// Tiempo para mostrar la informacion en el serial
#define INTERVALO_MOSTRAR 500  // Intervalo de 500 ms
unsigned long tiempo_inicio = 0;

// PUNTO MUERTO DEL SISTEMA
#define ERROR_MINIMO_X 2
#define ERROR_MINIMO_Z 2

//PARA EL AJUSTE DE VELOCIDAD 
#define ERROR_MAX 1023

// VARS DE LECTURA DE LOS SENSORES LDR
int valor_ldr_W = 0, valor_ldr_S = 0, valor_ldr_N = 0, valor_ldr_E = 0;

// CONSTANTES CONTROLADOR PID
float Kp = 235;   //Ajuste de ganancia proporcional ()
float Ki = 0.01;  //Ajuste de ganancia integral     ()
float Kd = 0.001;   //Ajuste de ganancia derivativa   ()

// CALCULO DE dt
float tiempo_anterior = 0;

// ERRORES DEL CONTROLADOR
float senial_control_horizontal = 0, senial_control_vertical = 0;
float error_horizontal = 0, error_vertical = 0;
float integral_error_horizontal = 0, integral_error_vertical = 0;
float derivada_error_horizontal = 0, derivada_error_vertical = 0;  // No lo acumulamos, lo usamos para mostrar cuanto fue el error en el loop, luego vuelve a 0
float error_horizontal_previo = 0, error_vertical_previo = 0;

// Límites en velocidad minima y maxima.
#define T_VEL_MIN 250
#define T_VEL_MAX 750
int tiempo_pulso_vertical = T_VEL_MIN;
int tiempo_pulso_horizontal = T_VEL_MIN;

String direccion_horizontal = "Sin movimiento";  // Dirección del movimiento horizontal
String direccion_vertical = "Sin movimiento";    // Dirección del movimiento vertical

#define NUM_LECTURAS 3
#define PASOS 10


// ------------------- SETUP -------------------
void setup() {
  // Habilitar motores
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);

  pinMode(PAP_DIR_X, OUTPUT);
  pinMode(PAP_DIR_Z, OUTPUT);
  pinMode(PAP_STEP_X, OUTPUT);
  pinMode(PAP_STEP_Z, OUTPUT);

  // Iniciar monitor serie
  Serial.begin(115200);

  // Comentamos por si queremos realizar SOLO la grafica de los errores.
  iniciarSeguidor();

  tiempo_inicio = millis();
}


// ------------------- LOOP -------------------
void loop() {
  leerEntradaSerial();

  // Calcular errores
  calcularErrores();

  // Transformamos el error mediante un controlador P
  controladorPID();

  // Mostramos los valores de los ldr, los errores, los pasos, las constantes y la direccion.
  // mostrarDatos();   // Comentamos por si queremos realizar la grafica de los errores.

  // Mostramos solo los errores para ver la grafico en el plotter
  plotErrores();
  
  // Indicamos los movimientos a realizar
  orientarSeguidor();
}


// ------------------- FUNCIONES -------------------
// Leemos el serial para recibir el valor de Kp, Ki, y Kd
void leerEntradaSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Leer hasta el salto de línea
    ajustarConstantes(input);                     // Llamar a la función que procesa el comando
  }
}


// Funcion para iniciar el seguidor solar.
void iniciarSeguidor() {
  Serial.println("-----------------------------------------------------------------\n");
  Serial.println("Iniciando seguidor solar con control PID.");
  mostrarConstantesControlador(6);
  Serial.println("\nIngresa 'P(nuevo_valor)' para cambiar Kp.");
  Serial.println("\nIngresa 'I(nuevo_valor)' para cambiar Ki.");
  Serial.print("\nIngresa 'D(nuevo_valor)' para cambiar Kd.");

  // Mostramos como se cambian las cts por el monitor serial
  for (int i = 0; i < 5; i++) {
    delay(INTERVALO_MOSTRAR);
    Serial.print(".");
  }
  Serial.println(".");
}


// Funcion para procesar el grafico del error (SERIALPLOTTER)
void plotErrores() {
  Serial.print("Error_Horizontal:");
  Serial.print(error_horizontal);   // Error horizontal
  Serial.print(",");
  Serial.print("Error_Vertical:");
  Serial.print(error_vertical);     // Error vertical
  Serial.print(",");              
  Serial.print("Max:");
  Serial.print(700);                // Valor maximo
  Serial.print(",");
  Serial.print("Min:");
  Serial.println(-700);             // Valor minimo
}


// Función para procesar y cambiar Kp o Ki si es válido
void ajustarConstantes(String comando) {
  char tipo = comando.charAt(0);  // Obtener el primer carácter ('P' o 'I')

  // Verificar si el comando tiene la estructura correcta
  if (comando.startsWith("P(") || comando.startsWith("I(") || comando.startsWith("D(") || comando.startsWith("p(") || comando.startsWith("i(") || comando.startsWith("d(")) {
    if (comando.endsWith(")")) {
      String valorStr = comando.substring(2, comando.length() - 1);  // Extraer el valor entre paréntesis
      float nuevoValor = valorStr.toFloat();                         // Convertir a número

      // Verificar si el valor es válido
      if (nuevoValor >= 0) {
        switch (tipo) {
          case 'p':  // Cambiar Kp
          case 'P':
            Kp = nuevoValor;
            Serial.print("\nNuevo valor de Kp: ");
            Serial.println(Kp);
            break;

          case 'i':  // Cambiar Ki
          case 'I':
            Ki = nuevoValor;
            Serial.print("\nNuevo valor de Ki: ");
            Serial.println(Ki);
            break;

          case 'd':  // Cambiar Kd
          case 'D':
            Kd = nuevoValor;
            Serial.print("\nNuevo valor de Kd: ");
            Serial.println(Kd);
            break;

          default:  // No debería entrar aquí
            Serial.println("ERROR inesperado.");
            break;
        }
      } else {
        Serial.println("ERROR: el valor debe ser mayor o igual a 0.");
      }
    } else {
      Serial.println("ERROR: falta el cierre del paréntesis.");
    }
  } else {
    Serial.println("Comando no reconocido. Usa 'P(valor)' o 'I(valor)'.");
  }
}


// Realizamos N cantidad de lecturas analogicas
int leerSuavizado(int pin) {
  long suma = 0;
  for (int i = 0; i < NUM_LECTURAS; i++) {
      suma += analogRead(pin);
      delayMicroseconds(5); // Pequenia espera entre lecturas
  }
  return suma / NUM_LECTURAS;
}


// Calculamos el error entre los ejes
void calcularErrores() {
  valor_ldr_W = leerSuavizado(ldr_W);
  valor_ldr_S = leerSuavizado(ldr_S);
  valor_ldr_N = leerSuavizado(ldr_N);
  valor_ldr_E = leerSuavizado(ldr_E);

  error_horizontal  = valor_ldr_E - valor_ldr_W;  // Error entre Este y Oeste
  error_vertical    = valor_ldr_N - valor_ldr_S;  // Error entre Norte y Sur
}


// Ajustamos la velocidad de acorde al error que exista entre sus ejes
void ajustarVelocidad(int error, int *tiempo_pulso) {
  *tiempo_pulso = map(abs(error), 0, ERROR_MAX, T_VEL_MAX, T_VEL_MIN);
  *tiempo_pulso = constrain(*tiempo_pulso, T_VEL_MIN, T_VEL_MAX);
}


// Transformamos el error prop, int y derivativamente a la velocidad del motor
void controladorPID() {
  float tiempo_actual = millis();

  float dt = (tiempo_actual - tiempo_anterior) / 1000.0;  // Convertir a segundos

  // Acumular el error integral
  integral_error_horizontal += error_horizontal * dt;
  integral_error_vertical   += error_vertical * dt;

  derivada_error_horizontal = 0;
  derivada_error_vertical   = 0;

  if (dt > 0) {
    derivada_error_horizontal = (error_horizontal - error_horizontal_previo)  / dt;
    derivada_error_vertical   = (error_vertical - error_vertical_previo)      / dt;
  }

  // Calcular pasos proporcionales e integrales al error
  senial_control_horizontal = Kp * error_horizontal + Ki * integral_error_horizontal  + Kd * derivada_error_horizontal;
  senial_control_vertical   = Kp * error_vertical   + Ki * integral_error_vertical    + Kd * derivada_error_vertical;

  ajustarVelocidad(senial_control_vertical, &tiempo_pulso_vertical);
  ajustarVelocidad(senial_control_horizontal,   &tiempo_pulso_horizontal);

  error_horizontal_previo = error_horizontal;
  error_vertical_previo =   error_vertical;
  tiempo_anterior = tiempo_actual;
}


// Movemos el seguidor y indicamos en que direccion se movio.
void orientarSeguidor() {
  // Zona muerta
  if( abs(error_horizontal) <= ERROR_MINIMO_X && abs(error_vertical) <= ERROR_MINIMO_Z) digitalWrite(PIN_ENABLE, HIGH);
  else   digitalWrite(PIN_ENABLE, LOW);

  // Movimiento horizontal
  if (error_horizontal < 0) {
    direccion_horizontal = "Izquierda";
    moverHorizontal("Izquierda");

  } else if (error_horizontal > 0) {
    direccion_horizontal = "Derecha";
    moverHorizontal("Derecha");

  } else {
    direccion_horizontal = "Sin movimiento";
  }

  // Movimiento vertical
  if (error_vertical > 0) {
    direccion_vertical = "Abajo";
    moverVertical("Abajo");

  } else if (error_vertical < 0) {
    direccion_vertical = "Arriba";
    moverVertical("Arriba");

  } else {
    direccion_vertical = "Sin movimiento";
  }
}


// Función para mover un motor paso a paso
void moverVertical(String direccion) {
  if (direccion == "Abajo") {
    digitalWrite(PAP_DIR_Z, LOW);
  } else if (direccion == "Arriba"){
    digitalWrite(PAP_DIR_Z, HIGH);
  }

  int paso = 0;
  while(paso < PASOS) {
    digitalWrite(PAP_STEP_Z, HIGH);            // Pulso alto
    delayMicroseconds(tiempo_pulso_vertical);  // Ajustar para velocidad
    digitalWrite(PAP_STEP_Z, LOW);             // Pulso bajo
    delayMicroseconds(tiempo_pulso_vertical);
    paso++;
  }
}


// Función para los motor paso a paso, giro puramente Horizontal
void moverHorizontal(String direccion) {
  if (direccion == "Izquierda") {
    digitalWrite(PAP_DIR_X, LOW);
    digitalWrite(PAP_DIR_Z, LOW);
  } else if (direccion == "Derecha") {
    digitalWrite(PAP_DIR_X, HIGH);
    digitalWrite(PAP_DIR_Z, HIGH);
  }

  int paso = 0;
  while(paso < PASOS) {
    digitalWrite(PAP_STEP_Z, HIGH);
    digitalWrite(PAP_STEP_X, HIGH);
    delayMicroseconds(tiempo_pulso_horizontal);  // Ajustar para velocidad

    digitalWrite(PAP_STEP_Z, LOW);
    digitalWrite(PAP_STEP_X, LOW);
    delayMicroseconds(tiempo_pulso_horizontal);
    paso++;
  }
}


// Mostramos todos los datos en cada INTERVALO
void mostrarDatos() {
  // Cuando la diferencia de marcas temporales supere el intervalo, muestra los datos
  if (millis() - tiempo_inicio >= INTERVALO_MOSTRAR) {
    tiempo_inicio = millis();

    Serial.println("\n-----------------------------------------------------------------");
    mostrarLDR();
    mostrarErrores();
    mostrarDireccion();
    mostrarTiemposDePulso();
    mostrarConstantesControlador(6);
  }
}


// Mostrar las constantes Kp, Ki, Kd
void mostrarConstantesControlador(int cifras) {
  Serial.println("\n== Constantes controlador PID ==");
  Serial.print("Kp: ");
  Serial.print(Kp, cifras);
  Serial.print("  | Ki: ");
  Serial.print(Ki, cifras);
  Serial.print("  | Kd: ");
  Serial.println(Kd, cifras);
}


// Mostrar el pulso que usa en cada direccion
void mostrarTiemposDePulso() {
  Serial.println("\n== Tiempos de Pulso Actuales ==");
  Serial.print("Tiempo Pulso Vertical: ");
  Serial.print(tiempo_pulso_vertical);
  Serial.println(" ms");

  Serial.print("Tiempo Pulso Horizontal: ");
  Serial.print(tiempo_pulso_horizontal);
  Serial.println(" ms");
}


// Mostrar los LDRS en el monitor serial
void mostrarLDR() {
  Serial.println("\n== Sensores ==");
  Serial.print("LDR_W: ");
  Serial.println(valor_ldr_W);
  Serial.print("LDR_E: ");
  Serial.println(valor_ldr_E);
  Serial.print("LDR_S: ");
  Serial.println(valor_ldr_S);
  Serial.print("LDR_N: ");
  Serial.println(valor_ldr_N);
}


// Mostrar los errores en el monitor serial
void mostrarErrores() {
  Serial.println("\n== Errores entre sensores ==");
  Serial.print("Error Horizontal: ");
  Serial.print(error_horizontal);
  Serial.print("  | Error Vertical: ");
  Serial.println(error_vertical);

  Serial.println("\n== Errores integrativos  ==");
  Serial.print("Error Integral Horizontal: ");
  Serial.print(integral_error_horizontal);
  Serial.print("  | Error Integral Vertical: ");
  Serial.println(integral_error_vertical);

  Serial.println("\n== Errores derivativos  ==");
  Serial.print("Error derivativo Horizontal: ");
  Serial.print(derivada_error_horizontal);
  Serial.print("  | Error derivativo Vertical: ");
  Serial.println(derivada_error_vertical);
}


// Mostrar la direccion del seguidor
void mostrarDireccion() {
  Serial.println("\n== Direcciones ==");
  Serial.print("Horizontal: ");
  Serial.println(direccion_horizontal);
  Serial.print("Vertical: ");
  Serial.println(direccion_vertical);
}
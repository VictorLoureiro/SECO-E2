/*************/    
/* VARIABLES */ 
/*************/

/* Pines del motor */
int motorPinA = 4;
int motorPinB = 5;

/* Pines del encoder */
int encoderA = 7;
int encoderB = 3;

/* Valores y estados de los encoders */
volatile int counter = 0;
int valorA = 0;
int valorB = 0;
long T0 = 0;
volatile int estado = 0;
volatile int estado_aux = 0;
void contador();

void setup() {

  /* Iniciamos puerto serie */
  Serial.begin(9600);

  /* Leemos posición inicial del motor para saber en qué estado se encuentra y poder determinar su posicion */
  valorA = digitalRead(encoderA);
  valorB = digitalRead(encoderB);

  if (valorA == HIGH && valorB == LOW) {
    estado_aux = 1;
  }
  else if (valorA == HIGH && valorB == HIGH) {
    estado_aux = 2;
  }
  else if (valorA == LOW && valorB == HIGH) {
    estado_aux = 3;
  }
  else if (valorA == LOW && valorB == LOW) {
    estado_aux = 4;
  }

  /* Configuramos los pines del motor como salidas */
  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);

  /* Configuramos los pines del encoder como entradas */
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA), contador, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), contador, CHANGE);
}

void loop() {
  Serial.println(counter);
}

void contador() {
  valorA = digitalRead(encoderA);
  valorB = digitalRead(encoderB);

  if (valorA == HIGH && valorB == LOW) {
    estado = 1;
  }
  else if (valorA == HIGH && valorB == HIGH) {
    estado = 2;
  }
  else if (valorA == LOW && valorB == HIGH) {
    estado = 3;
  }
  else if (valorA == LOW && valorB == LOW) {
    estado = 4;
  }

  switch (estado) {
    case 1:
      if (estado_aux == 2) {
        counter--;
      } else if (estado_aux == 4)  {
        counter++;
      }
      estado_aux = estado;
      break;

    case 2:
      if (estado_aux > estado) {
        counter--;
      } else {
        counter++;
      }
      estado_aux = estado;
      break;

    case 3:
      if (estado_aux > estado) {
        counter--;
      } else {
        counter++;
      }
      estado_aux = estado;
      break;

    case 4:
      if (estado_aux == 1) {
        counter--;
      } else if (estado_aux == 3) {
        counter++;
      }
      estado_aux = estado;
      break;
  }
}

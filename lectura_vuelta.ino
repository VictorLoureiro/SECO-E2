int motorPinA = 4;
int motorPinB = 5;

int encoderA = 7;
int encoderB = 3;

volatile int counter = 0;
int valorA = 0;
int valorB = 0;
long T0 = 0;

volatile int estado = 0;
volatile int estado_aux = 0;
void contador();

void setup() {
  Serial.begin(9600); //Iniciar puerto serie

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

  pinMode(motorPinA, OUTPUT);
  pinMode(motorPinB, OUTPUT);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(7), contador, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), contador, CHANGE);
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

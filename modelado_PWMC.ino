#include <DueTimer.h> // Libreria Timer de Arduino Due, para utilizar los Timer y sus funciones

/*************/    
/* VARIABLES */ 
/*************/

/* Pines para PWM a 20kHz */
int pinPWMH1 = 35;
int pinPWMH2 = 37;

/* Pines para control de interrupciones (lectura del ENCODER) */
int encoderA = 7;
int encoderB = 3;

/* Tension de entrada del escalon */
double voltaje = 1;
 
/* Valores de los encoders */
int valorA = 0;
int valorB = 0;

/* Estados de los encoders */
volatile int estadoActual = 0;
volatile int estadoAnterior = 0;

/* Posicion actual del encoder */
volatile int posicionActual = 0; 

/* Array para guardar las posiciones del motor recogidas */
int posicion[1201];

/* Contador para el timer cada 1ms */
volatile int timerCntr = 0; 

/*************/
/* FUNCIONES */
/*************/

/* Asignar a los pines el control PWMC */
void SetPin(uint8_t pin) {
  PIO_Configure(g_APinDescription[pin].pPort,
                PIO_PERIPH_B, 
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}

/* Ajusta la tensión en funcion del ciclo de trabajo */
void setVoltage (double voltage) {
  if(voltage>=12) {
    PWMC_SetDutyCycle(PWM, 1, 2100); 
    PWMC_SetDutyCycle(PWM, 0, 0); 
  } else if(voltage<=-12){
    PWMC_SetDutyCycle(PWM, 0, 2100); 
    PWMC_SetDutyCycle(PWM, 1, 0); 
  } else{
    if(voltage>0){
      PWMC_SetDutyCycle(PWM, 1, (voltage/12)*2100); 
      PWMC_SetDutyCycle(PWM, 0, 0); 
    } else if(voltage<0){
      PWMC_SetDutyCycle(PWM, 0, -(voltage/12)*2100); 
      PWMC_SetDutyCycle(PWM, 1, 0); 
    } else if(voltage==0){
      PWMC_SetDutyCycle(PWM, 1, 0); 
      PWMC_SetDutyCycle(PWM, 0, 0); 
    }
  }
}

/* Calcula la posición del enconder */
void contador() {
  valorA = digitalRead(encoderA);
  valorB = digitalRead(encoderB);

  if (valorA == HIGH && valorB == LOW) {estadoActual=1; }
  else if (valorA == HIGH && valorB == HIGH) {estadoActual=2; }
  else if (valorA == LOW && valorB == HIGH) {estadoActual=3; }
  else if (valorA == LOW && valorB == LOW) {estadoActual=4; }

  switch(estadoActual){
    case 1: 
      if (estadoAnterior == 2) {
        posicionActual--;
      } else if (estadoAnterior == 4)  {
        posicionActual++;
      }
      estadoAnterior=estadoActual;
      break;

    case 2:
      if (estadoAnterior > estadoActual) {
        posicionActual--;
      } else {
        posicionActual++;
      }
      estadoAnterior=estadoActual;
      break;
    
    case 3:
      if (estadoAnterior > estadoActual) {
        posicionActual--;
      } else {
        posicionActual++;
      }
      estadoAnterior=estadoActual;
      break;
    
    case 4:
      if (estadoAnterior == 1) {
        posicionActual--;
      } else if (estadoAnterior == 3) {
        posicionActual++;
      }
      estadoAnterior=estadoActual;
      break;    
  }
}

/* Movemos el motor mediante una entrada escalon y guardamos la posicion del encoder en el array */ 
void moverMotor() {
  if ( timerCntr < 1201 ){
    posicion[timerCntr] = posicionActual; 
    if(timerCntr<601){
      setVoltage(voltaje);
    } else{
      setVoltage(0);
    }
  }
  timerCntr++; // Aumenta el contador cada 1ms
}

/* Imprimimos el numero muestra y su valor en el terminal */
void imprimir() {
  for(unsigned i=0; i<=1200; i++){
      Serial.println(posicion[i]);
  }
}

void setup() {
  /* Iniciamos puerto serie */
  Serial.begin(115200); 
  
  /* Habilitamos el ENABLE */
  pinMode(2, OUTPUT); 
  digitalWrite(2,HIGH); 
  
  /* Configuramos los pines del encoder como entradas */
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
    

  /* Configuramos los dos canales PWM a 20 KHz */
  pmc_enable_periph_clk(PWM_INTERFACE_ID); //Inicializamos el reloj del sistema (lo pone a 42MHz, porque lleva un prescaler interno)
  
  /* Canal 0 (PWMH1) */
  PWMC_ConfigureChannel (PWM, 0, 1, 0, 0); 
  PWMC_SetPeriod(PWM, 0, 2100); 
  PWMC_SetDutyCycle(PWM, 0, 0); 
  PWMC_EnableChannel(PWM, 0);
  
  /* Canal 1 (PWMH2) */
  PWMC_ConfigureChannel (PWM, 1, 1, 0, 0); 
  PWMC_SetPeriod(PWM, 1, 2100); 
  PWMC_SetDutyCycle(PWM, 1, 0); 
  PWMC_EnableChannel(PWM, 1);

  /* Configuramos PWM para mover motor */
  SetPin(pinPWMH1); //PWMH0
  SetPin(pinPWMH2); //PWMH1

  /* Leemos posición inicial del motor para saber en qué estado se encuentra y poder determinar su posicion */
  valorA = digitalRead(encoderA); 
  valorB = digitalRead(encoderB);
  
  if (valorA == HIGH && valorB == LOW) {estadoAnterior=1; }
  else if (valorA == HIGH && valorB == HIGH) {estadoAnterior=2; }
  else if (valorA == LOW && valorB == HIGH) {estadoAnterior=3; }
  else if (valorA == LOW && valorB == LOW) {estadoAnterior=4; }

  /* Saltamos a la función contador cada vez que las señales del encoder cambian de valor */
  attachInterrupt(digitalPinToInterrupt(encoderA), contador, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), contador, CHANGE);

  /* Interrumpimos cada 1ms para leer la posición del encoder y almacenar los datos en un array */
  Timer3.attachInterrupt(moverMotor);
  Timer3.start(1000);
}


void loop() {
  if(timerCntr>=1201){
    Timer3.stop();
    imprimir();
    while(1);
  }
}

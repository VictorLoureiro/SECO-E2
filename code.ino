int pinAmotor = 4; //giro a der
int pinBmotor = 5; //giro a izq

/*********************/
/*     Variables     */
/*********************/

/* Pines para el control de las interrupciones */
int pinA = 7; // Cable Amarillo = encoder A output
int valorA = 0; // Valor encoder A

int pinB = 3; // Cable Blanco = encoder B output
int valorB = 0; // Valor encoder B

/* Variables para la posicion del encoder */
int estado = 0;
int estado_anterior = 0;

/* Pines donde se habilita la salida PWM de 20kHz */
int pinPWMH1 = 35;
int pinPWMH2 = 37;

/* Variable para el contador del encoder */
volatile int counter = 0;

/* Contador para el timer */
volatile int timerCntr = 0;

/* Constante de PI */
#define PI 3.1415

/********************************************/
/* Descripcion de las funciones intermedias */
/********************************************/

/* Calcula la posición del enconder */
void posicionEncoder() {
  if ((valorA == LOW) && (valorB == LOW)) {
    estado = 1;
  }
  if ((valorA == HIGH) && (valorB == LOW)) {
    estado = 2;
  }
  if ((valorA == HIGH) && (valorB == HIGH)) {
    estado = 3;
  }
  if ((valorA == LOW) && (valorB == HIGH)) {
    estado = 4;
  }
  switch (estado_anterior) {
    case 1:
      if (estado == 2) {
        counter --;
        estado_anterior = estado;
        break;
      } else if (estado == 4) {
        counter ++;
        estado_anterior = estado;
        break;
      }
    case 2:
      if (estado == 3) {
        counter --;
        estado_anterior = estado;
        break;
      } else if (estado == 1) {
        counter ++;
        estado_anterior = estado;
        break;
      }
    case 3:
      if (estado == 4 ) {
        counter --;
        estado_anterior = estado;
        break;
      } else if (estado == 2) {
        counter ++;
        estado_anterior = estado;
        break;
      }
    case 4:
      if (estado == 1) {
        counter --;
        estado_anterior = estado;
        break;
      } else if (estado == 3 ) {
        counter ++;
        estado_anterior = estado;
        break;
      }
    default:
      break;
  }
}

/* Función para la conversión en radiandes de la posición del encoder */
double rad( int counter){
  //calculamos la posicion en radianes de acuerdo con la resolucion que tenemos con el encoder y la reductora
  double y = (double)(counter * 2 * PI) / (12*75);
  return y;
}

void setup() {
  // Configuramos como salida los dos pines del motor
  pinMode(pinAmotor, OUTPUT);
  pinMode(pinBmotor, OUTPUT);

  // Configuramos como entrada los dos pines del encoder
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  // Asiganmos los valores del encoder a A y B
  valorA = digitalRead(pinA);
  valorB = digitalRead(pinB);

  /* Modificación de la frecuencia del PWM para conseguir 20kHz en los dos canales del motor */
  // Habilitar el controlador del periférico PWM
  pmc_enable_periph_clk(PWM_INTERFACE_ID); 
  // Canal 0 (PWMH1)
  PWMC_ConfigureChannel (PWM, 0, 1, 0, 0); 
  PWMC_SetPeriod(PWM, 0, 2100); //Canal 0, Periodo: 1/(2100/42 Mhz) = 20 kHz
  PWMC_SetDutyCycle(PWM, 0, 0); 
  PWMC_EnableChannel(PWM, 0);
  // Canal 1 (PWMH2)
  PWMC_ConfigureChannel (PWM, 1, 1, 0, 0); 
  PWMC_SetPeriod(PWM, 1, 2100); //Canal 1, Periodo: 1/(2100/42 Mhz) = 20 kHz
  PWMC_SetDutyCycle(PWM, 1, 0); 
  PWMC_EnableChannel(PWM, 1);

  // Funciones de atención a interrupciones Hardware: para obtener la posicion del motor
  attachInterrupt( digitalPinToInterrupt(pinA), posicionEncoder, CHANGE);
  attachInterrupt( digitalPinToInterrupt(pinB), posicionEncoder, CHANGE); 

  // Empleamos la interrupción del timer cada 1ms para leer la posición del motor
  Timer3.attachInterrupt(controlador_P);
  Timer3.start(1000);

}

void loop() {
  printf("%d\n", counter);
}

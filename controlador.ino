#include <DueTimer.h> // Libreria Timer de Arduino Due, para utilizar los Timer y sus funciones
#include <vector>     // Librería para crear el vector de count

/* ****************************************************************************************************************************************************************************** */
/*                                                            DESCRIPCION DE LAS VARIABLES GLOBALES UTILIZADAS                                                                    */
/* ****************************************************************************************************************************************************************************** */

//Pines de salida de PWM de 20 kHz
int pwmPin1 = 35;
int pwmPin2 = 37;

//Pines de control de interrupciones de PWM
int pinIntA = 7;
int pinIntB = 3;

int count; //Contador de cambios, salida del sistema con codificacion del encoder

/* Variables para el control de posicion con interrupciones*/
char valorActual;
char valorAnterior;

/* Vector donde se almacenan los valores de count cada ms*/
std::vector<int> valores={0};

//Variable de control de impresion para el modelado del Motor
int indice;

/* modificar PWM */
uint32_t maxDutyCount = 2100; //12 voltios
uint32_t channel_1 = g_APinDescription[pwmPin1].ulPWMChannel;
uint32_t channel_2 = g_APinDescription[pwmPin2].ulPWMChannel;
uint32_t prescaler = 1;

/* Constantes del controlador*/
const double pi = 3.141592;
const double referencia=2*pi; //Con lo que hay que jugar

/* Parametros del controlador*/
const double kp = 188.4922;

/* Variables de la no linealidad*/
 double b[12] = {0};
       
        

/* ****************************************************************************************************************************************************************************** */
/*                                                            DESCRIPCION DE LAS FUNCIONES INTERMEDIAS                                                                            */
/* ****************************************************************************************************************************************************************************** */

/* *********************************************  Funcion de lectura de los pines de interrupcion *************************************************************** */
char obtenerValorActual() {

  if (digitalRead(pinIntA) == LOW && digitalRead(pinIntB) == LOW)
    return 'a';

  if (digitalRead(pinIntA) == LOW && digitalRead(pinIntB) == HIGH)
    return 'b';

  if (digitalRead(pinIntA) == HIGH && digitalRead(pinIntB) == LOW)
    return 'c';

  if (digitalRead(pinIntA) == HIGH && digitalRead(pinIntB) == HIGH)
    return 'd';
}


/* ****************************************************************************************************************************************************************************** */
/*                                                                 DESCRIPCION DEL CONTROLADOR P                                                                                  */
/* ****************************************************************************************************************************************************************************** */

/* ***********************************************************  Funcion para mover el motor con PWM de 20kHz ******************************************************************** */

// Configuracion del ciclo de trabajo segun el nivel de tensión de la señal de referencia para mover el motor
void setTension(double x){
   if(x>=12.0){
    PWMC_SetDutyCycle(PWM_INTERFACE, 1, maxDutyCount);
     PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0);
   }else if(x<=(-12.0)){
    PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0);
    PWMC_SetDutyCycle(PWM_INTERFACE, 0, maxDutyCount);
   }
  else{
    double a = maxDutyCount/12.0;
    if (x>0.0){
      double b = a*x;
      PWMC_SetDutyCycle(PWM_INTERFACE, 1, (uint32_t)b);
      PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0);
    }else if(x<0.0){
      double b = -a*x;
      PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0);
      PWMC_SetDutyCycle(PWM_INTERFACE, 0, (uint32_t)b);
    }else if (x == 0.0){
      PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0);
      PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0);
    }    
  }
}


/* ***********************************************************  Funciones de covertir la salida a radianes o a la codificacion del encoder ************************************* */
// Valor de la posicion en radianes
double getPosition(int numero){
  double rad=0.0;
  rad = (((double)numero)/3591.8)*2*pi;
  return rad;
}

// Valor de la señal a la codificación del encoder
double getPositionInversa(double numero){
  double pos=0.0;
  pos = (numero*3591.8)/(2*pi);
  return pos;
}
/* ***********************************************************  Funcion de call-back del controlador **************************************************************************** */

/* Se ejecuta cada 12 ms para obtener los puntos por el valor de la salida para la señal de referencia introducida */
void controlador(){
    
    double e = 0.0;
    double u =0.0;
    double u_f =0.0;

    e = referencia-getPosition(count);
    u = kp*e;
    if (u>=12.0)
    
        setTension(12.0);
    
    else if(u<=-12.0)
    
        setTension(-12.0);
    
    else {         
        for (int i = 0; i<12;i++){
            u_f +=b[i]*pow(u,((2*i)+1)); //Ajustamos la ecuacion porque nuestro indice empieza en 0 (no en 1)
            setTension(u_f);
         }
    }

   valores.push_back(count);
  
}


/* ****************************************************************************************************************************************************************************** */
/*                                           DESCRIPCION DE LAS FUNCIONES DE MPDIFICACION DE LOS VALORES DE LOS PINES                                                             */
/* ****************************************************************************************************************************************************************************** */
/* *********************************************  Funcion para el control de la posicion del motor  ********************************************************************* */

// Se obtiene el valor de la posicion actual(numero entre 0 y 1800-si se da una vuelta, o mayor si se dan mas vueltas y positivo o negativo segun el sentido de giro)

void controlPosicion() {

  // Cuando se produce una interrupcion, leo los valores de los pines en ese momento  y estudio segun lo que tenía en el Valor anterior en que direccion me he movido.
  valorActual = obtenerValorActual();
  switch (valorAnterior) {

    case 'a':
      if (valorActual == 'b') {
        count++;
        valorAnterior = valorActual;
        break;
      }
      if (valorActual == 'c') {
        count--;
        valorAnterior = valorActual;
        break;
      }

    case 'c':
      if (valorActual == 'd') {
        count--;
        valorAnterior = valorActual;
        break;
      }
      if (valorActual == 'a') {
        count++;
        valorAnterior = valorActual;
        break;
      }

    case 'd':
      if (valorActual == 'b') {
        count--;
        valorAnterior = valorActual;
        break;
      }
      if (valorActual == 'c') {
        count++;
        valorAnterior = valorActual;
        break;
      }

    case 'b':
      if (valorActual == 'a') {
        count--;
        valorAnterior = valorActual;
        break;
      }
      if (valorActual == 'd') {
        count++;
        valorAnterior = valorActual;
        break;
      }

    default:
      break;

  }
}


/* *********************************************  Funcion de call-back para mover el motor, generando un pulso (entrada escalon) para el modelado ******************************************************** */

void modeladoMotor() {
    
    if(indice<600){
      setTension(1.0);
    }
    else if(600<indice<1200) {
      setTension(0.0);
    }
    
    valores.push_back(count);
    //indice++;

}
/* ****************************************************************************************************************************************************************************** */
/*                                                                 CONFIGURACION INICIAL                                                                                          */
/* ****************************************************************************************************************************************************************************** */

/* **********************************************************************  SETUP *************************************************************************************** */

void setup() {

  /* Habilitamos el enable */
  pinMode(2, OUTPUT);
  digitalWrite(2,HIGH);

  /*PWMC*/
  //Habilitar los perifericos de control de PWM.
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
 
  PIO_Configure(
    g_APinDescription[pwmPin1].pPort,
    PIO_PERIPH_B,
    g_APinDescription[pwmPin1].ulPin,
    g_APinDescription[pwmPin1].ulPinConfiguration);

  PIO_Configure(
    g_APinDescription[pwmPin2].pPort,
    PIO_PERIPH_B,
    g_APinDescription[pwmPin2].ulPin,
    g_APinDescription[pwmPin2].ulPinConfiguration);

  //Habilitar los canales utilizados(pines) y los parametros de configuracion.
  PWMC_ConfigureChannel(PWM_INTERFACE, 0 , prescaler, 0, 0);  
  PWMC_SetPeriod(PWM_INTERFACE, 0, maxDutyCount); 
  PWMC_EnableChannel(PWM_INTERFACE, 0);
  PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0); // entre -2100 , 2100

  //Habilitar el canal 12 y los parametros de configuracion.
  PWMC_ConfigureChannel(PWM_INTERFACE, 1, prescaler, 0, 0);
  PWMC_SetPeriod(PWM_INTERFACE, 1, maxDutyCount); 
  PWMC_EnableChannel(PWM_INTERFACE, 1);
  PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0); // entre -2100 , 2100 


  //Iniciamos la variable para la impresion de los resultados de Modelado Motor a 0
  indice = 0;

  /* Control por interrupciones*/
  pinMode(pinIntA, INPUT);
  pinMode(pinIntB, INPUT);

  /* Pines del MOTOR*/
  //pinMode(motorPin1, OUTPUT);
  //pinMode(motorPin2, OUTPUT);

  
  /* Inicializacion de las variables de control */
  // Ponemos el contador de cambios a 0;
  count = 0;
  
  /* Iniciar valorAnterior a valorActual */
  valorActual = obtenerValorActual();
  valorAnterior = valorActual;

  /* Inicializacion de los valores de la no linealidad */
    b[0] = 1.1622;
    b[1] = -0.0603;
    b[2] = 0.0116;
    b[3] = -0.0012;
    b[4] = 7.2752e-5;
    b[5] = -2.7735e-6;
    b[6] = 6.8172e-8;
    b[7] = -1.0923e-9;
    b[8] = 1.1289e-11;
    b[9] = -7.2278e-14;
    b[10] = 2.5984e-16;
    b[11] = -3.9983e-19;

  /* Con interrupciones Hardware: para obtener la posicion del motor*/
  //Se inicia la lectura de interrupciones
  attachInterrupt(digitalPinToInterrupt(pinIntA), controlPosicion, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinIntB), controlPosicion, CHANGE);
  
  /* Con interrupciones por Timer: para mover el motor*/
  /* Interrupcion cada 1000 microsegundos (1ms) */

  /* Para el modelado con el que se obtendrán los valores de kp */
  //Timer3.attachInterrupt(modeladoMotor).setPeriod(1000).start();

  /* Para el controlador final */
  Timer3.attachInterrupt(controlador).setPeriod(12000).start();

  /* Para la consola*/
  Serial.begin(115200);
}


/* ****************************************************************************************************************************************************************************** */
/*                                                            OBTENCION DE RESULTADOS POR PANTALLA                                                                                */
/* ****************************************************************************************************************************************************************************** */

/* *********************************************  Funcion de impresión de los resultados por consola *************************************************************** */

void imprime() {  
  Serial.print("Muestra");
  Serial.print("\t");
  Serial.println("Valor");
  for(unsigned long k=0 ;k<valores.size();k++){
    
    Serial.print(k);
    Serial.print("\t");
    Serial.println(valores[k]);
  }
}

/* ****************************************************************************************************************************************************************************** */
/*                                                                    MAIN                                                                                                        */
/* ****************************************************************************************************************************************************************************** */
/* *****************************************************************  LOOP ********************************************************************************************* */

void loop() {

  if(referencia > 0){
    if (count >= getPositionInversa(referencia)){
       imprime();
       while(1);
    }
  }
   else{
      if (count <= getPositionInversa(referencia)){
        imprime();
        while(1);
      }
   }
}

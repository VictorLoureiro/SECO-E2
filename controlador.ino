#include <DueTimer.h> // Libreria Timer de Arduino Due, para utilizar los Timer y sus funciones
#include <vector>     // Librería para crear el vector de count

/********************************/    
/* Descripcion de las variables */ 
/********************************/

  const double pi = 3.141592;

/* Pines para PWM a 20kHz */
  int pwmPin1 = 35;
  int pwmPin2 = 37;

/* Pines para control de interrupciones */
  int pinA = 7;
  int pinB = 3;

/* Variables del controlador */ 
  const double ref = -2*pi;        //Entrada del controlador
  const double kp = 37.5284543;  //Valor optimo (x75) para CPR=48
  int counter = 0;                //Salida del controlador

/* Variables para saber la posicion del encoder */
  int estado = 0;
  int estado_anterior = 0;

/* Vector donde se almacenan los valores de count cada ms */
  std::vector<int> valores={0};
  int indice = 0;
  double b[12];    

/*Modificamos PWM*/
  uint32_t maxDutyCount = 2100;  //+12 Voltios
  uint32_t channel_1 = g_APinDescription[pwmPin1].ulPWMChannel;
  uint32_t channel_2 = g_APinDescription[pwmPin2].ulPWMChannel;
  uint32_t prescaler = 1;

/********************************************/    
/* Descripcion de las funciones intermedias */ 
/********************************************/

/* Calcula la posición del enconder */
void posicionEncoder(){
  if((digitalRead(pinA)==LOW)&&(digitalRead(pinB)==LOW)){ estado = 1;}
  if((digitalRead(pinA)==HIGH)&&(digitalRead(pinB)==LOW)){ estado = 2;}
  if((digitalRead(pinA)==HIGH)&&(digitalRead(pinB)==HIGH)){ estado = 3;}
  if((digitalRead(pinA)==LOW)&&(digitalRead(pinB)==HIGH)){ estado = 4;}
  switch(estado_anterior){
    case 1:
      if(estado == 2){
         counter --;
         estado_anterior = estado;
         break;
      }else if(estado == 4){
         counter ++;
         estado_anterior = estado;
         break;     
      }
    case 2:
      if(estado == 3){
        counter --;
        estado_anterior = estado;
        break;
      }else if(estado == 1){
        counter ++;
        estado_anterior = estado;
        break;     
      }
    case 3:
      if(estado == 4 ){
        counter --;
        estado_anterior = estado;
        break;
      }else if(estado == 2){
        counter ++;
        estado_anterior = estado;
        break;     
      }
    case 4:
      if(estado == 1){
        counter --;
        estado_anterior = estado;
        break;
      }else if(estado ==3 ){
        counter ++;
        estado_anterior = estado;
        break;     
      }
    default: 
      break;
    }
}
  
/* Convierte la salida 'y' en radianes */
double getPosition(int num){
  double rad = 0.0;
  rad = ((double)num)*2*pi/3591.8;
  return rad;
}

/* Convierte la entrada 'ref' en valor del encoder */
double getPositionInver(double num){
  double posicion = 0.0; 
  posicion = ((double)num)*3591.8/(2*pi);
  return posicion;
}
/* Relaciona la tensión con el ciclo de trabajo */  
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
    if (x>=0.0){
      double b = a*x;
      PWMC_SetDutyCycle(PWM_INTERFACE, 1, (uint32_t)b);
      PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0);
    }else if(x<0.0){
      double b = -a*x;
      PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0);
      PWMC_SetDutyCycle(PWM_INTERFACE, 0, (uint32_t)b);
    } 
  }
}

/* Controlador final */
 void controlador(){
  double e = 0.0;
  double u = 0.0;
  double uf = 0.0;

  e = ref - getPosition(counter);
  u = kp*e;
  
  if (u>=12.0){
    setTension(12.0);
  }
  else if (u<=-12.0){
    setTension(-12.0);
  }
  else
  { 
    for(int i=0; i<12; i++){
      uf+=b[i]*pow(u,(2*i+1)); //Ajustamos la ecuación debido al cambio de índice del vector b
      setTension(uf);
    }
   }
  valores.push_back(counter);
}


/***** SETUP *****/
void setup() {
 /* Habilitamos el enable */
 pinMode(2, OUTPUT);
 digitalWrite(2,HIGH);
  
 Serial.begin(115200);

/* Valores de las constantes b obtenidas en Matlab */ 
  b[0] = 1.1622;
  b[1] = -0.0613;
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
  
 /* PWMC */
 //Enable PWM controller peripheral
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

 //Enable channel1 and set parameters
 PWMC_ConfigureChannel(PWM_INTERFACE, 0 , prescaler, 0, 0);  
 PWMC_SetPeriod(PWM_INTERFACE, 0, maxDutyCount); 
 PWMC_EnableChannel(PWM_INTERFACE, 0);
 PWMC_SetDutyCycle(PWM_INTERFACE, 0, 0); // entre 0 , 2100

 //Enable channel2 and set parameters
 PWMC_ConfigureChannel(PWM_INTERFACE, 1, prescaler, 0, 0);
 PWMC_SetPeriod(PWM_INTERFACE, 1, maxDutyCount); 
 PWMC_EnableChannel(PWM_INTERFACE, 1);
 PWMC_SetDutyCycle(PWM_INTERFACE, 1, 0); // entre 0 , 2100

  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  if((digitalRead(pinA)==LOW)&&(digitalRead(pinB)==LOW)){ estado = 1;}
  if((digitalRead(pinA)==HIGH)&&(digitalRead(pinB)==LOW)){ estado = 2;}
  if((digitalRead(pinA)==HIGH)&&(digitalRead(pinB)==HIGH)){ estado = 3;}
  if((digitalRead(pinA)==LOW)&&(digitalRead(pinB)==HIGH)){ estado = 4;}

  estado_anterior = estado;

 /* Con interrupciones Hardware: para obtener la posicion del motor */
  attachInterrupt( digitalPinToInterrupt(pinA), posicionEncoder, CHANGE);
  attachInterrupt( digitalPinToInterrupt(pinB), posicionEncoder, CHANGE); 

  /* Con interrupciones por Timer: para mover el motor */
  Timer3.attachInterrupt(controlador).setPeriod(12000).start();
  //Timer3.attachInterrupt(modeladoMotor).setPeriod(1000).start();
}

void imprime() {

  Serial.print("Muestra");
  Serial.print("\t");
  Serial.println("Valor");
  
  for(unsigned k=0 ;k<valores.size();k++){
    Serial.print(k);
    Serial.print("\t");
    Serial.println(valores[k]);
  }
}

/*****LOOP*****/
void loop() {
  if (ref >= 0){
    if(counter >= getPositionInver(ref)){
      imprime();
      while(1);
     }
  } 
  else{
    if(counter <= getPositionInver(ref)){
      imprime();
      while(1);
    }
  }
}

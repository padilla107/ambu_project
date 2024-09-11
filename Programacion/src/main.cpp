#include <Arduino.h> //14-08-24 002
#include "EEPROM.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Chrono.h> 
#include <HX711.h>
#include <AccelStepper.h>

LiquidCrystal_I2C lcd(0x27,20,4);

bool prueba_core0=0;

HX711 O_PRESION;
HX711 O_CELDA;

AccelStepper stepper (1,26,25);  // (tipo driver=1 , pull, direccion) pin

const int PRESION_DOUT_PIN = 18;//18
const int PRESION_SCK_PIN = 19;//19

const int CELDA_DOUT_PIN = 27;//27
const int CELDA_SCK_PIN = 14;//14

/////////////////////////////////////////
bool enciende=0;
bool b_inicio=0;
int prueba=0;
int pin_entrada=23;
bool estado_input=0;
///////////////////////////////////////
int Pin_lcd12=34;
int valor_lcd12=0;

int pin_lcd34=35;
int valor_lcd34=0;

float PRESION_CERO=0;
float SENSOR_PRESION=0;
int PRESION=0;
bool trigg=0;
int P_SUMA=0;
float CELDA_CERO=0;
float SENSOR_CELDA=0;
float CELDA=0;
float pos_celda=0;    // dato positivo de celda
int C_SUMA=0;


bool light_on=0;

int n_inspira=1;
bool insp_boton=0;
bool insp_presion=0;
bool insp_tiempo=0;


bool estado_boton_start;
bool estado_boton_stop;
int boton_start=12;                  //pin de entrada
int boton_stop=13;                   //pin de entrda
int luz_amarillo=2;//8             //pin salida
int luz_rojo=15;                     //pin salida
int luz_verde=4;                   //pin salida
int buzzer=5;                      //pin salida
int PinSensor_Mag=32;//11                     /sin usar
bool Sensor_Mag=0;               //sin usar
//int boton_input=0;     
//bool estado_input=0;                     //sin usar

bool actual_1,anterior_1,boton_1;   //estado de boton abajo
bool actual_2,anterior_2,boton_2;   //estado de boton arriba  
bool actual_3,anterior_3,boton_3;   //estado de boton derecha
bool actual_4,anterior_4,boton_4;   //estado de boton izquiera
int pos_f=0;                        //posicion de caracter flecha en el menu
int pag_menu=0;                     //posicion de pagina menu
bool cambio_menu=0;
bool entra_menu=0;                       //  Entrada menu lcd
bool entra_MenuAnterior=0;               // bandera clear lcd
int dato_prueba=50;                     // dato de prueba
////////////////////////////////////      String para concatenar
char conca_d1[30], conca_d2[30], conca_d3[30], conca_d4[30], conca_d5[30], conca_d6[30];
char  conca_d7[30], conca_d8[30], conca_d9[30], conca_d10[30], conca_d11[30], conca_d12[30];
char  conca_d13[30], conca_d14[30], conca_d15[30], conca_d16[30], conca_d17[30], conca_d18[30];

char  conca_A1[30],conca_A2[30],conca_A3[30],conca_A4[30],conca_A1_1[30],conca_A1_2[30];
int conca_linea_A1_1=0;
int conca_linea_A1_2=1;

   
int ant_dato_0;
int dato_0=0;                       // pulsacion de boton start=1 o stop=0  (envia)
int dato_1=10;                        //recorrido 0-100 /                   (10-100)
int dato_2=10;                        //velocidad inspiracion 0-100           (envia)
float dato_3=0;                      //tiempo mantencion                    (0.0 - 5.0)
int dato3_1=0;                       //tiempo mantencion*10(entero)          (envia)
int dato_4=10;                        //velocidad expiracion                  (envia)
float dato_5=0;                      //tiempo reposo                        (0.0 - 5.0)
int dato5_1=0;                       // tiempo reposo (entero)
int dato_6=0;                        // sonido alarma         off/on
int dato_7=0;                        // bip apoyo         off/insp/exp/e+i
int dato_8=0;                        // fuerza            si/no
float dato_9=0.5;                        // Rango de fuerza   0.5-10 kg     (0.5 - 10.0)
int dato_10=0;                       // Accion de F     (det- alarm)
int dato_11=0;                       // Presion inspira (si - no)
int dato_12=-1;                       // Umbral detc.    (10 - 40)
int dato_13=0;                      //  Accion Umb.     (comp-solo)
int dato_14=0;                      //  Presion Max      (si - no)
int dato_15=5;                      // Rango Pmax.      (5-40)
int dato_16=0;                      // Accion Pmax      (det-alarma)
int dato_17=0;                      // Boton          (si - no)
int dato_18=0;                      //  accion boton (detencion/ alarma/insp/auto)
bool alarma=0;                      /// dato de alarma 0 apagada 1 encendido
bool EstadoAmarillo=0;
bool AntAlarm=0;

int menu_preceso=0;
int linea=0;
bool mantenido4=0;
bool mantenido3=0;
bool pulso3=0;
bool pulso4=0;
bool beep_Act=0;
bool beep_Alarm=0;
int  Status=0;
int linea_menu=0;
int linea_alarma=0;
int linea_presion=0;
////////////////////////////          variables de motor
int TRANS_dato_1=0;                        //recorrido 0-100 /                   (10-100)
int TRANS_dato_2=0;                        //velocidad inspiracion 0-100           (envia)
int TRANS_dato3_1=0;                       //tiempo mantencion*10(entero)          (envia)
int TRANS_dato_4=0;                        //velocidad expiracion                  (envia)
int TRANS_dato5_1=0;                       // tiempo reposo (entero)

int home1=0;
int bandera_home=0;
int Calibra1=0;
//int velocidad_home_1=400;
int etapa=0;

int estado_sensor;
int detecta_atras=0;


int home_paso=0;
int funciona=0;
int Longitud=0;                 //map()10-100:200-1000
int Velocidad_Inspira=0;   
int Aceleracion_Inspira=0;

int Aceleracion_Expira=0;     //  
int Velocidad_Expira=0;

int acel_fin_adelante=0;
int acel_inicio_delante=0;
int Rampa_adelante=0;

int pasos_margen=300;
int total_pasos=1200;

int Tiempo_inspira=0;
int Tiempo_reposo=0;

int acel_fin_atras=0;
int acel_inicio_atras=0;
int Rampa_atras=0;

int pulsoUP=10;    /// 10 us


//bool bandera_inspira=0; // bandera de entrada de tiempo
//bool bandera_expira=0; // bandera de entrada de tiempo

///////////////////////////////////////           Temporizadores
Chrono luz;               // backlight  lcd
Chrono timeOut;           // salir del menu
Chrono Tiempo_Beep;       // tiempo de sonido beep
Chrono tiempo_manB4;      // tiempo         
Chrono tiempo1_manB4;
Chrono tiempo_manB3;
Chrono tiempo1_manB3;
Chrono time_alarma;       //  tiempo de alarma
Chrono Refresh_menu;      //  tiempo de refresh LCD

Chrono tiempo_1(Chrono::MICROS);
Chrono tiempo_2(Chrono::MICROS);
Chrono Tiempo_espera;


void alerta(){     
   if(alarma!=AntAlarm){                // inicial de alarma
      EstadoAmarillo=1;
      digitalWrite(luz_amarillo,EstadoAmarillo);
      AntAlarm=alarma;
    }                    
  switch (alarma){   //funcion de luz y sonido alternado
    case 0:
      EstadoAmarillo=0;
      digitalWrite(luz_amarillo,EstadoAmarillo);     //apagado de luz
     time_alarma.restart();                      //reset tiempo de luz
     time_alarma.stop();                         //stop tiempo de luz 
    break;
    case 1:
      time_alarma.resume();
      if (time_alarma.hasPassed(250)){
        EstadoAmarillo=!EstadoAmarillo;  
        if((EstadoAmarillo)&&(dato_6)){
          beep_Alarm=1;
        }                          // si es distinto
        digitalWrite(luz_amarillo,EstadoAmarillo);  //  
        dato_prueba=1;
        time_alarma.restart();                    // reset tiempo alarma
      }                    
    break;
  }
}

void beep_respiracion(){
  if((beep_Act)||(beep_Alarm)){
    Tiempo_Beep.resume();
    digitalWrite(buzzer,HIGH);
   
  }
  if (Tiempo_Beep.hasPassed(50)){
    beep_Act=0;
    beep_Alarm=0;
    digitalWrite(buzzer,LOW);
    Tiempo_Beep.restart();
    Tiempo_Beep.stop();
  }


} //fin de void beep

void inicio (){
  if(enciende==0){
    dato_0=EEPROM.readInt(0);
    dato_1=EEPROM.readInt(10);
    dato_2=EEPROM.readInt(20);
    dato_3=EEPROM.readFloat(30);
    dato_4=EEPROM.readInt(40);
    dato_5=EEPROM.readFloat(50);
    dato_6=EEPROM.readInt(60);
    dato_7=EEPROM.readInt(70);
    dato_8=EEPROM.readInt(80);
    dato_9=EEPROM.readFloat(90);
    dato_10=EEPROM.readInt(100);
    dato_11=EEPROM.readInt(110);
    dato_12=EEPROM.readInt(120);
    dato_13=EEPROM.readInt(130);
    dato_14=EEPROM.readInt(140);
    dato_15=EEPROM.readInt(150);
    dato_16=EEPROM.readInt(160);
    dato_17=EEPROM.readInt(170);
    dato_18=EEPROM.readInt(180);

    dato3_1=int(((dato_3)*10)+0.4);
    dato5_1=int(((dato_5)*10)+0.4);

    if(dato_0==1){
        b_inicio=1;     
      }
    enciende=1;
  }
    
}
void Input(){
  estado_boton_start=digitalRead(boton_start);
  estado_boton_stop=digitalRead(boton_stop);
  estado_sensor=digitalRead(PinSensor_Mag);
}
void sensor_celda(){

  
  CELDA=(((O_CELDA.read())-(CELDA_CERO))/97250);   // PRESION 115600
  
  if(dato_8){ // activacioon sensor fuerza
    if((CELDA>=dato_9)&&(dato_0)){
      switch (dato_10)  // detencion o aviso
      {
      case 0:
          alarma=1;
          etapa=0;
          linea_alarma=3;
          stepper.setCurrentPosition(0);
        break;

      case 1:
           alarma=1;
           linea_alarma=3;

        break;
      }
    }
  }
}
 void sensor_presion(){
  PRESION=(((O_PRESION.read())-(PRESION_CERO))/64285); //  (cmH2O o mbar) 
  if(dato_14){
    if(PRESION>=dato_15){
      switch  (dato_16)
      {
        case 0:
          alarma=1;
          etapa=0;
          linea_alarma=4;
          stepper.setCurrentPosition(0);
        break;
        case 1:
          alarma=1;
          linea_alarma=4;
        break;
      }
    }
  }
if(dato_11){
  if(PRESION<=dato_12){
    trigg=1;
  }
  else{
    trigg=0;
  } 
 }
}



void pin_input(){
  estado_input=digitalRead(pin_entrada);
  if((dato_17)&&(estado_input)){
      switch  (dato_18)
      {
        case 0:
          alarma=1;
          etapa=0;
          linea_alarma=5;
          stepper.setCurrentPosition(0);
        break;
        case 1:
          alarma=1;
          linea_alarma=5;
        break;
      }
    
  }
}
void proceso(){
  
  if(((estado_boton_start)||(b_inicio==1)) && (etapa==0)){
     etapa=1;
     b_inicio=0;
     //Tiempo_espera.start();
  }
  if((estado_boton_start) && ((linea_alarma>=3))){   /// reset de alarmas con start
    alarma=0;
    linea_alarma=0;
  }
  if(estado_boton_stop){
    etapa=0;
    alarma=0;
    linea_alarma=0;
    Tiempo_espera.restart();
    Tiempo_espera.stop();
    stepper.setCurrentPosition(0);
    stepper.disableOutputs ();
  }
  if(etapa==0){
    digitalWrite(luz_rojo,1);
    digitalWrite(luz_verde,0);
  }
  if(etapa!=0){
    digitalWrite(luz_rojo,0);
    digitalWrite(luz_verde,1);
  }
  if((home1==12)||home1==32){        

    TRANS_dato_1=dato_1;               //recorrido 10-100 /          
    TRANS_dato_2= dato_2;              //velocidad inspiracion 0-100 
    TRANS_dato3_1=dato3_1;            //tiempo mantencion*10(entero)
    TRANS_dato_4= dato_4;             //velocidad expiracion        
    TRANS_dato5_1=dato5_1;            // tiempo reposo (entero)
    Longitud=map(TRANS_dato_1,10,100,200,1000);      //pasos longitud
    Tiempo_inspira=map(TRANS_dato3_1,0,50,1,5000);      // TIEMPO INSPIRA (ms)
    Tiempo_reposo=map(TRANS_dato5_1,0,50,1,5000);
  }
  
}

void teclado(){
  valor_lcd12=analogRead(Pin_lcd12); //  analogico para boton 1,2
  if((valor_lcd12 > 850)&&(valor_lcd12  <1500)) {actual_2=1;  } else  {actual_2=0;}
  if((valor_lcd12 >1500)&&(valor_lcd12  <3500)) {actual_1=1;  } else  {actual_1=0;}
  valor_lcd34=analogRead(pin_lcd34);
  if((valor_lcd34 > 850)&&(valor_lcd34  <1500)) { actual_4=1; } else  {actual_4=0;}
  if((valor_lcd34 >1500)&&(valor_lcd34  <3500)) { actual_3=1; } else  {actual_3=0;}
  //boton 3 mantenido
  if(actual_3==1){            //codigo de mantencion de boton
    tiempo_manB3.resume();
    tiempo1_manB3.resume();
  }
  if(tiempo_manB3.elapsed()>500){
    mantenido3=1;
  }
  if(actual_3==0){
    tiempo_manB3.restart();
    tiempo_manB3.stop();
    tiempo1_manB3.restart();
    tiempo1_manB3.stop();
    pulso3=0;
    mantenido3=0;
  }
  // boton 3 pulsos
  if(mantenido3==1){
    if(tiempo1_manB3.hasPassed(50)){
      pulso3=1;
      cambio_menu=1;
      tiempo1_manB3.restart();
    }
  }
  //boton 4 mantenido
  if(actual_4==1){
    tiempo_manB4.resume();
    tiempo1_manB4.resume();
  }
  if(tiempo_manB4.elapsed()>500){
    mantenido4=1;
  }
  if(actual_4==0){
    tiempo_manB4.restart();
    tiempo_manB4.stop();
    tiempo1_manB4.restart();
    tiempo1_manB4.stop();
    pulso4=0;
    mantenido4=0;
  }
  if(mantenido4==1){
    if(tiempo1_manB4.hasPassed(50)){
      pulso4=1;
      cambio_menu=1;
      tiempo1_manB4.restart();
    }
  }
  switch (actual_1) { // lach boton 1
    case 0:
      switch (anterior_1)
      {
        case 0: boton_1=LOW;  anterior_1=LOW; break;
        case 1: boton_1=LOW;  anterior_1=LOW; break;
      }
    break;
    case 1:
      switch (anterior_1)
      {
        case 0: boton_1=HIGH; cambio_menu=1;  anterior_1=HIGH;  break;
        case 1: boton_1=LOW;  anterior_1=HIGH;  break;  
      }  
    break;
  }
  switch (actual_2){ // lach boton 2
    case 0:
      switch (anterior_2)
      {
        case 0: boton_2=LOW;  anterior_2=LOW; break;
        case 1: boton_2=LOW;  anterior_2=LOW; break;
      }
    break;
    case 1:
      switch (anterior_2)
      {
        case 0: boton_2=HIGH; cambio_menu=1;  anterior_2=HIGH;  break;
        case 1: boton_2=LOW;  anterior_2=HIGH;  break;  
      }  
    break;
  }
  switch (actual_3){ // lach boton 3
    case 0:
      switch (anterior_3)
      {
        case 0: boton_3=LOW;  anterior_3=LOW; break;
        case 1: boton_3=LOW;  anterior_3=LOW; break;
      }
    break;
    case 1:
      switch (anterior_3)
      {
        case 0: boton_3=HIGH; cambio_menu=1;  anterior_3=HIGH;  break;
        case 1: boton_3=LOW;  anterior_3=HIGH;  break;  
      }  
    break;
  }

  switch (actual_4){  // lach boton 3
    case 0:
      switch (anterior_4)
      {
        case 0: boton_4=LOW;  anterior_4=LOW; break;
        case 1: boton_4=LOW;  anterior_4=LOW; break;
      }
    break;
    case 1:
      switch (anterior_4)
      {
        case 0: boton_4=HIGH; cambio_menu=1;  anterior_4=HIGH;  break;
        case 1: boton_4=LOW;  anterior_4=HIGH;  break;  
      }  
    break;
  }
}

void light(){// tempo luz fondo
  if (actual_1||actual_2||actual_3||actual_4||mantenido3||mantenido4||(linea_alarma!=0)){
    luz.restart();                        
    timeOut.restart();   
    //lcd.setBacklight(1);
    if(light_on==0){                    // una vez
      light_on=1;
      lcd.setBacklight(1);              //ENCENDER LUZ}         
    }
  }
  if (luz.elapsed() > 120000){ //15
    luz.restart();
    luz.stop();
    pos_f=0;
    pag_menu=0;
    lcd.setBacklight(0);
  }
}

void time_Out(){//tempo para salira al proceso solo
  if (timeOut.elapsed() > 3000){
    timeOut.restart();
    timeOut.stop();
    light_on=0;
    entra_menu=0;
    //EEPROM.writeInt(0,dato_0);
    EEPROM.writeInt(10,dato_1);
    EEPROM.writeInt(20,dato_2);
    EEPROM.writeFloat(30,dato_3);
    EEPROM.writeInt(40,dato_4);
    EEPROM.writeFloat(50,dato_5);
    EEPROM.writeInt(60,dato_6);
    EEPROM.writeInt(70,dato_7);
    EEPROM.writeInt(80,dato_8);
    EEPROM.writeFloat(90,dato_9);
    EEPROM.writeInt(100,dato_10);
    EEPROM.writeInt(110,dato_11);
    EEPROM.writeInt(120,dato_12);
    EEPROM.writeInt(130,dato_13);
    EEPROM.writeInt(140,dato_14);
    EEPROM.writeInt(150,dato_15);
    EEPROM.writeInt(160,dato_16);
    EEPROM.writeInt(170,dato_17);
    EEPROM.writeInt(180,dato_18);
    EEPROM.commit();
    
  }
}

void menu_logica (){// desplazamiento flecha abajo
  if(boton_1){   
    switch (pos_f){
     case 0:pos_f=1;boton_1 = LOW;break;
     case 1:pos_f=2;boton_1 = LOW;break;
     case 2:pos_f=3;boton_1 = LOW;break;
     case 3:
       switch (pag_menu)    // desplazamiento de menu abajo
       {
        case 0:   pag_menu =1;  boton_1 = LOW;break;
        case 1:   pag_menu =2;  boton_1 = LOW;break;
        case 2:   pag_menu =3;  boton_1 = LOW;break;
        case 3:   pag_menu =4;  boton_1 = LOW;break;
        case 4:   pag_menu =5;  boton_1 = LOW;break;
        case 5:   pag_menu =6;  boton_1 = LOW;break;
        case 6:   pag_menu =7;  boton_1 = LOW;break;
        case 7:   pag_menu =8;  boton_1 = LOW;break;
        case 8:   pag_menu =9;  boton_1 = LOW;break;
        case 9:   pag_menu =10; boton_1 = LOW;break;
        case 10:  pag_menu =11; boton_1 = LOW;break;
        case 11:  pag_menu =12; boton_1 = LOW;break;
        case 12:  pag_menu =13; boton_1 = LOW;break;
        case 13:  pag_menu =14; boton_1 = LOW;break;
       }
     break;
    }
  }
  
  if(boton_2){        // desplazamiento flecha arriba
    switch (pos_f)
    {
      case 3: pos_f=2; boton_2 = LOW;break;
      case 2: pos_f=1; boton_2 = LOW;break;
      case 1: pos_f=0; boton_2 = LOW;break;
      case 0:       // desplazamiento menu arriba
        switch (pag_menu)
        {
          case 1:   pag_menu =0;  boton_2 = LOW;break;
          case 2:   pag_menu =1;  boton_2 = LOW;break;
          case 3:   pag_menu =2;  boton_2 = LOW;break;
          case 4:   pag_menu =3;  boton_2 = LOW;break;
          case 5:   pag_menu =4;  boton_2 = LOW;break;
          case 6:   pag_menu =5;  boton_2 = LOW;break;
          case 7:   pag_menu =6;  boton_2 = LOW;break;
          case 8:   pag_menu =7;  boton_2 = LOW;break;
          case 9:   pag_menu =8;  boton_2 = LOW;break;
          case 10:  pag_menu =9;  boton_2 = LOW;break;
          case 11:  pag_menu =10; boton_2 = LOW;break;
          case 12:  pag_menu =11; boton_2 = LOW;break;
          case 13:  pag_menu =12; boton_2 = LOW;break;
          case 14:  pag_menu =13; boton_2 = LOW;break;
        }
      break;
    }
  } 

  linea=pag_menu+pos_f;
 
  switch (linea)          // logica linea del menu
  {
    case 0:       ////////////////////////////////          linea 0   Volumen (dato 1)
      if(((boton_4 == HIGH)||(pulso4==1))&&(dato_1<500)){
        dato_1=dato_1+10;
        pulso4=0;
        boton_4=LOW;
      }
      if(((boton_3 == HIGH)||(pulso3==1))&&(dato_1>10)){
        dato_1=dato_1-10;
        pulso3=0;
        boton_3=LOW;
      }
    break;
    ///////////////////////////////////////////           linea 1    Velocidad inspira (dato 2)
    case 1:
      if(((boton_4 == HIGH)||(pulso4==1))&&(dato_2<100)){
        dato_2=dato_2+5;
        pulso4=0;
        boton_4=LOW;
      }
      if(((boton_3 == HIGH)||(pulso3==1))&&(dato_2>10)){
        dato_2=dato_2-5;
        pulso3=0;
        boton_3=LOW;
      }
    break;
    ///////////////////////////////////////////         linea 2     tiempo de mant (dato 3)
    case 2:
      if(((boton_4 == HIGH)||(pulso4==1))&&(dato_3<4.9)){
        dato_3=dato_3+0.1;
        pulso4=0;
        boton_4=LOW;
      }
      if(((boton_3 == HIGH)||(pulso3==1))&&(dato_3>0.1)){
        dato_3=dato_3-0.1;
        pulso3=0;
        boton_3=LOW;
      }
      dato3_1=int(((dato_3)*10)+0.4);   // pasa dato flotante a entero para ser enviado
    break;
    //////////////////////////////////////////          linea 3  velocidad Expira (dato 4)
    case 3:
      if(((boton_4 == HIGH)||(pulso4==1))&&(dato_4<100)){
        dato_4=dato_4+5;
        pulso4=0;
        boton_4=LOW;
      }
      if(((boton_3 == HIGH)||(pulso3==1))&&(dato_4>10)){
        dato_4=dato_4-5;
        pulso3=0;
        boton_3=LOW;
      }

    break;
    //////////////////////////////////////////          linea 4   tiempo reposo (dato 5)
    case 4:
      if(((boton_4 == HIGH)||(pulso4==1))&&(dato_5<=4.9)){
        dato_5=dato_5+0.1;
        pulso4=0;
        boton_4=LOW;
      }
      if(((boton_3 == HIGH)||(pulso3==1))&&(dato_5>0.1)){
        dato_5=dato_5-0.1;
        pulso3=0;
        boton_3=LOW;
      }
      dato5_1=int(((dato_5)*10)+0.4);
    break;
     ///////////////////////////////////////           linea 5   Sonido alarma (dato_6)
    case 5:
      if((boton_4 == HIGH)){
        dato_6=!dato_6;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)){
        dato_6=!dato_6;
        pulso3=0;
        boton_3=LOW;
      }// fin dato7
    break;
    ///////////////////////////////////////         linea 6   bip de apoyo (dato_7)
    case 6:
      if((boton_4 == HIGH)&&(dato_7<3)){
        dato_7++;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)&&(dato_7>0)){
        dato_7--;
        pulso3=0;
        boton_3=LOW;
      }//fin dato 8
    break;
    //////////////////////////////////////          Linea 7   Sensor de fuerza (dato_8)   (si-no)
    case 7:
      if((boton_4 == HIGH)){
        dato_8=!dato_8;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)){
        dato_8=!dato_8;
        pulso3=0;
        boton_3=LOW;
      }// fin dato7
    break;
    //////////////////////////////////////      linea 8   Fuerza Max.   (dato_9)    (0.5-10 kg)
    case 8:
      if(((boton_4 == HIGH)||(pulso4==1))&&(dato_9<=10.0)){
        dato_9=dato_9+0.1;
        pulso4=0;
        boton_4=LOW;
      }
      if(((boton_3 == HIGH)||(pulso3==1))&&(dato_9>=0.6)){
        dato_9=dato_9-0.1;
        pulso3=0;
        boton_3=LOW;
      }
    break;
    //////////////////////////////////////      Linea 9  Accion fuerza max. (dato_10)    (det-aviso)
    case 9:
      if((boton_4 == HIGH)){
        dato_10=!dato_10;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)){
        dato_10=!dato_10;
        pulso3=0;
        boton_3=LOW;
      }// fin dato7
    break;
    /////////////////////////////////////      linea 10  presion trigger  (dato_11) (No-Si) 
    case 10:
      if((boton_4 == HIGH)){
        dato_11=!dato_11;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)){
        dato_11=!dato_11;
        pulso3=0;
        boton_3=LOW;
      }// fin dato7
    break;
     ////////////////////////////////////      Linea 11  Umbral Trigger (dato_12) (-0.5 -15)
    case 11:      
      if(((boton_4 == HIGH)||(pulso4==1))&&(dato_12>-20)){
        dato_12--;
        pulso4=0;
        boton_4=LOW;
      }
      if(((boton_3 == HIGH)||(pulso3==1))&&(dato_12<-1)){
        dato_12++;
        pulso3=0;
        boton_3=LOW;
      }
    break;  
    /////////////////////////////////////      Linea 12 Accion Trigger  (dato_13) ( solo - auto)
    case 12:                 
      if((boton_4 == HIGH)){
        dato_13=!dato_13;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)){
        dato_13=!dato_13;
        pulso3=0;
        boton_3=LOW;
      }// fin dato7
    break;
    /////////////////////////////////////       Linea 13 Presion Max. (dato_14) (No - Si)
    case 13:
      if((boton_4 == HIGH)){
        dato_14=!dato_14;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)){
        dato_14=!dato_14;
        pulso3=0;
        boton_3=LOW;
      }// fin dato7
    break;
    ////////////////////////////////////       linea 14 Rango Presion max (dato_15) (5 - 50 [mb] )
    case 14:       
      if(((boton_4 == HIGH)||(pulso4==1))&&(dato_15<50)){
        dato_15++;
        pulso4=0;
        boton_4=LOW;
      }
      if(((boton_3 == HIGH)||(pulso3==1))&&(dato_15>5)){
        dato_15--;
        pulso3=0;
        boton_3=LOW;
      }
    break;
    ///////////////////////////////////       Linea 15 Accion Presion max (dato_16) (Det - Alarma)
    case 15:
      if((boton_4 == HIGH)){
        dato_16=!dato_16;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)){
        dato_16=!dato_16;
        pulso3=0;
        boton_3=LOW;
      }// fin dato7
    break;
    //////////////////////////////////        Linea 16 Entrada digital (dato_17)  (Si - No)
    case 16:
      if((boton_4 == HIGH)){
        dato_17=!dato_17;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)){
        dato_17=!dato_17;
        pulso3=0;
        boton_3=LOW;
      }// fin dato7
    break;
    //////////////////////////////////        Linea 17 [Accion digital] (dato_18) (Det - Alarma - insp - Auto)
    case 17:
      if((boton_4 == HIGH)&&(dato_18<3)){
        dato_18++;
        pulso4=0;
        boton_4=LOW;
      }
      if((boton_3 == HIGH)&&(dato_18>0)){
        dato_18--;
        pulso3=0;
        boton_3=LOW;
      }//fin dato 8
    break;
  }
}    //////////// fin switch (linea)

void concatena_lcd(){  // variables de union para enviar lcd
  sprintf(conca_d1, " Volumen [mL]  %3d  ",dato_1);                    // linea 0  
  sprintf(conca_d2, " Vel. Inspira  %3d %s",dato_2,"%");                    // Linea 1
  sprintf(conca_d3, " Tiempo mant.  %1.1f %s",dato_3,"s");                  // linea 2
  sprintf(conca_d4, " Vel. Expira   %3d %s",dato_4,"%");                    // linea 3
  sprintf(conca_d5, " T.Reposo      %1.1f %s",dato_5,"s");                  // linea 4
  switch (dato_6){                                    //                    // linea 5
    case 0: sprintf(conca_d6, " Sonido Alar.    %2s ","No");  break;                                               // 
    case 1: sprintf(conca_d6, " Sonido Alar.    %2s ","Si");  break;                                               //
  }   
  switch (dato_7){                                                          // linea 6
    case 0: sprintf(conca_d7, " Bip apoyo       %2s ","No");  break;  
    case 1: sprintf(conca_d7, " Bip apoyo      %4s","Insp"); break;
    case 2: sprintf(conca_d7, " Bip apoyo      %4s","Expi"); break;
    case 3: sprintf(conca_d7, " Bip apoyo      %3s ","E+I"); break;
  }
  switch (dato_8){                                    //                    // linea 7
    case 0: sprintf(conca_d8, " Sen. Fuerza     %2s ","No");  break;                                           
    case 1: sprintf(conca_d8, " Sen. Fuerza     %2s ","Si");  break;                                        
  } 
  if(dato_9<10){sprintf(conca_d9, " Fuerza Max.   %1.1f%2s",dato_9,"kg");}  //linea 8
  else {sprintf(conca_d9, " Fuerza Max.  %2.1f%2s",dato_9,"kg");}

  switch (dato_10){                                    //                   // linea 9
    case 0: sprintf(conca_d10, " Accion F.Max.  %4s","Det.");  break;                                           
    case 1: sprintf(conca_d10, " Accion F.Max. %5s","Alarm");  break;                                        
  } 
  switch (dato_11){                                                         // linea 10 
    case 0: sprintf(conca_d11, " Presion Trig.   %2s ","No");  break;                                           
    case 1: sprintf(conca_d11, " Presion Trig.   %2s ","Si");  break;                                        
  }  
  sprintf(conca_d12, " Umbral Trig. %3d %2s",dato_12,"mb");                // linea 11
  switch (dato_13){                                                         // linea 12
    case 0: sprintf(conca_d13, " Accion Trig.   %4s","Solo");  break;                                           
    case 1: sprintf(conca_d13, " Accion Trig.   %4s","Auto");  break;                                        
  }
  switch (dato_14){                                                         // linea 13
    case 0: sprintf(conca_d14, " Presion Max.    %2s ","No");  break;                                          
    case 1: sprintf(conca_d14, " Presion Max.    %2s ","Si");  break;                                        
  }     
  sprintf(conca_d15, " Rango Pmax.   %2d %2s",dato_15,"mb");                // linea 14
  switch (dato_16){                                                         // linea 15
    case 0: sprintf(conca_d16, " Accion Pmax.   %4s","Det.");  break;                                          
    case 1: sprintf(conca_d16, " Accion Pmax.  %5s","Alarm");  break;                                      
  } 
  switch (dato_17){                                                         // linea 16
    case 0: sprintf(conca_d17, " Entrada Digi.   %2s ","No");  break;                                          
    case 1: sprintf(conca_d17, " Entrada Digi.   %2s ","Si");  break;                                        
  }  
  switch (dato_18){                                                        // linea 17
    case 0: sprintf(conca_d18, " Accion Digi.   %4s","Det.");  break;                                          
    case 1: sprintf(conca_d18, " Accion Digi.  %5s","Alar.");  break;   
    case 2: sprintf(conca_d18, " Accion Digi.  %5s","Insp.");  break;                                          
    case 3: sprintf(conca_d18, " Accion Digi.   %4s","Auto");  break;                                           
  }   
  switch (linea)                                                    //  posicion de flecha sobre escribe
  {
    case 0:
      //sprintf(conca_d1, "%sLongitud      %3d %s",">",dato_1,"%");
      sprintf(conca_d1, "%sVolumen [mL]  %3d  ",">",dato_1);                    // linea 0  
      //sprintf(conca_d1, "%sLongitud      %3d %s",">",dato_1,"%");
    break;
    case 1:
      sprintf(conca_d2, "%sVel. Inspira  %3d %s",">",dato_2,"%");
    break;
    case 2:
      sprintf(conca_d3, "%sTiempo mant.  %1.1f %s",">",dato_3,"s");
    break;
    case 3:
      sprintf(conca_d4, "%sVel. Expira   %3d %s",">",dato_4,"%");
    break;
    case 4:
      sprintf(conca_d5, "%sT.Reposo      %1.1f %s",">",dato_5,"s");
    break;
    case 5:
      switch (dato_6){                                    //                    // linea 5
        case 0: sprintf(conca_d6, "%sSonido Alar.    %2s ",">","No");  break;                                               // 
        case 1: sprintf(conca_d6, "%sSonido Alar.    %2s ",">","Si");  break;                                               //
      } 
    break;
    case 6:
      switch (dato_7){                                                          // linea 6
        case 0: sprintf(conca_d7, "%sBip apoyo       %2s ",">","No");  break;  
        case 1: sprintf(conca_d7, "%sBip apoyo      %4s",">","Insp"); break;
        case 2: sprintf(conca_d7, "%sBip apoyo      %4s",">","Expi"); break;
        case 3: sprintf(conca_d7, "%sBip apoyo      %3s ",">","E+I"); break;
      }
    break;

    case 7:
      switch (dato_8){                                    //                    // linea 7
        case 0: sprintf(conca_d8, "%sSen. Fuerza     %2s ",">","No");  break;                                           
        case 1: sprintf(conca_d8, "%sSen. Fuerza     %2s ",">","Si");  break;                                        
      } 
    break;
    case 8:
      if(dato_9<10){sprintf(conca_d9, "%sFuerza Max.   %1.1f%2s",">",dato_9,"kg");}  //linea 8
      else {sprintf(conca_d9, "%sFuerza Max.  %2.1f%2s",">",dato_9,"kg");}
    break;
    case 9:
      switch (dato_10){                                    //                   // linea 9
        case 0: sprintf(conca_d10, "%sAccion F.Max.  %4s",">","Det.");  break;                                           
        case 1: sprintf(conca_d10, "%sAccion F.Max. %5s",">","Alarm");  break;                                        
      } 
    break;
    case 10:  
      switch (dato_11){                                    //
        case 0: sprintf(conca_d11, "%sPresion Trig.   %2s ",">","No");  break;                                           
        case 1: sprintf(conca_d11, "%sPresion Trig.   %2s ",">","Si");  break;                                        
      }
    break;
    case 11:    
      sprintf(conca_d12, "%sUmbral Trig. %3d %2s",">",dato_12,"mb");  
    break;  
    case 12:
      switch (dato_13){                                    //
        case 0: sprintf(conca_d13, "%sAccion Trig.   %4s",">","Solo");  break;                                           
        case 1: sprintf(conca_d13, "%sAccion Trig.   %4s",">","Auto");  break;                                        
      }
    break;
    case 13:  
      switch (dato_14){                                    //
        case 0: sprintf(conca_d14, "%sPresion Max.    %2s ",">","No");  break;                                          
        case 1: sprintf(conca_d14, "%sPresion Max.    %2s ",">","Si");  break;                                        
      }
    break;
    case 14:       
      sprintf(conca_d15, "%sRango Pmax.   %2d %2s",">",dato_15,"mb"); 
    break;
    case 15:    
      switch (dato_16){                                    //
        case 0: sprintf(conca_d16, "%sAccion Pmax.   %4s",">","Det.");  break;                                          
        case 1: sprintf(conca_d16, "%sAccion Pmax.  %5s",">","Alarm");  break;                                      
      }
    break;
    case 16:   
      switch (dato_17){                                    //
        case 0: sprintf(conca_d17, "%sEntrada Digi.   %2s ",">","No");  break;                                          
        case 1: sprintf(conca_d17, "%sEntrada Digi.   %2s ",">","Si");  break;                                        
      }
    break;
    case 17:    
      switch (dato_18){                                    //
        case 0: sprintf(conca_d18, "%sAccion Digi.   %4s",">","Det.");  break;                                          
        case 1: sprintf(conca_d18, "%sAccion Digi.  %5s",">","Alar.");  break;   
        case 2: sprintf(conca_d18, "%sAccion Digi.  %5s",">","Insp.");  break;                                          
        case 3: sprintf(conca_d18, "%sAccion Digi.   %4s",">","Auto");  break;                                           
      }   
    break;
  }

}
void conca_pricipal(){
  /////////////////////////linea 0
  switch (n_inspira)
  {
    case 1: sprintf(conca_A1_1," TIEMPO  ");  break;
    case 2: sprintf(conca_A1_1," TRIGGER ");  break;
    case 3: sprintf(conca_A1_1," T+TRIG. ");  break;
    case 4: sprintf(conca_A1_1," INPUT   ");  break;
    case 5: sprintf(conca_A1_1," T+INPUT ");  break;
    case 6: sprintf(conca_A1_1,"TRIG.+IN ");  break;
    case 7: sprintf(conca_A1_1,"TRIG+T+IN");  break;
  }

  if((Status==0)&&(linea_alarma==0)){
    conca_linea_A1_2=1;
    dato_0=0;
  }
  if((Status==1)&&(linea_alarma==0)){
    conca_linea_A1_2=2;
  }
  if((Status==2)&&(linea_alarma==0)){
    conca_linea_A1_2=3;
    dato_0=1;
  }
  if((linea_alarma==1)||(linea_alarma==2)){
    conca_linea_A1_2=4;
    dato_0=0;
  }
  if(Status==2){
    if(linea_alarma==3){
      conca_linea_A1_2=5;
    }
    if(linea_alarma==4){
      conca_linea_A1_2=6;
    }
    if(linea_alarma==5){
      conca_linea_A1_2=7;
    }
  }
  if(Status==0){
    if(linea_alarma==3){
      conca_linea_A1_2=8;
      dato_0=0;
    }
    if(linea_alarma==4){
      conca_linea_A1_2=9;
      dato_0=0;
    }
    if(linea_alarma==5){
      conca_linea_A1_2=10;
      dato_0=0;
    }
  }
  if(dato_0!=ant_dato_0){
    ant_dato_0=dato_0;
    EEPROM.writeInt(0,dato_0);
    EEPROM.commit();
    
  }
  switch (conca_linea_A1_2)
  {
     //  Status= 0 detenido; 1 calibrando; 2 funcionando
  //  linea_alarma= 0 sistema ok;1 sensor no det; 2 sen no acciona;3lim fuerza:4 Limitepresion; 5 input 
    case 1:   sprintf(conca_A1_2,"DET. LISTO");  break;
    case 2:   sprintf(conca_A1_2,"CALIBRANDO");  break;
    case 3:   sprintf(conca_A1_2," FUNCIONA ");  break;
    case 4:   sprintf(conca_A1_2,"ERR.SENSOR");  break;
    case 5:   sprintf(conca_A1_2,"ALM. F.MAX");  break;
    case 6:   sprintf(conca_A1_2,"ALM. P.MAX");  break;
    case 7:   sprintf(conca_A1_2,"ALM. INPUT");  break;
    case 8:   sprintf(conca_A1_2,"DET. F.MAX");  break;
    case 9:   sprintf(conca_A1_2,"DET. P.MAX");  break;
    case 10:  sprintf(conca_A1_2,"DET. INPUT");  break;
  }
  sprintf(conca_A1,"%9s %10s",conca_A1_1,conca_A1_2);
  /////////////////////////linea 1
  switch (dato_11)
  {
    case 0:
      sprintf(conca_A2,"%3d=mb %3d Trig= Off",PRESION,dato_12);
    break;
    case 1:
      switch (dato_13)
      {
      case 0:
        sprintf(conca_A2,"%3d=mb %3d Trig=Insp",PRESION,dato_12);
      break;
      case 1:
        sprintf(conca_A2,"%3d=mb %3d Trig=Auto",PRESION,dato_12);
      break;
      }
    break;
  }
  /////////////////////////linea 2
  if(CELDA>=0){       // variables para positivos
    pos_celda=CELDA;    
  }
  switch (dato_14)
  { 
    case 0:
      sprintf(conca_A3,"%1.1f=Kg  %2d Pmax= Off",pos_celda,dato_15);
    break;
    case 1:
      switch (dato_16)
      {
        case 0:
            sprintf(conca_A3,"%1.1f=Kg  %2d Pmax= Det",pos_celda,dato_15);
        break;
        case 1:
            sprintf(conca_A3,"%1.1f=Kg  %2d Pmax=Alar",pos_celda,dato_15);
        break;
      }
    break; 
  }
  /////////////////////////linea 3
  switch (dato_8)
  {
    case 0:
      sprintf(conca_A4,"%3d=mL %1.1f Fmax= Off",dato_1,dato_9);
    break;
    case 1:
      switch (dato_10)
      {
        case 0:
          sprintf(conca_A4,"%3d=mL %1.1f Fmax=Det.",dato_1,dato_9);
        break;
        case 1:
          sprintf(conca_A4,"%3d=mL %1.1f Fmax=Alar",dato_1,dato_9);
        break;
      }
    break;
  }
  
}
void pantallaPrincipal (){

    lcd.setCursor(0,0); lcd.print(conca_A1);
    lcd.setCursor(0,1); lcd.print(conca_A2);
    lcd.setCursor(0,2); lcd.print(conca_A3);
    lcd.setCursor(0,3); lcd.print(conca_A4);
  }
void menu_texto(){//lcd print
  switch (pag_menu){   ///////////////////////  Pagina del menu               
    case 0: //menu 1
      lcd.setCursor(0,0); lcd.print(conca_d1);
      lcd.setCursor(0,1); lcd.print(conca_d2);
      lcd.setCursor(0,2); lcd.print(conca_d3);
      lcd.setCursor(0,3); lcd.print(conca_d4);
    break;
    case 1: //menu2
      lcd.setCursor(0,0); lcd.print(conca_d2);
      lcd.setCursor(0,1); lcd.print(conca_d3);
      lcd.setCursor(0,2); lcd.print(conca_d4);
      lcd.setCursor(0,3); lcd.print(conca_d5);
    break; 
    case 2: 
     lcd.setCursor(0,0); lcd.print(conca_d3);
     lcd.setCursor(0,1); lcd.print(conca_d4);
     lcd.setCursor(0,2); lcd.print(conca_d5);
     lcd.setCursor(0,3); lcd.print(conca_d6);
    break;                        
    case 3: 
      lcd.setCursor(0,0); lcd.print(conca_d4);
      lcd.setCursor(0,1); lcd.print(conca_d5);
      lcd.setCursor(0,2); lcd.print(conca_d6);
      lcd.setCursor(0,3); lcd.print(conca_d7);
    break;                        
    case 4: 
      lcd.setCursor(0,0); lcd.print(conca_d5);
      lcd.setCursor(0,1); lcd.print(conca_d6);
      lcd.setCursor(0,2); lcd.print(conca_d7);
      lcd.setCursor(0,3); lcd.print(conca_d8);  
    break;                        
    case 5: 
      lcd.setCursor(0,0); lcd.print(conca_d6);
      lcd.setCursor(0,1); lcd.print(conca_d7);
      lcd.setCursor(0,2); lcd.print(conca_d8);
      lcd.setCursor(0,3); lcd.print(conca_d9);  
    break;    
    case 6: 
      lcd.setCursor(0,0); lcd.print(conca_d7);
      lcd.setCursor(0,1); lcd.print(conca_d8);
      lcd.setCursor(0,2); lcd.print(conca_d9);
      lcd.setCursor(0,3); lcd.print(conca_d10);  
    break;                                            
    case 7: 
      lcd.setCursor(0,0); lcd.print(conca_d8);
      lcd.setCursor(0,1); lcd.print(conca_d9);
      lcd.setCursor(0,2); lcd.print(conca_d10);
      lcd.setCursor(0,3); lcd.print(conca_d11);  
    break;
    case 8: 
      lcd.setCursor(0,0); lcd.print(conca_d9);
      lcd.setCursor(0,1); lcd.print(conca_d10);
      lcd.setCursor(0,2); lcd.print(conca_d11);
      lcd.setCursor(0,3); lcd.print(conca_d12);  
    break;
    case 9: 
      lcd.setCursor(0,0); lcd.print(conca_d10);
      lcd.setCursor(0,1); lcd.print(conca_d11);
      lcd.setCursor(0,2); lcd.print(conca_d12);
      lcd.setCursor(0,3); lcd.print(conca_d13);  
    break;
    case 10: 
      lcd.setCursor(0,0); lcd.print(conca_d11);
      lcd.setCursor(0,1); lcd.print(conca_d12);
      lcd.setCursor(0,2); lcd.print(conca_d13);
      lcd.setCursor(0,3); lcd.print(conca_d14);  
    break;
    case 11: 
      lcd.setCursor(0,0); lcd.print(conca_d12);
      lcd.setCursor(0,1); lcd.print(conca_d13);
      lcd.setCursor(0,2); lcd.print(conca_d14);
      lcd.setCursor(0,3); lcd.print(conca_d15);  
    break;
    case 12: 
      lcd.setCursor(0,0); lcd.print(conca_d13);
      lcd.setCursor(0,1); lcd.print(conca_d14);
      lcd.setCursor(0,2); lcd.print(conca_d15);
      lcd.setCursor(0,3); lcd.print(conca_d16);  
    break;
    case 13: 
      lcd.setCursor(0,0); lcd.print(conca_d14);
      lcd.setCursor(0,1); lcd.print(conca_d15);
      lcd.setCursor(0,2); lcd.print(conca_d16);
      lcd.setCursor(0,3); lcd.print(conca_d17);  
    break;
    case 14: 
      lcd.setCursor(0,0); lcd.print(conca_d15);
      lcd.setCursor(0,1); lcd.print(conca_d16);
      lcd.setCursor(0,2); lcd.print(conca_d17);
      lcd.setCursor(0,3); lcd.print(conca_d18);  
    break;
  }
  cambio_menu=0; 
}
TaskHandle_t task2;
  void loop2(void *parameter){
    while(1){
      //teclado();
      time_Out();
      light();
      alerta();
      beep_respiracion();
      sensor_celda();
      sensor_presion();
      pin_input();
    if(entra_menu){
      //concatena_lcd();
    }
    //concatena_lcd();
    if (actual_1||actual_2||actual_3||actual_4){
      entra_menu=1;
    } 
    if(entra_menu!=entra_MenuAnterior){    // clear al ser distinto
       lcd.clear();
       pos_f=0;
       entra_MenuAnterior=entra_menu;
    }
    if(Refresh_menu.hasPassed(10)){
      switch (entra_menu)
      {
        case 0:  
          conca_pricipal();
          pantallaPrincipal(); 
         break;

        case 1:  //lcd.setBacklight(1);//ENCENDER LUZ
          concatena_lcd();
          menu_texto(); 
        break;
      }
      Refresh_menu.restart();
      //delay(200); // resfresco de 
    }
  }
  vTaskDelay(1000); //advertencia watchdog
}
    

void setup(){
  if (!EEPROM.begin(1000)) {
      delay(1000);
    ESP.restart();
  }
    dato_0=EEPROM.readInt(0);
   //Serial.begin(115200);
   stepper.setEnablePin(33);   // definir pin enable stepper
  stepper.setPinsInverted(0,0,1);  /// inverterd (direction, step, enable) 
  stepper.setMaxSpeed(4200); ///steps/second 3320 a 500 rpm trabajo / 6640 =1000rpm max 
  stepper.setAcceleration(2000); /// step/(second)^2 14000
  stepper.disableOutputs ();

  delay(10);
  O_PRESION.begin(PRESION_DOUT_PIN, PRESION_SCK_PIN);
  O_CELDA.begin(CELDA_DOUT_PIN, CELDA_SCK_PIN);  

  for (int i = 1; i <= 5; i++) {
    P_SUMA=P_SUMA+(O_PRESION.read());
    delay(100);
  }
  for (int i = 1; i <= 5; i++) {
    C_SUMA=C_SUMA+(O_CELDA.read());
    delay(100);
  }
  PRESION_CERO=P_SUMA/5;
  CELDA_CERO=C_SUMA/5;

  Wire.begin();

  lcd.begin(20,4);
  lcd.clear();

  pinMode(boton_start,INPUT);
  pinMode(boton_stop,INPUT);
  pinMode(luz_rojo,OUTPUT);
  pinMode(luz_verde,OUTPUT); 
  pinMode(luz_amarillo,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(PinSensor_Mag,INPUT);
  pinMode(pin_entrada,INPUT);
  
  luz.restart();
  //luz.stop();
  lcd.setBacklight(1);
  timeOut.restart();
  timeOut.stop();
  tiempo_manB4.restart();
  tiempo_manB4.stop();
  tiempo1_manB4.restart();
  tiempo1_manB4.stop();
  tiempo_manB3.restart();
  tiempo_manB3.stop();
  tiempo1_manB3.restart();
  tiempo1_manB3.stop();
  
  time_alarma.restart();
  time_alarma.stop();
  Tiempo_espera.restart();
  Tiempo_espera.stop();
  Tiempo_Beep.restart();
  Tiempo_Beep.stop();


  Refresh_menu.restart();


  xTaskCreatePinnedToCore(
   loop2, /*funcion de la tarea*/
   "task_2", /*Nombre de la tarea*/
   10000, /*Tamaño de la pila*/
   NULL, /*Parametros de entrada*/
   5, /*Prioridad de tarea*/
   &task2, /*objeto TaskHandle_t*/
   0); /*Nuclo donde se c*/
  
  delay(500);


}

void loop(){
  inicio();
  Input(); 
  proceso();
  teclado();
  menu_logica(); 

  switch (etapa){
    case 0:

        stepper.setCurrentPosition(0);
        stepper.disableOutputs ();
        Status=0;
    break;
    case 1:
        alarma=1;
        Status=1;
        Tiempo_espera.restart();
        etapa=2;
    break;
    case 2:  // tiempo de espera para iniciar calibracion
      if(Tiempo_espera.hasPassed(1000)){
        Tiempo_espera.restart();
        Tiempo_espera.stop();
        etapa=3;
      }
    break;

    case 3:
        stepper.enableOutputs();
        stepper.setAcceleration(2000);
        stepper.setMaxSpeed(4200);
        stepper.moveTo(100);
       // 
        etapa=4;
    break;

    case 4:
        if(stepper.currentPosition() >=100 ){ //no encontro sensor ida
        stepper.setCurrentPosition(0);
        Tiempo_espera.start();
        etapa=5;
        }
    break;
    case 5:     /// tiempo de espera para buscar home hacia atras
      if(Tiempo_espera.hasPassed(200)){
        Tiempo_espera.restart();
        Tiempo_espera.stop();
        etapa=6;
      }
    break;

    case 6:

       //stepper.enableOutputs();
        stepper.setMaxSpeed(1000);   // velocidad  hacia atras 
        stepper.moveTo(-1000);
        etapa=7;
    break;

    case 7:
    if(stepper.currentPosition() <= -1000 ){   // no encontro sensor vuelta
      stepper.setCurrentPosition(0);
      etapa=0;
      linea_alarma=1;
    }
    break;
    case 8:
      //stepper.stop();
      stepper.setMaxSpeed(200);
      stepper.moveTo(-150);
      etapa=9;
    break;


    case 9:
      if(stepper.currentPosition() == (-150) ){ // sensor no acciona
        stepper.setCurrentPosition(0);
        etapa=0;
        linea_alarma=2;
      }
      if(estado_sensor==0){                         //posicion home
        stepper.setCurrentPosition(0);
        Tiempo_espera.restart();
        etapa=10;
      }
    break;

    case 10:
    if(Tiempo_espera.hasPassed(1000)){           // tiempo de espera despues de calibracion 
        Tiempo_espera.restart();
        Tiempo_espera.stop();
        alarma=0;
        //stepper.disableOutputs();
        etapa=11;
        Status=2;
      }
    break;
    case 11:                    //// calculos iniciales
    
    TRANS_dato_1=dato_1;               //recorrido 10-100 /          
    TRANS_dato_2= dato_2;              //velocidad inspiracion 0-100 
    TRANS_dato3_1=dato3_1;            //tiempo mantencion*10(entero)
    TRANS_dato_4= dato_4;             //velocidad expiracion        
    TRANS_dato5_1=dato5_1;            // tiempo reposo (entero)

    Longitud=map(TRANS_dato_1,10,500,200,1000);
    
    Velocidad_Inspira=map(TRANS_dato_2,10,100,1500,4000);
    Aceleracion_Inspira=map(Velocidad_Inspira,1500,4000,1500,18000);
    
    Tiempo_inspira=map(TRANS_dato3_1,0,50,1,5000);      // TIEMPO INSPIRA (ms)
   
    Velocidad_Expira=map(TRANS_dato_4,10,100,1500,4000);
    Aceleracion_Expira=map(Velocidad_Expira,1500,4000,1500,18000);

    Tiempo_reposo=map(TRANS_dato5_1,0,50,1,5000);
    //CELDA_CERO=(O_CELDA.read());
    etapa=12;
    break;
    case 12:                            ///        Movimiento INSPIRACION
    stepper.setMaxSpeed(Velocidad_Inspira);
    stepper.moveTo(Longitud);
    stepper.setAcceleration(Aceleracion_Inspira);
    etapa=13;
    break;
    case 13:
      if(stepper.currentPosition() == Longitud ){
        stepper.setCurrentPosition(0);
        Tiempo_espera.start();
        etapa=14;
      } //no encontro sensor ida
    break;
    case 14:
    if(Tiempo_espera.hasPassed(Tiempo_inspira)){        // Tiempo INSPIRACION
        Tiempo_espera.restart();
        Tiempo_espera.stop();
       // stepper.disableOutputs ();
        etapa=15;
      }
    break;
    case 15:

    stepper.setMaxSpeed(Velocidad_Expira);
    stepper.moveTo(-Longitud +30);                    // margen sensor
    stepper.setAcceleration(Aceleracion_Expira);
    etapa=16;

    break;
    case 16:
    if(stepper.currentPosition() == (-Longitud +30) ){ // sensor no encontrado
      stepper.setCurrentPosition(0);
      etapa=0;
      alarma=1;
      linea_alarma=1;

    }
    if(estado_sensor){
      //stepper.setCurrentPosition(0);
      stepper.stop();
      stepper.setMaxSpeed(500);
      stepper.moveTo(-Longitud-50);
      etapa=17;  
        
    }
    break;
    case 17:
      if(stepper.currentPosition() == (-Longitud-50) ){ // sensor no acciona
        stepper.setCurrentPosition(0);
        etapa=0;
        alarma=1;
        
        linea_alarma=2;
      }
     if(estado_sensor==0){                         //posicion home Expiracion
        stepper.setCurrentPosition(0);
        Tiempo_espera.start();
        etapa=18;
      }
    break;
    case 18:
      switch (n_inspira)
      {
        case 1: 
            if(Tiempo_espera.hasPassed(Tiempo_reposo)){        // Tiempo INSPIRACION
            Tiempo_espera.restart();
            Tiempo_espera.stop();
            etapa=11;
          }
        break;
        case 2:
          if(trigg==HIGH){
            Tiempo_espera.restart();
            Tiempo_espera.stop();
            //trigg=0;
            etapa=11;
          }
        break;
        case 3:
          if((Tiempo_espera.hasPassed(Tiempo_reposo))||(trigg)){        // Tiempo INSPIRACION
            Tiempo_espera.restart();
            Tiempo_espera.stop();
            //trigg=0;
            etapa=11;
          }

        break;
        case 4:
          if(estado_input){
             Tiempo_espera.restart();
            Tiempo_espera.stop();
            //trigg=0;
            etapa=11;
          }
        break;
        case 5:
          if((Tiempo_espera.hasPassed(Tiempo_reposo))||(estado_input)){        // Tiempo INSPIRACION
            Tiempo_espera.restart();
            Tiempo_espera.stop();
            //trigg=0;
            etapa=11;
          }
        break;
        case 6:
          if((trigg)||(estado_input)){
            Tiempo_espera.restart();
            Tiempo_espera.stop();
            //trigg=0;
            etapa=11;
          }
        break;
        case 7:
          if((Tiempo_espera.hasPassed(Tiempo_reposo))||(estado_input)||(trigg)){        // Tiempo INSPIRACION
            Tiempo_espera.restart();
            Tiempo_espera.stop();
            //trigg=0;
            etapa=11;
          }
        break;
      }
    break;
  }  

  //////////////  boton
  if(dato_17){
    if(dato_18>=2){
      insp_boton=1;
    }
    else{insp_boton=0;}
  }
  else{insp_boton=0;}
  ////////////    presion
  if(dato_11){
    if(dato_13>=0){
      insp_presion=1;
    }
    else{insp_presion=0;}
    ///insp_presion=0;
  }
  else{insp_presion=0;}
  ///////////    tiempo
  if((insp_presion==0)&&(insp_boton==0)){
    insp_tiempo=1;
  }
  else if((insp_presion)&&(dato_13)){
    insp_tiempo=1;
  }
  else if((insp_boton)&&(dato_18==3)){
    insp_tiempo=1;
  }
  else{
    insp_tiempo=0;
  }
  n_inspira=insp_boton*4+insp_presion*2+insp_tiempo; 

  if(estado_sensor && (etapa==4 ||etapa==7) ){
    stepper.setCurrentPosition(0);
    etapa=8;
  }
  if((etapa==12)&&((dato_7==1)||(dato_7==3))){
    beep_Act=1;
  }
  if((etapa==15)&&((dato_7==2)||(dato_7==3))){
    beep_Act=1;
  }
  stepper.run(); // funcion de motor
 
}
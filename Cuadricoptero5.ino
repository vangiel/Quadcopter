#ifndef CUADRICOPTERO
#define CUADRICOPTERO

#include <avr/io.h>
#include <avr/interrupt.h>

#include <Wire.h>
#include <Servo.h>

#include "control.h"
#include "motores.h"
#include "pk.h" 

#define PIN_TEST 13

control tControl;
motor tMotor;

volatile bool leer;

ISR(TIMER0_COMPB_vect){
  static int flag = 0 ;
  if( flag ) leer=true;
  TCNT0=0x00;
  flag = ! flag ;
}

void setup() 
{
  Serial.begin(9600);
  tMotor.ini();
  delay(timIni);
  tControl.sensorIni();
  tControl.interruptIni();
  pinMode( PIN_TEST , OUTPUT ) ;
  digitalWrite( PIN_TEST , LOW ) ;
}

void loop() 
{  
  if( pk.estado == tPaquete::LLENO ) {
      switch( pk.datos[1] ) {
        case 6 :
          float f ;
          if( 1 == miscanf( (const char *)&pk.datos[2] , f ) ) {
            pk_limite = f ;
            Serial.print( "limite " ) ; Serial.println( f ) ;
          } else 
            Serial.println( "Error con 'limite'" ) ;
          if( 1 == miscanf( (const char *)&pk.datos[2+8] , f ) ) {
            pk_kp = f ;
            Serial.print( "kp " ) ; Serial.println( f ) ;
          } else 
            Serial.println( "Error con 'kp'" ) ;
          if( 1 == miscanf( (const char *)&pk.datos[2+8*2] , f ) ) {
            pk_ki = f ;
            Serial.print( "ki " ) ; Serial.println( f ) ;
          } else 
            Serial.println( "Error con 'ki'" ) ;
          if( 1 == miscanf( (const char *)&pk.datos[2+8*3] , f ) ) {
            pk_kd = f ;
            Serial.print( "kd " ) ; Serial.println( f ) ;
          } else 
            Serial.println( "Error con 'kd'" ) ;
          
        default :
          pk.estado = tPaquete::VACIO ;
      }
  }
   
  if(leer)
  {
    //digitalWrite( PIN_TEST , HIGH ) ;
    tControl.leerVals(); 
    tControl.calcAng();
    tMotor.actuador();
    tMotor.actuMotors();
    leer=false; 
    //digitalWrite( PIN_TEST , LOW ) ;
  }
}
#endif


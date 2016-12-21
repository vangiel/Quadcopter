#include "motores.h"
#include "control.h"
#include "pk.h"
#include <Arduino.h>



Servo tServo[numMotors];
PID tPID[variables]={PID(1.27,0.19,0.02,32.0),PID(1.62,0.0,2.68,15.0),PID(1.62,0.003,1.12,950.0),PID(0.82,0.21,0.7,25.0)};
long int respuesta[numMotors];

void motor::ini()
{
	tServo[MotorDelante].attach(PinMotor1);
	tServo[MotorDerecha].attach(PinMotor2);
	tServo[MotorAtras].attach(PinMotor3);
	tServo[MotorIzquierda].attach(PinMotor4);
        for(int i=0;i<numMotors;i++)
          tServo[i].writeMicroseconds(minPot);
        
}
void motor::calibrar()
{
	int i;
	for(i=0;i<numMotors;i++)
		tServo[i].writeMicroseconds(maxPot);
	delay(timCalibr);
	for(i=0;i<numMotors;i++)
		tServo[i].writeMicroseconds(minPot);
}
void motor::actuador()
{	
	int i;
	float resultados[variables],ref;
        ref=(5.0-float(analogRead(A0))*0.03);

	for(i=0;i<variables;i++)
        {
	    resultados[i]=tPID[i].PIDcalc(ref, angulo [i],i+4);
        }
        
        #define potBase 1450
        
	respuesta[MotorDelante]=potBase+resultados[PITCH]; 
	respuesta[MotorAtras]=potBase-resultados[PITCH];  
	/*respuesta[MotorDerecha]=1250+resultados[ROLL];
	respuesta[MotorIzquierda]=1250-resultados[ROLL];*/

        for(i=0;i<numMotors;i++)
        {
          if(respuesta[i]>maxPot)
            respuesta[i]=maxPot;
          else if(respuesta[i]<minPot)
            respuesta[i]=minPot;
        }
        /*tPID[PITCH].Kp= pk_kp;
        tPID[PITCH].Ki= pk_ki;
        tPID[PITCH].Kd= pk_kd;*/
        //tPID[PITCH].Kd=float(analogRead(A0))*0.003;
        //Serial.print(respuesta[MotorDelante]);Serial.print(" | ");
        //Serial.print(respuesta[MotorAtras]);Serial.print(" | ");
        //Serial.print(resultados[PITCH]);Serial.print(" | ");
        //Serial.print(tPID[PITCH].error);Serial.print(" | ");
        //Serial.println(tPID[PITCH].diff);//Serial.print(" | ");
        //Serial.println(angulo[PITCH]);//Serial.print(" | ");
        //Serial.print("Ki=  "); Serial.println(tPID[PITCH].Kd);
}
void motor::actuMotors()
{
      tServo[MotorDelante].writeMicroseconds((int)respuesta[MotorDelante]);
      //tServo[MotorDerecha].writeMicroseconds((int)respuesta[MotorDerecha]);
      tServo[MotorAtras].writeMicroseconds((int)respuesta[MotorAtras]);
      //tServo[MotorIzquierda].writeMicroseconds((int)respuesta[MotorIzquierda]);

}



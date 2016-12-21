////////////////LIBRERIAS///////////////////
#include "control.h"
#include <Wire.h>
#include <Arduino.h>

int16_t sensorValsRaw[numSensors];
float angulo [variables];
double sensorAng[numSensors];

/////////Definiciones de la clase control//////////

void control::sensorIni()
{
  Wire.begin();
  Wire.beginTransmission(MPUadress);
  Wire.write(0x6B);  // registro PWR_MGMT_1 
  Wire.write(0);     // se ponbe a 0 para "despertar" el sensor
  Wire.endTransmission(true);
  Wire.beginTransmission(MPUadress);
  Wire.write(0x1A);  // registro CONFIG 
  Wire.write(0x05);     // Distintos valores para los filtros
  Wire.endTransmission(true);
  /*Wire.beginTransmission(MPUadress);
  Wire.write(0x1B);  // registro GYRO_CONFIG 
  Wire.write(0x08);     // Sensibilidad 500 º/s
  Wire.endTransmission(true);
  Wire.beginTransmission(MPUadress);
  Wire.write(0x1C);  // registro ACEL_CONFIG 
  Wire.write(0x08);     // Sensibilidad +- 4g
  Wire.endTransmission(true);*/
  for(int i=0;i<variables;i++)
    angulo[i]=0;
}

void control::interruptIni()
{
  cli();
  TCCR0A=0x00;
  TCCR0B=0x00;
  TCCR0A=(1<<COM0B0)|(1<<WGM01);
  TCCR0B=(1<<CS02)|(1<<CS00);
  TIMSK0=(1<<OCIE0B);
  OCR0B=156; //10 ms
  //OCR0B=0x9C;
  //OCR0B=0x40;
  //OCR0B=0x2E;
  sei();
}

void control::leerVals()
{
  int i;
  Wire.beginTransmission(MPUadress);
  Wire.write(0x3B);  // Empezando por el registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPUadress, numSensors * 2, true); // pide un total de 14 registros
  for (i = 0; i < numSensors && Wire.available(); i++)
      sensorValsRaw[i] = ((((int)Wire.read()) << 8) & 0xFF00) | (((int)Wire.read()) & 0x0FF);
}

void control::calcAng()
{
	//Angulos del acelerometro
	sensorAng[AcX]=RAD_TO_DEG*(atan2((double)sensorValsRaw[AcX],(double)sensorValsRaw[AcZ]));
	sensorAng[AcY]=RAD_TO_DEG*(atan2((double)sensorValsRaw[AcY],(double)sensorValsRaw[AcZ]));
        sensorAng[AcZ]=sensorValsRaw[AcZ]/SCA_FAC_AC;
	
	//Angulos del giroscopio
	sensorAng[GiX]=(sensorValsRaw[GiX]+433)/SCA_FAC_GY;
	sensorAng[GiY]=(sensorValsRaw[GiY]-61)/SCA_FAC_GY;
	sensorAng[GiZ]=(sensorValsRaw[GiZ]-200)/SCA_FAC_GY;
        
        #define DELTA_T 0.02
        #define FCA 0.92
        #define FCB (1-FCA)
        
	//Aplicación filtro complementario
	angulo[PITCH]=FCA*(angulo[PITCH]+sensorAng[GiX]*DELTA_T)+FCB*sensorAng[AcY];
	angulo[ROLL]=FCA*(angulo[ROLL]+sensorAng[GiY]*DELTA_T)+FCB*sensorAng[AcX];
	angulo[YAW]=sensorAng[GiZ];
        angulo[Z]=sensorAng[AcZ];
	
}


////////Definiciones de la clase PID/////////////

float PID::PIDcalc(double ref , double sensorVal,int Giro)
{
  error = ref - sensorVal;
  /*if(!(abs(error)<umbralMov))
    iSum += error*DELTA_T;*/

  if (iSum > windup)
  {
    iSum = windup;
  }
  else if (iSum < - windup)
  {
    iSum = - windup;
  }
  diff= sensorAng[Giro];
  //diff=sensorVal - sensorValOld;
  result = (Kp * error) + (Ki * iSum) + (Kd * diff);
  sensorValOld = sensorVal;
  return result;
}

//CONSTRUCTORES

PID::PID(float Kp0, float Ki0, float Kd0, int windup0)
{
  Kp=Kp0;
  Ki=Ki0;
  Kd=Kd0;
  windup=windup0;
}




#ifndef CONTROL_H
#define CONTROL_H

#include "motores.h"
////////////////VALORES SENSORES////////////////

//Valores maximos y minimos de los sensores

#define sensorMAX 32768 //15 bits
#define sensorMIN -32768

#define pitchMAX 60.00
#define pitchMIN -60.00
#define rollMAX 60.00
#define rollMIN -60.00
#define yawMAX 200.00
#define yawMIN -200.00
#define zMAX 18.00
#define zMIN 0.00

//Factores de escala

#define SCA_FAC_AC 16384.0
//#define SCA_FAC_GY 65.5
#define SCA_FAC_GY 131.0

//Umbral de detección de movimiento

#define umbralMov 2

//Otors parametros

#define MPUadress 0x68
#define g 9.81
#define numSensors 7
#define variables 4
#define TIM_MUESTREO 0x9C 
#define RAD_TO_DEG 180.00/pi
#define DEG_TO_RAD pi/180.00

#define AcX 0
#define AcY 1
#define AcZ 2
#define Tmp 3
#define GiX 4
#define GiY 5
#define GiZ 6

#define PITCH 0
#define ROLL 1 
#define YAW 2
#define Z 3



//////////////VARIABLES GLOBALES////////////

extern  int16_t sensorValsRaw[numSensors];
extern  float angulo [variables];
extern double sensorAng[numSensors];
const float pi = 3.1416;
//////////////CLASES/////////////////////

class control {
  public:
    void sensorIni(); //Inicializa la comunicación I2C y despierta el sensor
    void interruptIni();  //Configura el manejador de la interrupción del temporizador 0
    void leerVals();  //Lee los valores de todos los sensores y los guarda en sensorValsRaw
    void calcAng();   //Cacula los angulos de euler aplicando filtro complementario
};

class PID {
  public:
    float Kp,Ki,Kd,windup,sensorValOld, result, error, iSum, diff;
  public:
    PID(float Kp0, float Ki0, float Kd0, int windup0);
    float PIDcalc(double ref , double sensorVal, int Giro); 
};

#endif

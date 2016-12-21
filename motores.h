#ifndef MOTORES_H
#define MOTORES_H
#include <Servo.h>

#define numMotors 4

#define MotorDelante 0
#define MotorDerecha 1
#define MotorIzquierda 3
#define MotorAtras 2

#define minPot 1000 //microsegundos
#define maxPot 2000 

#define PinMotor1 3
#define PinMotor2 6
#define PinMotor3 9
#define PinMotor4 11

//////////TIEMPOS///////////

#define timCalibr 3000
#define timIni 10000

extern Servo tServo[numMotors];
extern long int respuesta[numMotors];

class motor {
  private:
  public:
    void ini();
    void calibrar();
    void actuador(); //Transforma los valores del sensor a valores del motor
    void actuMotors(); //Actualiza la potencia dada a los motores
};

#endif

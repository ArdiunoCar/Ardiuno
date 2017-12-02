#ifndef _PID_H
#define _PID_H
#include <Arduino.h>
struct MyPID
{
  float Kp;
  float Kd;
  float Ki;
  float PIDOut;
};

extern MyPID AnglePID;
extern MyPID SpeedPID;

void PID_init();
void PID_CalAngle(float _Angle, float _Gyro_gx);
void PID_CalSpeed(float Speed, float* _Position , int run_back);


#endif

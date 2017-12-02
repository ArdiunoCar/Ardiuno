#ifndef _PWM_H
#define _PWM_H

#include <Arduino.h>
//TB6612FNG驱动模块  
#define LeftPWM 9 
#define RightPWM 10
#define AIN1 7
#define AIN2 6
#define BIN1 13
#define BIN2 12
/*
#define AIN1 6    
#define AIN2 7
#define BIN1 12
#define BIN2 13
*/
void PWM_init();
void PWM_Cal(float _Angle, float _AngleOut, float _SpeedOut, float _TurnOut,float* _LeftPWMnum, float* _RightPWMnum, float* _Position);
void PWM_SetMotor(float _LeftPWMnum, float _RightPWMnum);

#endif

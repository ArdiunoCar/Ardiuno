#ifndef _TIMER_H
#define _TIMER_H
#include <MsTimer2.h>
#include "PID.h"
#include "MyMPU6050.h"
#include "Interrupt.h"
#include "PWM.h"
#include "Filter.h"
#include "Control.h"
//extern int Count40ms;
//extern int Count50ms;
extern bool Count1s;
extern float Speed  ; //经过一阶低通滤波后得到的最终速度


void Timer_init();
void Speed_Cal(int _LeftPulseCount, int _RightPulseCount, float _LeftPWMnum, float _RightPWMnum, float* _LeftSpeed, float* _RightSpeed );

#endif

#ifndef _CONTROL_H
#define _CONTROL_H
#include "Serial.h"
#include "SR04.h"
 
/*小车运行状态枚举*/
enum {
  STOP = 0,
  RUN,
  BACK,
  LEFT,
  RIGHT,
  SpinLEFT,
  SpinRIGHT
} State; //控制小车运动的指令
extern int CarState ;
extern int run_back ;
extern int TurnOut ;
void Control_Wireless();
void Control_State();
#endif

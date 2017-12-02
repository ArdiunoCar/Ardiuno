#ifndef _INTERRUPT_H
#define _INTERRUPT_H
#include <Arduino.h>
/**********************管脚定义*************************/

//霍尔编码器模块
#define LeftEncoderA  2   //编码器左轮A相
#define RightEncoderA 3  //编码器右轮A相

/**********************管脚定义*************************/

extern volatile int LeftPulseCount ; //左轮A相脉冲计数
extern volatile int RightPulseCount;//右轮A相脉冲计数

void Interrupt_init();

#endif

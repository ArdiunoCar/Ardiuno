#ifndef _SR04_H
#define _SR04_H

#include <Arduino.h>
#define Trig1 A0 
#define Echo1 A1
#define Trig2 A2
#define Echo2 A3
extern float distance1;
extern float distance2;
void SR04_init();
void SR04_GetDistance();
#endif

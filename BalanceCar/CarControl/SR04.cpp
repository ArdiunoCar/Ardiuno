#include "SR04.h"

float distance1=0;
float distance2=0;

void SR04_init()
{ 
  pinMode(Trig1, OUTPUT); 
  pinMode(Echo1, INPUT); 
  pinMode(Trig2, OUTPUT); 
  pinMode(Echo2, INPUT); 
} 

void SR04_GetDistance() 
{ 
  //发一个10ms的高脉冲去触发TrigPin 
  digitalWrite(Trig1, LOW); 
  delayMicroseconds(2); 
  digitalWrite(Trig1, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(Trig1, LOW); 
  distance1 = pulseIn(Echo1, HIGH) / 58.0; //算成厘米 
  distance1 = (int(distance1 * 100.0)) / 100.0; //保留两位小数  

  //发一个10ms的高脉冲去触发TrigPin 
  digitalWrite(Trig2, LOW); 
  delayMicroseconds(2); 
  digitalWrite(Trig2, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(Trig2, LOW); 
  distance2 = pulseIn(Echo2, HIGH) / 58.0; //算成厘米 
  distance2 = (int(distance2 * 100.0)) / 100.0; //保留两位小数  
}

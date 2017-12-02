#ifndef _SERIAL_H
#define _SERIAL_H
#include <HardwareSerial.h>
#include <Arduino.h>
//- - - - -  无线控制，包括蓝牙控制和WiFi控制
extern String DataReceive ;
extern bool NewCommandReceived ; //数据接收判断
void Serial_init();
void Serial_SendSpeedAngleT(float _LeftPWMnum,float  _RightPWMnum,float  _Angle,float  _AngleOut,float  _AngleOut2,float  _SpeedOut) ;
void Serial_SendSpeedAngleGPS(float _LeftPWMnum,float  _RightPWMnum,float  _Angle,char*  _TIME, char* _lati,char* _long);
void Serial_SendOthers(float _AngleKp,float _AngleKd, float _SpeedKp,float  _SpeedKi);
void Serial_Receive();
#endif

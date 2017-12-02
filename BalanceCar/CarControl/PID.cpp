#include "PID.h"


//float AngleOut=0; //角度PID后得到的最终控制量
//float AngleKp = 55 , AngleKd = 1.0 , AngleKi = 0 ; //角度PID的参数
//// 38/0.58
//float SpeedOut=0; //速度PID后得到的最终控制量上
//float SpeedKp = 3.5, SpeedKd = 0, SpeedKi =  SpeedKp/200+0.010; //速度PID的参数
////      3.5/0.1058 
//float Position=0  ; //速度PID参数
//float TurnKp = 0, TurnKd = 0, TurnKi = 0 ; //旋转PID的参数

//#define Accel_bias 0 //加速度计的零点漂移
//#define Angle_bias 0  //机械倾角
//#define SpeedSetPoint 0
//#define PositionSetPiont 0

MyPID AnglePID;
MyPID SpeedPID;

void PID_init(){
  AnglePID.Kp = 55;
  AnglePID.Kd = 1.0;
  AnglePID.Ki = 0;
  AnglePID.PIDOut = 0;
  SpeedPID.Kp = 3.5;
  SpeedPID.Kd = 0;
  SpeedPID.Ki =  SpeedPID.Kp/200+0.010;
  SpeedPID.PIDOut = 0;
  return ;
}
void PID_CalAngle(float _Angle, float _Gyro_gx) {
 
//  AnglePID.PIDOut = AnglePID.Kp * (_Angle - Angle_bias) + AnglePID.Kd * _Gyro_gx;
  AnglePID.PIDOut = AnglePID.Kp * (_Angle - 0) + AnglePID.Kd * _Gyro_gx;
}

void PID_CalSpeed(float Speed, float* _Position , int run_back) {

//  Speed = ( _LeftSpeed + _RightSpeed) * 1.0;
//  LPF_1st_Factor_Cal(&OldSpeed, &Speed, 0.3);
 // float Error =  SpeedSetPoint - Speed ;
  float Error =  0 - Speed ;
  *_Position += Speed ;

  *_Position = *_Position + run_back; //run_back 的正负加到积分项中，相当于在短时间内，让小车前进或后退多少路程
  *_Position = constrain((*_Position) , -3550, 4100);

  // SpeedPID.PIDOut = SpeedPID.Kp * Error  + SpeedPID.Ki * (PositionSetPiont - (*_Position)) ;
  SpeedPID.PIDOut = SpeedPID.Kp * Error  + SpeedPID.Ki * (0 - (*_Position)) ;
}

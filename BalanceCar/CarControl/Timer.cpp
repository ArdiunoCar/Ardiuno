#include "Timer.h"


int Count40ms;
int Count50ms;
bool Count1s=false;
float Speed  ; //经过一阶低通滤波后得到的最终速度
float OldSpeed = 0; //前一次滤波后得到的最终速度
float Position = 0;
float LeftPWMnum = 0, RightPWMnum = 0; //左右轮最终的PWM值，范围 0~255

//float Speed = 0 ; //经过一阶低通滤波后得到的最终速度


float LeftSpeed = 0 , RightSpeed = 0 ; //以5ms为周期，计算左右轮的速度

void Timer2_Inter_5ms() {
  sei(); 
  MPU6050_GetData();
  MPU6050_CalAngleGryo_gx();
  MPU.Angle = Kalman_Filter_CalAngle(MPU.Angle_az,MPU.Gyro_gx,MPU.Angle);
  PID_CalAngle(MPU.Angle,MPU.Gyro_gx);
  Speed_Cal(LeftPulseCount, RightPulseCount, LeftPWMnum, RightPWMnum, &LeftSpeed, &RightSpeed);
  LeftPulseCount = 0;
  RightPulseCount = 0;
  if (Count40ms >= 8) {
    Count40ms = 0;
    Speed = ( LeftSpeed + RightSpeed) * 1.0;
    LPF_1st_Factor_Cal(&OldSpeed, &Speed, 0.3);
  //  SpeedOut = Speed_PID(LeftSpeed, RightSpeed, SpeedKp,  SpeedKi, &Position,run_back);  ///////////////////////////////////////
    PID_CalSpeed(Speed, &Position, run_back);
    LeftSpeed = 0;
    RightSpeed = 0;
  }
  else Count40ms++;
 // PWM_Cal(MPU.Angle, AnglePID.PIDOut ,  SpeedPID.PIDOut ,TurnOut , &LeftPWMnum, &RightPWMnum, &Position); //////////////////////////////////////////////////////////
  PWM_Cal(MPU.Angle, AnglePID.PIDOut ,  SpeedPID.PIDOut ,TurnOut , &LeftPWMnum, &RightPWMnum, &Position);
  PWM_SetMotor(LeftPWMnum, RightPWMnum);
  if(Count50ms >= 200){
      Count50ms = 0;
      Count1s = !Count1s;
  }
  else Count50ms ++;
 
}

void Timer_init(){
  MsTimer2::set(5, Timer2_Inter_5ms  ); // 5ms定时中断
  MsTimer2::start();
}






//- - - - - - - - 以5ms为周期，计算左右轮的速度
void Speed_Cal(int _LeftPulseCount, int _RightPulseCount, float _LeftPWMnum, float _RightPWMnum, float* _LeftSpeed, float* _RightSpeed ) {

  if (_LeftPWMnum < 0 )  *_LeftSpeed -= _LeftPulseCount;
  else    *_LeftSpeed += _LeftPulseCount;
  if (_RightPWMnum < 0 ) *_RightSpeed -= _RightPulseCount;
  else  *_RightSpeed += _RightPulseCount;


  //Speed = (FinalLCount + FinalRCount) * 0.8;
  // Speed = ( LeftSpeed + RightSpeed) * 1.0;
  // LPF_1st_Factor_Cal(&OldSpeed, &Speed, 0.3);
}










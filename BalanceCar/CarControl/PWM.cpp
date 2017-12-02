#include "PWM.h"


void PWM_init(){
  
  pinMode(LeftPWM, OUTPUT);
  pinMode(RightPWM, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  analogWrite(LeftPWM, 0);
  analogWrite(RightPWM, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  return ;
}

/*****************PWM计算****************************/
void PWM_Cal(float _Angle, float _AngleOut, float _SpeedOut, float _TurnOut,float* _LeftPWMnum, float* _RightPWMnum, float* _Position) {

  if (_Angle < -50 || _Angle > 50) {
    *_LeftPWMnum = 0;
    *_RightPWMnum = 0;
    *_Position = 0 ;
  }
  else {
    *_LeftPWMnum = -_AngleOut - _SpeedOut+_TurnOut;
    *_RightPWMnum = -_AngleOut - _SpeedOut-_TurnOut;
    *_LeftPWMnum = constrain(*_LeftPWMnum, -255, 255);
    *_RightPWMnum = constrain(*_RightPWMnum, -255, 255);
  }

}

void PWM_SetMotor(float _LeftPWMnum, float _RightPWMnum) {
  if (_LeftPWMnum >= 0 ) {
    digitalWrite(AIN1, 1);  //沿着y轴负方向前进
    digitalWrite(AIN2, 0);
    analogWrite(LeftPWM, _LeftPWMnum);
  }
  else {
    digitalWrite(AIN1, 0);  //沿着y轴正方向前进
    digitalWrite(AIN2, 1);
    analogWrite(LeftPWM, -_LeftPWMnum);
  }

  if (_RightPWMnum >= 0 ) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(RightPWM , _RightPWMnum);
  }
  else {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(RightPWM , -_RightPWMnum);
  }
  return ;
}

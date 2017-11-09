

#define Accel_bias 0 //加速度计的零点漂移
#define Gyro_bias  -128.1 //陀螺仪的零点漂移
#define Angle_bias 0  //机械倾角
#define SpeedSetPoint 0
#define PositionSetPiont 0


float Speed = 0 ; //经过一阶低通滤波后得到的最终速度
float OldSpeed = 0; //前一次滤波后得到的最终速度



void Angle_Cal(float _Accel_y, float _Accel_z, float _Gyro_x, float* _Angle_az, float* _Gyro_gx, float* _Angle) {
  /******************加速度***************
     范围为+/- 2g，换算关系：16384 LSB/g
     角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
     因为x>=sinx,故乘以1.2适当放大
  */
  //Angle_ax = (Accel_x - Accel_bias) / 16384 ; //去除零点偏移,计算得到角度（弧度）
  //Angle_ax = Angle_ax * (180 / 3.14) * 1.2 ; //弧度转换为度并放大
  *_Angle_az = (double) atan2(_Accel_y, _Accel_z) * (180 / 3.14); //计算与Z轴的夹角，即Pitch角，并转换为度
  /******************加速度****************/

  /******************角速度***************
      范围为+/- 250deg/s时，换算关系：131LSB/(deg/s)
  */

  *_Gyro_gx = (double) (_Gyro_x + Gyro_bias ) / 131; //去除零点偏移，计算角速度值【负号为方向处理】
  /*********注：Gyro_x表示绕x轴旋转的角速度，而在原厂的小车中，前进方向为y轴正方向，则x轴正方向即为相对于y轴顺时针旋转90度得到。因此，Gyro_x积分得到的也是Pitch角**********************
    /******************角速度***************/

  //- - - - - - - - -  卡尔曼滤波融合
  Kalman_Filter(*_Angle_az , *_Gyro_gx, _Angle ); //卡尔曼滤波计算倾角
}


float Angle_PID(float _Angle, float _Gyro_gx,  float _AngleKp, float _AngleKd ) {
  float _AngleOut;
  _AngleOut = _AngleKp * (_Angle - Angle_bias) + _AngleKd * _Gyro_gx;
  return _AngleOut;
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

float Speed_PID(float _LeftSpeed,float  _RightSpeed, float _SpeedKp, float _SpeedKi, float* _Position , int run_back) {

  Speed = ( _LeftSpeed + _RightSpeed) * 1.0;
  LPF_1st_Factor_Cal(&OldSpeed, &Speed, 0.3);
  float Error =  SpeedSetPoint - Speed ;
  *_Position += Speed ;
  if (run_back>0){
    *_Position = *_Position + 550 ;
  }
  else {
     *_Position = *_Position +run_back ;
  }
  //*_Position = *_Position + run_back; //run_back 的正负加到积分项中，相当于在短时间内，让小车前进或后退多少路程
  *_Position = constrain((*_Position) , -3550, 4100);
//  if(*_Position <-3550) *_Position = -3550;
 // else if(*_Position>3550) *_Position = 3550;
 // Serial.print(Speed);
//  Serial.print("  ");
//  Serial.println(*_Position);
  float SpeedOut;
  SpeedOut = SpeedKp * Error  + SpeedKi * (PositionSetPiont - (*_Position)) ;
 // Serial.print(SpeedOut);
  return SpeedOut;
}

//float Turn_PID


/*****************PWM计算****************************/
void PWM_Cal(float _Angle, float _AngleOut, float _SpeedOut, float _TurnOut,float* _LeftPWMnum, float* _RightPWMnum, float* _Position) {

  if (_Angle < -50 || _Angle > 50) {
    //      CarState = STOP;
    *_LeftPWMnum = 0;
    *_RightPWMnum = 0;
    *_Position = 0 ;
  }
  else {
    //  Kp  =1;
    //    constrain(AngleOut,-255,255);
    *_LeftPWMnum = -_AngleOut - _SpeedOut+TurnOut;
    *_RightPWMnum = -_AngleOut - _SpeedOut-TurnOut;
    //   LeftPWMnum =  - SpeedOut ;
    //  RightPWMnum =  - SpeedOut;
    *_LeftPWMnum = constrain(*_LeftPWMnum, -255, 255);
    *_RightPWMnum = constrain(*_RightPWMnum, -255, 255);
  }

}

void Set_Motor(float _LeftPWMnum, float _RightPWMnum) {
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

}

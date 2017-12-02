#include "Filter.h"

//******卡尔曼参数参数************

float  Q_angle = 0.001;
float  Q_gyro = 0.005;
float  R_angle = 0.5;
float  dt = 0.005;                //dt为kalman滤波器采样时间;
char   C_0 = 1;
float  Q_bias, Angle_err;
float  PCt_0, PCt_1, E;
float  K_0, K_1, t_0, t_1;
float  Pdot[4] = {0, 0, 0, 0};
float  PP[2][2] = { { 1, 0 }, { 0, 1 } };

//*********************************************************


/*************卡尔曼滤波********************************/
float Kalman_Filter_CalAngle(float _Accel, float _Gyro, float _Angle) // 滤波
{
  _Angle += (_Gyro - Q_bias) * dt; //先验估计陀螺角度

  Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

  Pdot[1] = - PP[1][1];
  Pdot[2] = - PP[1][1];
  Pdot[3] = Q_gyro;

  PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
  PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;

  Angle_err = _Accel - _Angle;//zk-先验估计

  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];

  E = R_angle + C_0 * PCt_0;

  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;

  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];

  PP[0][0] -= K_0 * t_0; //后验估计误差协方差
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;

  _Angle += K_0 * Angle_err; //后验估计
  Q_bias += K_1 * Angle_err; //后验估计
  //Gyro_y  = _Gyro - Q_bias; //输出值(后验估计)的微分=角速度
  return _Angle;
}
/*************卡尔曼滤波********************************/



/*********一阶互补滤波器********************/

void   LPF_1st_Factor_Cal(float* OldData, float* NewData, float lpf_factor)
{
  *NewData = ( *OldData) * (1 - lpf_factor) + (*NewData) * lpf_factor;
  *OldData = *NewData ;
}
/*********一阶互补滤波器********************/

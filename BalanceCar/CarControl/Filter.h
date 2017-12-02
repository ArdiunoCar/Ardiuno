#ifndef _FILTER_H
#define _FILTER_H

float Kalman_Filter_CalAngle(float _Accel, float _Gyro, float _Angle);
void   LPF_1st_Factor_Cal(float* OldData, float* NewData, float lpf_factor);
#endif

#ifndef _MYMPU6050_H
#define _MYMPU6050_H


#include "MPU6050.h"
#include "I2Cdev.h"


struct MyMPU6050{  
  int16_t Accel_x, Accel_y, Accel_z; //加速度计直接得到的三轴加速度
  int16_t Gyro_x, Gyro_y, Gyro_z;//陀螺仪直接得到的角速度
  float Angle_az;//由加速度计算的倾斜角度
  float Gyro_gx; //由陀螺仪计算的角速度
  float Angle; //小车最终倾斜角度
  float Temp; //温度
};
extern MyMPU6050 MPU;
void  MPU6050_init();
void MPU6050_GetData();
void MPU6050_CalAngleGryo_gx();
void MPU6060_GetTemperature();

#endif

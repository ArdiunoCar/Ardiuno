#include "MyMPU6050.h"

#define Gyro_bias  -128.1 //陀螺仪的零点漂移
MPU6050 Mpu6050; //实例化一个MPU6050的对象，用于调用库函数
MyMPU6050 MPU;
void  MPU6050_init(){
  Wire.begin(); // join I2C bus
  Mpu6050.initialize();  // initialize device
  bool isConnect = Mpu6050.testConnection(); //test for connecting
  digitalWrite(LED_BUILTIN, isConnect);
  return ;
}

void MPU6050_GetData(){
  Mpu6050.getMotion6(&MPU.Accel_x, &MPU.Accel_y, &MPU.Accel_z, &MPU.Gyro_x, &MPU.Gyro_y, &MPU.Gyro_z);
  return ;
}

void MPU6050_CalAngleGryo_gx() {
  /******************加速度***************
     范围为+/- 2g，换算关系：16384 LSB/g
     角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
     因为x>=sinx,故乘以1.2适当放大
  */
  //Angle_ax = (Accel_x - Accel_bias) / 16384 ; //去除零点偏移,计算得到角度（弧度）
  //Angle_ax = Angle_ax * (180 / 3.14) * 1.2 ; //弧度转换为度并放大
  MPU.Angle_az = (double) atan2(MPU.Accel_y, MPU.Accel_z) * (180 / 3.14); //计算与Z轴的夹角，即Pitch角，并转换为度
  /******************加速度****************/

  /******************角速度***************
      范围为+/- 250deg/s时，换算关系：131LSB/(deg/s)
  */

  MPU.Gyro_gx = (double) (MPU.Gyro_x + Gyro_bias ) / 131; //去除零点偏移，计算角速度值【负号为方向处理】
  /*********注：Gyro_x表示绕x轴旋转的角速度，而在原厂的小车中，前进方向为y轴正方向，则x轴正方向即为相对于y轴顺时针旋转90度得到。因此，Gyro_x积分得到的也是Pitch角**********************
    /******************角速度***************/
  return ;
}
void MPU6060_GetTemperature(){
  double rawTemp = Mpu6050.getTemperature();
  MPU.Temp = (((double) (rawTemp + 13200)) / 280)-13;
  return ;
}


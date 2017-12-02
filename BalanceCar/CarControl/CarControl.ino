/**************
     Arduino uno r3 有两个外部中断可用，分别为：
                                               Pin2
                                               Pin3

     ATMega328(UNO) 有三个计时器Timer0-2，一些延时语句如delay（），mills（），micro（）用的是Timer0
     6个PWM输出脚用到计时器的对应关系如下：
                                                Pin 5 和 6： Timer0
                                                Pin 9 和 10：Timer1 
                                                Pin 11 和 3：Timer2

    为避免冲突，管脚分配如下：


               TB6612FNG驱动模块                   霍尔编码器模块                    MPU6050            蓝牙模块           超声波模块  1        超声波模块2       
          Pin9   -->   左电机PWM/PWMA   //      Pin2   -->   左编码器A相   //     SCL/A5 --> SCL  //   0 -->  TX     //   A2 --> Tri      //   A0 --> Tri 
          Pin10  -->   右电机PWM/PWMB   //      Pin3   -->   右编码器A相   //    SDA/A4 --> SDA   //   1 --> RX     //   A3 --> Echo    //   A1 --> Echo
          6 -->  AIN1   7 -->  AIN2     //      Pin4   -->   左编码器B相（预留）
          12 --> BIN1   13 --> BIN2     //      Pin5   -->   右编码器B相（预留）
    剩余管脚：
          Pin8
          Pin11 
*/

/*********包含必要的库文件****************/

#include "PWM.h"
#include "MyMPU6050.h"
//#include "GPS.h"
#include "Filter.h"
#include "Timer.h"
#include "PID.h"
#include "Interrupt.h"
#include "Serial.h"
#include "Control.h"
#include "SR04.h"
/*********包含必要的库文件****************/


void setup() {
    Timer_init();
    Interrupt_init();
    PWM_init();
    MPU6050_init();
    PID_init();
    Serial_init();
    SR04_init();
 
}


void loop() {
//  Serial.println(Count1s);
  Serial_Receive();
  Control_Wireless();
  SR04_GetDistance();f 
  Control_State();
//  
//  GPS_ReadData();  //获取GPS数据
//  GPS_ParseBuffer();//解析GPS数据
  if(Count1s){
  //   if(GPSData.isUsefull)  Serial_SendSpeedAngleGPS(Speed,Speed, MPU.Angle, GPSData.latitude,GPSData.latitude, GPSData.longitude);
  //   else 
//     MPU6060_GetTemperature();  //!!!!注:将通过MPU6050获取温度数据的函数放主函数中，会出现程序跑一半停止的奇怪现象，放if语句中可以解决，好像跟I2C读取数据的方式有关（会导致程序阻塞）
     Serial_SendSpeedAngleT(MPU.Temp ,Speed, MPU.Angle, AnglePID.PIDOut,distance1, distance2); 
  }
  //Serial.print("Gyro_gx:  "); Serial.print(Gyro_gx); 
  /* Serial.print("  Angle:  "); Serial.print(Angle);
  Serial.print("  AngleOut:  ") ; Serial.print(AngleOut);
  Serial.print("  SpeedOut:  "); Serial.print(SpeedOut);
  Serial.print("  OUT:  "); Serial.print(LeftPWMnum);
  Serial.print("  Speed: "); Serial.println(Speed);*/
  
}


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

               TB6612FNG驱动模块                   霍尔编码器模块                    MPU6050            蓝牙模块              GPS模块
          Pin9   -->   左电机PWM/PWMA   //      Pin2   -->   左编码器A相   //     SCL/A5 --> SCL  //   0 -->  TX    //       Pin8 --> RX
          Pin10  -->   右电机PWM/PWMB   //      Pin3   -->   右编码器A相   //    SDA/A4 --> SDA   //   1 --> RX2    //      
          6 -->  AIN1   7 -->  AIN2
          12 --> BIN1   13 --> BIN2
*/

/*********包含必要的库文件****************/
#include "MPU6050.h"
#include "I2Cdev.h"
#include <MsTimer2.h>
/*********包含必要的库文件****************/


/**********************管脚定义*************************/

//霍尔编码器模块
#define LeftEncoderA  2   //编码器左轮A相
#define RightEncoderA 3  //编码器右轮A相

//TB6612FNG驱动模块  
#define LeftPWM 9 
#define RightPWM 10
#define AIN1 7
#define AIN2 6
#define BIN1 13
#define BIN2 12
/*
#define AIN1 6    
#define AIN2 7
#define BIN1 12
#define BIN2 13
*/

/**********************管脚定义*************************/

struct GPS
{
  char GPS_Buffer[80];
  bool isGetData;   //是否获取到GPS数据
  bool isParseData; //是否解析完成
  char UTCTime[11];   //UTC时间
  char latitude[11];    //纬度
  char N_S[2];    //N/S
  char longitude[12];   //经度
  char E_W[2];    //E/W
  bool isUsefull;   //定位信息是否有效
} ;

volatile int LeftPulseCount = 0; //左轮A相脉冲计数
volatile int RightPulseCount = 0;//右轮A相脉冲计数




int16_t Accel_x, Accel_y, Accel_z; //加速度计直接得到的三轴加速度
int16_t Gyro_x, Gyro_y, Gyro_z;//陀螺仪直接得到的角速度
float Angle_az;//由加速度计算的倾斜角度
float Gyro_gx; //由陀螺仪计算的角速度
float Angle; //小车最终倾斜角度

float LeftSpeed = 0 , RightSpeed = 0 ; //以5ms为周期，计算左右轮的速度
float LeftPWMnum = 0, RightPWMnum = 0; //左右轮最终的PWM值，范围 0~255


float AngleOut=0; //角度PID后得到的最终控制量
float AngleKp = 55 , AngleKd = 1.0 , AngleKi = 0 ; //角度PID的参数
// 38/0.58
float SpeedOut=0; //速度PID后得到的最终控制量上
float SpeedKp = 3.5, SpeedKd = 0, SpeedKi =  SpeedKp/200+0.010; //速度PID的参数
//      3.5/0.1058 
float Position=0  ; //速度PID参数
float TurnKp = 0, TurnKd = 0, TurnKi = 0 ; //旋转PID的参数

//- - - - -  无线控制，包括蓝牙控制和WiFi控制
String DataReceive = "";
bool NewCommandReceived = false; //数据接收判断

int Count40ms = 0;  //实现每40ms进行一次电平变化
int Count50ms=0 ;

MPU6050 Mpu; //实例化一个MPU6050的对象，用于调用库函数

/*小车运行状态枚举*/
enum {
  STOP = 0,
  RUN,
  BACK,
  LEFT,
  RIGHT,
  SpinLEFT,
  SpinRIGHT
} State; //控制小车运动的指令
int CarState = STOP;
int run_back = 0;
int TurnOut = 0;


/**********外部中断 ----  霍尔编码器A相信号**************/
void LeftPulse_ADD() {
  LeftPulseCount ++ ;
}
void RightPulse_ADD() {
  RightPulseCount ++ ;
}
/**********外部中断 ----  霍尔编码器A相信号**************/







/*******************无线控制，包括蓝牙控制和微信控制***********/
void Remote_Control() {
  if (NewCommandReceived == true) {
    switch (DataReceive[1]) {
      case  '1':  CarState = RUN; break;
      case  '2':  CarState = BACK; break;
      case  '3':  CarState = LEFT; break;
      case  '4':  CarState = RIGHT; break;
      case  '5':  CarState = STOP; break;
      default: CarState = STOP; break;
    }
    if (DataReceive[3] == '1' ) {
      CarState = SpinLEFT;
      Serial.print("$OK#");
    }
    else if (DataReceive[3] == '2') {
      CarState = SpinRIGHT;
    }
    if (DataReceive[5] == '1') {    //查询PID
      String DataSend = Send_Data(AngleKp, AngleKd,  SpeedKp, SpeedKi);
      Serial.print(DataSend);
    }
    else if (DataReceive[5] == '2') { //恢复PID
      Reset_PID(&AngleKp, &AngleKd,  &SpeedKp, &SpeedKi);
      Serial.print("$OK#");
    }

    if (DataReceive[7] == '1') { //自动上报
      Serial.print("$OK#");
    }
    else if (DataReceive[7] == '2') { //停止自动上报
      Serial.print("$OK#");
    }
    if (DataReceive[9] == '1') { //角度PID更新
      Renew_AnglePID(DataReceive,&AngleKp, &AngleKd);
      Serial.print("$OK#");
    }

    if (DataReceive[11] == '1') { //速度PID更新
      Renew_AccelPID(DataReceive,&SpeedKp,&SpeedKi);
      Serial.print("$OK#");
    }

    //数据处理完成
    DataReceive = "";
    NewCommandReceived = false;
  }
}
/*******************无线控制，包括蓝牙控制和微信控制***********/




/***********串口接收数据*************/
//void Serial_Receive() {
//  
//  if (Serial.available()) {
//    DataReceive = "";
//    bool isDataBegin = false;
//    bool isDataEnd = false;
//    while (Serial.available() && isDataEnd == false) {
//      delay(10);
//      int temp = Serial.read();
//      //Serial.println(temp);
//      //-------------------------wifi指令-----------------------------//
//      if(temp == 0xFF){
//          //  Serial.println("test");
//           unsigned char Com[3];
//           int DataCount = 0;
//           memset(Com ,0,sizeof(Com));
//           while(Serial.available()){
//                delay(10);
//                temp = Serial.read();
//                if(DataCount < 3)Com[DataCount ++] = temp;
//           }
//           if(temp == 0xFF && DataCount == 3){
//                if(Com[1]==0x00){    //停止
//                    DataReceive = "$0,0,0,0,0,0,0,0,0,0" ;
//                }
//                else if(Com[1]==0x01){   //前进
//                    DataReceive = "$1,0,0,0,0,0,0,0,0,0" ;
//                }
//                else if(Com[1]==0x02){   //后退
//                    DataReceive = "$2,0,0,0,0,0,0,0,0,0" ;
//                }
//                else if(Com[1]==0x04){  //左转
//                     DataReceive = "$3,0,0,0,0,0,0,0,0,0" ;
//                }
//                else if(Com[1]==0x03){   //右转
//                     DataReceive = "$4,0,0,0,0,0,0,0,0,0" ;
//                }
//                isDataBegin = true ;
//                isDataEnd = true ;
//           }
//      }
//    //  Serial.println(DataReceive);
//      /*-------------------------wifi指令-----------------------------*/
//      
//      if (temp == '$') {
//        isDataBegin = true ;
//      }
//      else if (temp == '#') {
//        isDataEnd = true ;
//      }
//      if (isDataBegin && !isDataEnd )DataReceive += (char)temp;
//    }
//    if (isDataBegin && isDataEnd) {
//      NewCommandReceived = true;
//    }
//  }
//}
/***********串口接收数据*************/ 





void Timer2_Inter_5ms() {
  sei();
  Mpu.getMotion6(&Accel_x, &Accel_y, &Accel_z, &Gyro_x, &Gyro_y, &Gyro_z);
  
  Angle_Cal(Accel_y, Accel_z, Gyro_x, &Angle_az, &Gyro_gx, &Angle);
  AngleOut = Angle_PID(Angle,Gyro_gx,AngleKp, AngleKd);
  
  Speed_Cal(LeftPulseCount, RightPulseCount, LeftPWMnum, RightPWMnum, &LeftSpeed, &RightSpeed);
  LeftPulseCount = 0;
  RightPulseCount = 0;
  if (Count40ms >= 8) {
    Count40ms = 0;
    SpeedOut = Speed_PID(LeftSpeed, RightSpeed, SpeedKp,  SpeedKi, &Position,run_back);  ///////////////////////////////////////
    LeftSpeed = 0;
    RightSpeed = 0;
   // CarState = STOP;
  }
  else Count40ms++;
    //Serial.print(" aaa    ");
  //Serial.println(AngleOut);
  PWM_Cal(Angle, AngleOut,  SpeedOut ,TurnOut , &LeftPWMnum, &RightPWMnum, &Position); //////////////////////////////////////////////////////////
  Set_Motor(LeftPWMnum, RightPWMnum);
  if(Count50ms >= 200)Count50ms = 0;
  else Count50ms ++;
  
}





void setup() {

  LeftPulseCount = 0;
  RightPulseCount = 0;

  attachInterrupt( digitalPinToInterrupt(LeftEncoderA), LeftPulse_ADD, CHANGE );
  attachInterrupt( digitalPinToInterrupt(RightEncoderA), RightPulse_ADD, CHANGE );

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

  Wire.begin(); // join I2C bus
  Mpu.initialize();  // initialize device
  bool isConnect = Mpu.testConnection(); //test for connecting
  digitalWrite(LED_BUILTIN, isConnect);

  MsTimer2::set(5, Timer2_Inter_5ms  ); // 5ms定时中断
  MsTimer2::start();
  Serial.begin(9600);

  GPS_init();
  
}

void loop() {


//  Serial_Receive();
//  Remote_Control();
  gpsRead();  //获取GPS数据
  parseGpsBuffer();//解析GPS数据
  if(Count50ms==0){
     String DataSend = "Error without data!";
     GPS GPSData = getGPSData();
     if(GPSData.isUsefull)  DataSend = Send_Speed_Angle_GPS(LeftPWMnum,RightPWMnum, Angle, GPSData.latitude,GPSData.latitude, GPSData.longitude);
     else 
        DataSend = Send_Speed_Angle(LeftPWMnum,RightPWMnum, Angle, AngleOut,AngleOut, SpeedOut); 
  //  String DataSend = Send_Speed_Angle(LeftPWMnum,RightPWMnum, Angle, AngleOut,AngleOut, SpeedOut); 
 //  printGpsBuffer();//输出解析后的数据
     Serial.println(DataSend);
  }
  //Serial.print("Gyro_gx:  "); Serial.print(Gyro_gx); 
  /* Serial.print("  Angle:  "); Serial.print(Angle);
  Serial.print("  AngleOut:  ") ; Serial.print(AngleOut);
  Serial.print("  SpeedOut:  "); Serial.print(SpeedOut);
  Serial.print("  OUT:  "); Serial.print(LeftPWMnum);
  Serial.print("  Speed: "); Serial.println(Speed);*/
  switch (CarState){
    case BACK:
      run_back = -380;
      TurnOut = 0;
      break;
    case RUN:
      run_back = 300;
      TurnOut = 0;
      break;
    case LEFT: 
      run_back = 0;
      TurnOut = 70;
      break;
    case RIGHT:
      run_back = 0;
      TurnOut = -70;
      break;
    case SpinLEFT:
      run_back = 0;
      TurnOut = 70;
      break;
    case SpinRIGHT:
      run_back = 0;
      TurnOut = -70;
      break;
    default:
      run_back = 0;
      TurnOut = 0;
      break;  
  }
  //delay(20);
  //CarState = STOP;
 // Serial.print(SpeedOut);
 // Serial.print("  ");
 // Serial.println(AngleOut);

  
}


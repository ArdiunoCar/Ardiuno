#include "Serial.h"

//- - - - -  无线控制，包括蓝牙控制和WiFi控制
String DataReceive = "";
bool NewCommandReceived = false; //数据接收判断

void Serial_init(){
    Serial.begin(9600);
}


/***********串口接收数据*************/
void Serial_Receive() {
  
  if (Serial.available()) {
    DataReceive = "";
    bool isDataBegin = false;
    bool isDataEnd = false;
    while (Serial.available() && isDataEnd == false) {
      delay(10);
      int temp = Serial.read();
      //Serial.println(temp);
      //-------------------------wifi指令-----------------------------//
      if(temp == 0xFF){
          //  Serial.println("test");
           unsigned char Com[3];
           int DataCount = 0;
           memset(Com ,0,sizeof(Com));
           while(Serial.available()){
                delay(10);
                temp = Serial.read();
                if(DataCount < 3)Com[DataCount ++] = temp;
           }
           if(temp == 0xFF && DataCount == 3){
                if(Com[1]==0x00){    //停止
                    DataReceive = "$0,0,0,0,0,0,0,0,0,0" ;
                }
                else if(Com[1]==0x01){   //前进
                    DataReceive = "$1,0,0,0,0,0,0,0,0,0" ;
                }
                else if(Com[1]==0x02){   //后退
                    DataReceive = "$2,0,0,0,0,0,0,0,0,0" ;
                }
                else if(Com[1]==0x04){  //左转
                     DataReceive = "$3,0,0,0,0,0,0,0,0,0" ;
                }
                else if(Com[1]==0x03){   //右转
                     DataReceive = "$4,0,0,0,0,0,0,0,0,0" ;
                }
                isDataBegin = true ;
                isDataEnd = true ;
           }
      }
    //  Serial.println(DataReceive);
      /*-------------------------wifi指令-----------------------------*/
      
      if (temp == '$') {
        isDataBegin = true ;
      }
      else if (temp == '#') {
        isDataEnd = true ;
      }
      if (isDataBegin && !isDataEnd )DataReceive += (char)temp;
    }
    if (isDataBegin && isDataEnd) {
      NewCommandReceived = true;
    }
  }
}
/***********串口接收数据*************/ 



void Serial_SendOthers(float _AngleKp,float _AngleKd, float _SpeedKp,float  _SpeedKi) {
  /* 函数：char* dtostrf(double _val,signed char _width, unsigned char prec, char* _s)
          _val:要转换的float或者double值；  _width:转换后整数部分长度； _prec：转换后小数部分长度； _s:保存到该char数组中
  */
  String strAngleKp, strAngleKd, strSpeedKp, strSpeedKi;
  char temp[10] = {0};

  memset(temp, 0, sizeof(temp));
  dtostrf(_AngleKp, 2, 2, temp);
  strAngleKp = temp;

  memset(temp, 0 , sizeof(temp));
  dtostrf(_AngleKd, 2, 2, temp);
  strAngleKd = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf(_SpeedKp, 2, 2, temp);
  strSpeedKp = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf(_SpeedKi, 2, 2, temp);
  strSpeedKi = temp;
  
  String QueryPID;
  QueryPID = "$0,0,0,0,0,0,AP" + strAngleKp + ",AD" + strAngleKd + ",VP" + strSpeedKp + ",VI" + strSpeedKi + "#";
  return QueryPID ;
}

void Serial_SendSpeedAngleGPS(float _LeftPWMnum,float  _RightPWMnum,float  _Angle,char*  _TIME, char* _lati,char* _long) {
  char temp[10];
  String LV, RV, AC, GY, CSB, VT;

  memset(temp, 0, sizeof(temp));
  dtostrf( _LeftPWMnum, 3, 1, temp);
  LV = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( _RightPWMnum, 3, 1, temp);
  RV = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( _Angle, 2, 2, temp);
  AC = temp;
  
  GY = "";
  if(_lati==NULL) GY="0";
  else for (int i =0 ; i< sizeof(_lati);i++) GY +=_lati[i];
  
  CSB = GY;
  
  VT = "";
  if(_long==NULL)VT ="0";
  else for (int i =0 ; i< sizeof(_long);i++) GY +=_long[i];

  
  String DataSend = "$LV" + LV + ",RV" + RV + ",AC" + AC + ",GY" + GY + ",CSB" + CSB + ",VT" + VT + "#";
 // Serial.println(VT);
  Serial.println(DataSend);
  return ;
}

void Serial_SendSpeedAngleT(float Temperature,float  Speed,float  _Angle,float  _AngleOut,float  Distance1,float  Distance2) {
  char temp[10];
  String LV, RV, AC, GY, CSB, VT;

  memset(temp, 0, sizeof(temp));
  dtostrf( Temperature, 3, 1, temp);
  LV = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( Speed, 3, 1, temp);
  RV = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( _Angle, 2, 2, temp);
  AC = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( _AngleOut, 2, 2, temp);
  GY = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( Distance1, 2, 2, temp);
  CSB = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( Distance2, 2, 2, temp);
  VT = temp;
  String DataSend = "$LV" + LV + ",RV" + RV + ",AC" + AC + ",GY" + GY + ",CSB" + CSB + ",VT" + VT + "#";
//  return DataSend; //返回协议数据包
  Serial.println(DataSend);
  return ;
}




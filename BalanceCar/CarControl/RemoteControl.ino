/*******************无线控制，包括蓝牙控制和微信控制***********
      微信云在与Arduino通信时遵循如下协议：$0,0,0,0,0,0,0,0,0,0,0,100,4200#(即微信发送给arduino的数据遵循以上格式)
      其中，(注：符号也算一个数据位)
          协议中第1位数据和指令关系对应如下：1 --> RUN ; 2 --> BACK ; 3 --> LEFT ; 4 --> RIGHT ; 5 --> STOP
          协议中第11位数据和指令关系对应如下： 1 --> SpinLEFT ; 2 --> SpinRIGHT (与代码不对应)
          协议中第21位和指令关系对应如下：1 --> STOP （与代码不对应）
      蓝牙与Arduino通信时遵循如下协议：
          下位机接收数据格式（即蓝牙发送给Arduino的数据格式）：'$0,0,0,0,0,0,0,0,0,0#'  浮点数字符串（或者'$0,0,0,0,1,0,AP23.4,AD0.45,VP0,VI0#'）
          上位机接收数据格式（即Arduino发送给蓝牙的数据格式）：'$LVx,RVx,ACx,GYx,CSBx,VIx#'（发送传感器数据和电机速度）  浮点数字符串，或者'$0,0,0,0,1,0,AP23.4,AD0.45,VP0,VI0#'（查询PID参数）
      其中，
          协议中第1位数据和指令关系对应如下：1 --> RUN ; 2 --> BACK ; 3 --> LEFT ; 4 --> RIGHT ; 0 --> STOP
          协议中第3位数据和指令关系对应如下：1 --> SpinLEFT ; 2 --> SpinRIGHT ;
          协议中第5位数据和指令关系对应如下：1 --> 查询PID ; 2 --> 恢复原始PID
          协议中第7位数据和指令关系对应如下：1 --> 自动上报 ; 2 --> 停止上报
          协议中第9位数据和指令关系对应如下：1 --> 角度PID更新
          协议中第11位数据和指令关系对应如下：1 --> 速度PID更新

*/

float PIDParam[3][3] = { 38, 0, 0.58, -3.5, -0.1058, 0, 0, 0, 0 };  


void Renew_AnglePID(String _DataReceive,float* _AngleKp, float* _AngleKd ) {
  int BeginIndex, EndIndex, dotIndex, Integer, Fraction;
  String strAP, strAD;

  BeginIndex = _DataReceive.indexOf("AP");
  EndIndex = _DataReceive.indexOf(",", BeginIndex + 1);
  strAP = _DataReceive.substring(BeginIndex + 2, EndIndex);
  dotIndex = strAP.indexOf(".");
  Integer = (strAP.substring(0, dotIndex + 1)).toInt();
  Fraction = (strAP.substring(dotIndex + 1)).toInt();
  *_AngleKp = (float) ( Integer + (float) ( Fraction / pow(10, ( strAP.substring(dotIndex + 1) ).length() ) ));

  BeginIndex = _DataReceive.indexOf("AD");
  EndIndex = _DataReceive.indexOf(",", BeginIndex + 1);
  strAD = _DataReceive.substring(BeginIndex + 2, EndIndex);
  dotIndex = strAD.indexOf(".");
  Integer = (strAD.substring(0, dotIndex + 1)).toInt();
  Fraction = (strAD.substring(dotIndex + 1)).toInt();
  *_AngleKd = (float) ( Integer + (float) ( Fraction / pow(10, ( strAD.substring(dotIndex + 1) ).length() ) ));
}

void Renew_AccelPID(String _DataReceive,float*  _SpeedKp,float* _SpeedKi) {
  int BeginIndex, EndIndex, dotIndex, Integer, Fraction;
  String strVP, strVI;
  BeginIndex = _DataReceive.indexOf("VP");
  EndIndex = _DataReceive.indexOf(",", BeginIndex + 1);
  strVP = _DataReceive.substring(BeginIndex + 2, EndIndex);
  dotIndex = strVP.indexOf(".");
  Integer = (strVP.substring(0, dotIndex + 1)).toInt();
  Fraction = (strVP.substring(dotIndex + 1)).toInt();
  *_SpeedKp = (float) ( Integer + (float) ( Fraction / pow(10, ( strVP.substring(dotIndex + 1) ).length() ) ));


  BeginIndex = _DataReceive.indexOf("VI");
  EndIndex = _DataReceive.length();
  strVI = _DataReceive.substring(BeginIndex + 2);
  dotIndex = strVI.indexOf(".");
  Integer = (strVI.substring(0, dotIndex + 1)).toInt();
  Fraction = (strVI.substring(dotIndex + 1)).toInt();
  *_SpeedKi = (float) ( Integer + (float) ( Fraction / pow(10, ( strVI.substring(dotIndex + 1) ).length() ) ));

}

String Send_Data(float _AngleKp,float _AngleKd, float _SpeedKp,float  _SpeedKi) {
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


void Reset_PID(float* _AngleKp,float* _AngleKd, float* _SpeedKp,float*  _SpeedKi) {
  *_AngleKp = PIDParam[0][0];
 // *_AngleKi = PIDParam[0][1];
  *_AngleKd = PIDParam[0][2];
  *_SpeedKp = PIDParam[1][0];
  *_SpeedKi = PIDParam[1][1];
//*_SpeedKd = PIDParam[1][2];
 // *_TurnKp  = PIDParam[2][0];
 // *_TurnKi = PIDParam[2][1];
//  *_TurnKd = PIDParam[2][2];
}



String Send_Speed_Angle(float _LeftPWMnum,float  _RightPWMnum,float  _Angle,float  _AngleOut,float  _AngleOut2,float  _SpeedOut) {
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

  memset(temp, 0, sizeof(temp));
  dtostrf( _AngleOut, 2, 2, temp);
  GY = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( _AngleOut, 2, 2, temp);
  CSB = temp;

  memset(temp, 0, sizeof(temp));
  dtostrf( _SpeedOut, 2, 2, temp);
  VT = temp;
  String DataSend = "$LV" + LV + ",RV" + RV + ",AC" + AC + ",GY" + GY + ",CSB" + CSB + ",VT" + VT + "#";
  return DataSend; //返回协议数据包
}

#include "Control.h"


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

int run_back = 0;
int TurnOut = 0;
int CarState = STOP;
int cntKeep = 0;
/*******************无线控制，包括蓝牙控制和微信控制***********/
void Control_Wireless() {
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
    if (DataReceive[7] == '1') { //自动上报
      Serial.print("$OK#");
    }
    else if (DataReceive[7] == '2') { //停止自动上报
      Serial.print("$OK#");
    }
    //数据处理完成
    DataReceive = "";
    NewCommandReceived = false;
  }
}
/*******************无线控制，包括蓝牙控制和微信控制***********/

void Control_State(){
   if(distance1<=10&&distance2<=10){
      CarState = RUN;
   }
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
   if(distance1<=10&&distance2<=10){
      CarState = STOP;
   }
}




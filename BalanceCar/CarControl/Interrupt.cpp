#include "Interrupt.h"


volatile int LeftPulseCount ; //左轮A相脉冲计数
volatile int RightPulseCount ;//右轮A相脉冲计数

/**********外部中断 ----  霍尔编码器A相信号**************/
void LeftPulse_ADD() {
  LeftPulseCount ++ ;
}
void RightPulse_ADD() {
  RightPulseCount ++ ;
}
/**********外部中断 ----  霍尔编码器A相信号**************/

void Interrupt_init(){
  
  LeftPulseCount = 0;
  RightPulseCount = 0;
  attachInterrupt( digitalPinToInterrupt(LeftEncoderA), LeftPulse_ADD, CHANGE );
  attachInterrupt( digitalPinToInterrupt(RightEncoderA), RightPulse_ADD, CHANGE );

}

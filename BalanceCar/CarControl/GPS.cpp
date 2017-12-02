#include "GPS.h"
#include <SoftwareSerial.h>

#define GPSRX 8
#define GPSTX 4

GPS GPSData;

SoftwareSerial GPSSerial(GPSRX, GPSTX);
const unsigned int gpsRxBufferLength = 600;
char gpsRxBuffer[gpsRxBufferLength];
unsigned int ii = 0;



void clrGpsRxBuffer()
{
  memset(gpsRxBuffer, 0, gpsRxBufferLength);      //清空
  ii = 0;
}

void errorLog(String num)
{
 // Serial.print("Error ocurr in "+num+"-th step!" );
  return ;
}

void GPS_init(){
   

    GPSSerial.begin(9600);
    GPSData.isGetData = false;
    GPSData.isParseData = false;
    GPSData.isUsefull = false;
}
 



void GPS_ReadData() {
  while (GPSSerial.available() )
  {
 //   Serial.write( GPSSerial.read() );
    gpsRxBuffer[ii++] = GPSSerial.read();
    if (ii == gpsRxBufferLength)clrGpsRxBuffer();
  }

  char* GPS_BufferHead;
  char* GPS_BufferTail;
  if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL )
  {
    if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead))
    {
      memcpy(GPSData.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);
      GPSData.isGetData = true;
      clrGpsRxBuffer();
    }
  }
}



void GPS_ParseBuffer()
{
  char *subString;
  char *subStringNext;
  if (GPSData.isGetData)
  {
    GPSData.isGetData = false;
    for (int i = 0 ; i <= 6 ; i++)
    {
      if (i == 0)
      {
        if ((subString = strstr(GPSData.GPS_Buffer, ",")) == NULL)
          errorLog("1");  //解析错误
      }
      else
      {
        subString++;
        if ((subStringNext = strstr(subString, ",")) != NULL)
        {
          char usefullBuffer[2]; 
          switch(i)
          {
            case 1:memcpy(GPSData.UTCTime, subString, subStringNext - subString);break; //获取UTC时间
            case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break; //获取UTC时间
            case 3:memcpy(GPSData.latitude, subString, subStringNext - subString);break;  //获取纬度信息
            case 4:memcpy(GPSData.N_S, subString, subStringNext - subString);break; //获取N/S
            case 5:memcpy(GPSData.longitude, subString, subStringNext - subString);break; //获取纬度信息
            case 6:memcpy(GPSData.E_W, subString, subStringNext - subString);break; //获取E/W

            default:break;
          }

          subString = subStringNext;
          GPSData.isParseData = true;
          if(usefullBuffer[0] == 'A')
            GPSData.isUsefull = true;
          else if(usefullBuffer[0] == 'V')
            GPSData.isUsefull = false;

        }
        else
        {
          errorLog("2");  //解析错误
        }
      }
    }
  }
}
 



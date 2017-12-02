#ifndef _GPS_H
#define _GPS_H

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


extern GPS GPSData;
void GPS_init();
void GPS_ReadData();
void GPS_ParseBuffer();
#endif

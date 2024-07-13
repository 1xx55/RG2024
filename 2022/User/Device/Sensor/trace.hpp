#ifndef TRACE_HPP
#define TRACE_HPP

#include "main.h"
class Class_Sensor
{
    public:
    int16_t Trace_Error(uint16_t line);
    void Get_Track_Info(uint8_t RxData[]);//��io��ȡ��ģ����Ϣ
    void Init_Track();//��ʼ��������Ѳ��
    void Stop_Track();//ֹͣѲ��
    uint8_t SetScanInfo(uint8_t Info);
    uint8_t GetScanInfo(uint8_t Info);
    bool IsOnTrack(uint16_t line);
		int16_t LastError[4]={0};
    uint8_t track[16]={0};
    private:
    uint8_t ScanInfo=2;
	
};


#endif

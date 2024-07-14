// 						（0）
// 	 ----0   1   2   3---- 前
// 					|		  |		
// 					0		  0  		 
// 	（2）   1		  1			（3）
// 					2		  2
// 					3		  3
// 					|		  |
// 	 ----0    1   2   3---- 后
// 						（1）
#include "trace.hpp"

int16_t Class_Sensor::Trace_Error(uint16_t line)
{
    uint16_t start = line * 4;
  	int16_t error = LastError[line];
    //int16_t error = 0;
    if( (track[start]== 1) && (track[start+1] == 0) && (track[start+2] == 1) && (track[start+3] == 1) ) //1 0 1 1
        error = -1;
    else if( (track[start] == 1) && (track[start+1] == 1) && (track[start+2] == 1) && (track[start+3] == 0) ) //1 1 1 0
        error = 4;
    else if( (track[start] == 1) && (track[start+1] == 1) && (track[start+2] == 0) && (track[start+3] == 0) ) //1 1 0 0
        error = 2;
    else if( (track[start] == 1) && (track[start+1] == 0) && (track[start+2] == 0) && (track[start+3] == 0) ) //1 0 0 0  
        error = 1;
    else if( (track[start] == 1) && (track[start+1] == 1) && (track[start+2] == 0) && (track[start+3] == 1) ) //1 1 0 1  
        error = 1;
    else if( (track[start] == 0) && (track[start+1] == 1) && (track[start+2] == 1) && (track[start+3] == 1) ) //0 1 1 1
        error = -4;
    else if( (track[start] == 0) && (track[start+1] == 0) && (track[start+2] == 1) && (track[start+3] == 1) ) //0 0 1 1
        error = -2; 
    else if( (track[start] == 0) && (track[start+1] == 0) && (track[start+2] == 0) && (track[start+3] == 1) ) //0 0 0 1
        error = -1;
    else if( (track[start] == 1) && (track[start+1] == 0) && (track[start+2] == 0) && (track[start+3] == 1) ) //1 0 0 1
        error = 0;	
  	else if( (track[start] == 0) && (track[start+1] == 0) && (track[start+2] == 0) && (track[start+3] == 0) ) //0 0 0 0
        error = 0;
	LastError[line] = error;
	return error;
}

uint8_t Class_Sensor::SetScanInfo(uint8_t Info)
{
    ScanInfo=Info;
    return ScanInfo;
}
uint8_t Class_Sensor::GetScanInfo(uint8_t Info)
{
    return ScanInfo;
}

void Class_Sensor::Get_Track_Info(uint8_t RxData[])
{
    for(int i=0;i<15;i++)
        track[i]=RxData[i];
}

bool Class_Sensor::IsOnTrack(uint16_t line)
{
//	if(line == 0)
//	{
//		if(track[1] && track[2] && track[3]) return 0;
//		else return 1;
//	}
    if(track[4*line] && track[4*line+1] && track[4*line+2] && track[line*4+3])
        return 0;
    else
        return 1;
}

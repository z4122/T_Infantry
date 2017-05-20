#ifndef DRIVERS_UARTJUDGE_LOW_H
#define DRIVERS_UARTJUDGE_LOW_H

#include "utilities_iopool.h"

#define    CM_current_bottom      3000.f
#define 	 CMFLIntensity_bottom   1000.f
#define    CMFRIntensity_bottom   1000.f
#define  	 CMBLIntensity_bottom   1000.f
#define	   CMBRIntensity_bottom   1000.f

#define    CM_current_lower      12000.f
#define 	 CMFLIntensity_lower   3500.f
#define    CMFRIntensity_lower   3500.f
#define  	 CMBLIntensity_lower   3500.f
#define	   CMBRIntensity_lower   3500.f

#define    CM_current_MAX      15000.f
#define    CMFLIntensity_MAX   4900.f
#define    CMFRIntensity_MAX   4900.f
#define    CMBLIntensity_MAX   4900.f
#define    CMBRIntensity_MAX   4900.f


typedef struct 
{
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    float remainPower;
}tGameInfo;

void judgeUartRxCpltCallback(void);
void judgeUartInit(void);
void Judge_Refresh(void);

#endif

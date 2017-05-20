#ifndef DRIVERS_UARTJUDGE_LOW_H
#define DRIVERS_UARTJUDGE_LOW_H

#include "utilities_iopool.h"

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

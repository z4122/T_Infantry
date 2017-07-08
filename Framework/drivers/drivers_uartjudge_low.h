/**
  ******************************************************************************
  * File Name          : pdrivers_uartjudge_low.h
  * Description        : 裁判系统读取
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 底层函数
  ******************************************************************************
  */
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

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

#define    CM_CURRENT_BOTTOM      3000.f
#define 	 CMFLINTENSITY_BOTTOM   1000.f
#define    CMFRINTENSITY_BOOTOM   1000.f
#define  	 CMBLINTENSITY_BOTTOM   1000.f
#define	   CMBRINTENSITY_BOTTOM   1000.f

#define    CM_CURRENT_LOWER      12000.f
#define 	 CMFLINTENSITY_LOWER   3500.f
#define    CMFRINTENSITY_LOWER   3500.f
#define  	 CMBLINTENSITY_LOWER   3500.f
#define	   CMBRINTENSITY_LOWER   3500.f

#define    CM_CURRENT_MAX      15000.f
#define    CMFLINTENSITY_MAX   4900.f
#define    CMFRINTENSITY_MAX   4900.f
#define    CMBLINTENSITY_MAX   4900.f
#define    CMBRINTENSITY_MAX   4900.f


typedef struct 
{
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    float remainPower;
}tGameInfo;

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);
void Judge_Refresh(void);

#endif

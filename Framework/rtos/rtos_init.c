/**
  ******************************************************************************
  * File Name          : rtos_init.c
  * Description        : FreeRTOS初始化
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 对于板上所有外设如定时器，串口，蜂鸣器，CAN总线，MPU6050等进行必要的初始化

  ******************************************************************************
  */
#include <tim.h>
#include <stdbool.h>
#include "rtos_init.h"
#include "drivers_platemotor.h"
#include "application_motorcontrol.h"
#include "utilities_debug.h"
#include "drivers_canmotor_low.h"
//#include "drivers_mpu6050_low.h"
#include "peripheral_tim.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartrc_low.h"
#include "tasks_cmcontrol.h"
#include "drivers_imu_low.h"
#include "utilities_tim.h"
#include "drivers_buzzer_low.h"
#include "drivers_uartjudge_low.h"

bool isInited = 0;
void rtos_init()
	{
	playMusicWhenInit();
	fw_userTimeEnable();
	MPU6500_Init();
	IST8310_Init();
	ctrlUartInit();
	RemoteTaskInit();
	UserTimerInit();
	CMControtLoopTaskInit();
	rcInit();
	motorInit();
	plateMotorInit();
	judgeUartInit();
//	mpu6050Init();
//	Init_Quaternion();
	fw_printfln("init success");
}



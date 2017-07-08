/**
  ******************************************************************************
  * File Name          : tasks_timed.h
  * Description        : 2mså®šæ—¶ä»»åŠ¡
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 2mså®šæ—¶
	* é€šè¿‡countå¯ä»¥è·å¾—500ms,1sç­‰å®šæ—¶ä»»åŠ¡
	* çŠ¶æ€æœºåˆ‡æ¢ï¼Œä¸²å£å®šæ—¶è¾“å‡ºï¼Œçœ‹é—¨ç‹—ç­‰
  ******************************************************************************
  */
#ifndef FRAMEWORK_TASKS_CMCONTROL_H
#define FRAMEWORK_TASKS_CMCONTROL_H

#include "cmsis_os.h"

void CMControtLoopTaskInit(void);
void Timer_2ms_lTask(void const * argument);
void WorkStateFSM(void);
void WorkStateSwitchProcess(void);
void CMControlLoop(void);
void ShooterMControlLoop(void);
int32_t GetQuadEncoderDiff(void);
//initiate status: 
typedef enum
{
    PREPARE_STATE,     		//ÉÏµçºó³õÊ¼»¯×´Ì¬ 4sÖÓ×óÓÒ
    STANDBY_STATE,			//ÔÆÌ¨Í£Ö¹²»×ª×´Ì¬
    NORMAL_STATE,			//ÎŞÊäÈë×´Ì¬
    STOP_STATE,        	//Í£Ö¹ÔË¶¯×´Ì¬
    CALI_STATE,    			//Ğ£×¼×´Ì¬
}WorkState_e;


WorkState_e GetWorkState(void);

#define OneShoot (725)
#define PID_SHOOT_MOTOR_SPEED      (30)
#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define PREPARE_TIME_TICK_MS 500      //prapare time in ms*2
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	1.4f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT_old \
{\
	0,\
	0,\
	{0,0},\
	1.2f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}
//D²ÎÊıÔ­À´Îª0.4
#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	6.5f,\
	0.0f,\
	1.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}
#define CHASSIS_MOTOR_SPEED_PID_DEFAULT_old \
{\
	0,\
	0,\
	{0,0},\
	220.f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}
#define SHOOT_MOTOR_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	30.f,\
	0.5f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	1000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define SHOOT_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	50.0f,\
	0.5f,\
	0.0f,\
	0,\
	0,\
	0,\
	1000,\
	200,\
	100,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#endif


#ifndef FRAMEWORK_TASKS_CMCONTROL_H
#define FRAMEWORK_TASKS_CMCONTROL_H

#include "cmsis_os.h"

void CMControtLoopTaskInit(void);
void Timer_2ms_lTask(void const * argument);
void WorkStateFSM(void);
void WorkStateSwitchProcess(void);
void CMControlLoop(void);
void ShooterMControlLoop(void);
//initiate status: 
typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态 4s钟左右
    STANDBY_STATE,			//云台停止不转状态
    NORMAL_STATE,			//无输入状态
    STOP_STATE,        	//停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;


WorkState_e GetWorkState(void);

#define PID_SHOOT_MOTOR_SPEED      (30)
#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define PREPARE_TIME_TICK_MS 500      //prapare time in ms*2
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0.8f,\
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
//D参数原来为0.4
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


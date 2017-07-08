/**
  ******************************************************************************
  * File Name          : tasks_cmcontrol.c
  * Description        : 2ms定时任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 2ms定时
	* 通过count可以获得500ms,1s等定时任务
	* 状态机切换，串口定时输出，看门狗等
  ******************************************************************************
  */
#include <tim.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <cmsis_os.h>
#include <task.h>
#include <usart.h>
#include "tasks_timed.h"
#include "pid_Regulator.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "tasks_remotecontrol.h"
#include "application_motorcontrol.h"
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_task.h"
#include "peripheral_define.h"
#include "drivers_platemotor.h"


extern PID_Regulator_t CMRotatePID ; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;
PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;

Shoot_State_e last_shoot_state = NOSHOOTING;
Shoot_State_e this_shoot_state = NOSHOOTING;
//uint32_t last_Encoder = 0;
//uint32_t this_Encoder = 0;
int flag = 0;

WorkState_e workState = PREPARE_STATE;
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e GetWorkState()
{
	return workState;
}
/*2ms定时任务*/

extern float ZGyroModuleAngle;
float ZGyroModuleAngleMAX;
float ZGyroModuleAngleMIN;
extern float yawRealAngle;
extern uint8_t g_isGYRO_Rested;
extern float pitchRealAngle;
extern float gYroZs;
extern float g_yawAngleTarget;
extern float yawRealAngle;

extern uint8_t JUDGE_STATE;
//*********debug by ZY*********
typedef struct{
	uint16_t head;
	uint8_t id;
	uint8_t dlc;
	float dataPitch;
	float dataYaw;
	uint8_t checkSum;
}data_to_PC;


uint8_t data_send_to_PC[17];
data_to_PC my_data_to_PC;
void send_data_to_PC(UART_HandleTypeDef *huart,float zyPitch,float zyYaw,float zySpd)
{
//	my_data_to_PC.head=0xAAAA;
//	my_data_to_PC.id=0xF1;
//	my_data_to_PC.dlc=8;
//	my_data_to_PC.dataPitch=zyPitch;
//	my_data_to_PC.dataYaw=zyYaw;
//	
//	uint8_t * pTemp;
//	//uint8_t temp;
//	int i;
//	my_data_to_PC.checkSum=0;
//	pTemp = (uint8_t *)&(my_data_to_PC);  
//	for(i=0;i<12;i++)
//	{
//		 my_data_to_PC.checkSum+=pTemp[i];
//	}
	
	
	uint8_t * pTemp;
	int i;
	data_send_to_PC[0]=0xAA;
	data_send_to_PC[1]=0xAA;
	data_send_to_PC[2]=0xF1;
	data_send_to_PC[3]=12;
	pTemp=(uint8_t *)&zyPitch;
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[4+i]=pTemp[3-i];
	}
	
	pTemp=(uint8_t *)&zyYaw;
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[8+i]=pTemp[3-i];
	}
	
	pTemp=(uint8_t *)&zySpd;
	for(i=0;i<4;i++)
	{
		 data_send_to_PC[12+i]=pTemp[3-i];
	}
	
	data_send_to_PC[16]=0;
	for(i=0;i<16;i++)
	{
		 data_send_to_PC[16]+=data_send_to_PC[i];
	}
	
	HAL_UART_Transmit(huart,data_send_to_PC,17,1000);
}

//*********debug by ZY*********

int mouse_click_left = 0;
float this_fbspeed = 0;
float last_fbspeed = 0;
float diff_fbspeed = 0;

int stuck = 0;	//卡弹标志位，未卡弹为false，卡弹为true

extern uint8_t JUDGE_Received;
extern uint8_t JUDGE_State;

void Timer_2ms_lTask(void const * argument)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	static int s_countWhile = 0;
	static int countwhile1 = 0;
	static int countwhile2 = 0;
//	static int countwhile3 = 0;
	ShootMotorPositionPID.ref = 0x0;
	ShootMotorPositionPID.fdb = 0x0;
	static int count_judge = 0;
	//static int shootwhile = 0;
//	unsigned portBASE_TYPE StackResidue; //栈剩余
	while(1)  {       //motor control frequency 2ms
//监控任务
//		SuperviseTask();    
		WorkStateFSM();
	  WorkStateSwitchProcess();
//1s循环
		if(s_countWhile >= 1000){//定时 1S
		s_countWhile = 0;
			fw_printfln("ZGyroModuleAngle:  %f",ZGyroModuleAngle);
//			fw_printfln("YawAngle= %d", IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
//			fw_printfln("PitchAngle= %d", IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
//			fw_printfln("GMYawEncoder.ecd_angle:%f",GMYawEncoder.ecd_angle);
//			fw_printfln("PitAngle= %d", IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
//				fw_printfln("GMYAWEncoder.ecd_angle:%f",GMYawEncoder.ecd_angle );
//			fw_printfln("in CMcontrol_task");
//		StackResidue = uxTaskGetStackHighWaterMark( GMControlTaskHandle );
//		fw_printfln("GM%ld",StackResidue);
			if(JUDGE_State == 1){
				fw_printfln("Judge not received");
			}
			else{
				fw_printfln("Judge received");
			}
		}else{
			s_countWhile++;
		}
//陀螺仪复位计时
    if(countwhile1 > 3000){
			if(g_isGYRO_Rested == 0)GYRO_RST();//给单轴陀螺仪将当前位置写零
		}
		else{countwhile1++;}
		if(countwhile1 > 5500){
			g_isGYRO_Rested = 2;//正常遥控器或者键盘控制模式，底盘跟随模式
		}
		else{countwhile1++;}
//10ms循环
		if(countwhile2 >= 10){//定时 20MS
		countwhile2 = 0;
		this_fbspeed = (IOPool_pGetReadData(CMFLRxIOPool, 0)->RotateSpeed + IOPool_pGetReadData(CMFRRxIOPool, 0)->RotateSpeed 
			             + IOPool_pGetReadData(CMBLRxIOPool, 0)->RotateSpeed + IOPool_pGetReadData(CMBRRxIOPool, 0)->RotateSpeed)/4.f;
    diff_fbspeed = this_fbspeed - last_fbspeed;
		last_fbspeed = this_fbspeed;

//		send_data_to_PC(&DEBUG_UART,pitchRealAngle,ZGyroModuleAngle, gYroZs);//发送数据到上位机看波形
			//printf("pitch:%f *** yaw:%f",pitchRealAngle,ZGyroModuleAngle);
//		HAL_UART_Transmit(&DEBUG_UART,txbuf,strlen((char *)txbuf),1000);
		}else{
			countwhile2++;
		}

   if(JUDGE_Received==1){
			count_judge = 0;
		  JUDGE_State = 0;
		}
		else{
			count_judge++;
			if(count_judge > 150){//300ms
       JUDGE_State = 1;
			}
		}
		
		ShooterMControlLoop();       //发射机构控制任务
		
		vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_RATE_MS ) );
	}
}
	

	static uint32_t time_tick_2ms = 0;
void CMControtLoopTaskInit(void)
{
//上电计数初始化
	time_tick_2ms = 0;   
//底盘电机PID初始化，copy from官方开源程序
	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
/**********************************************************
*工作状态切换状态机
**********************************************************/

void WorkStateFSM(void)
{
	lastWorkState = workState;
	time_tick_2ms ++;
	switch(workState)
	{
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP )//|| Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if(time_tick_2ms > PREPARE_TIME_TICK_MS)
			{
				workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
			if(GetInputMode() == STOP )//|| Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if(GetShootState() != SHOOTING) //||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC
			{
			//	fw_printfln("进入STANDBY");
				workState = STANDBY_STATE;      
			}			
		}break;
		case STANDBY_STATE:  
		{
			if(GetInputMode() == STOP )//|| Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
			else if((GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)
			{
				workState = NORMAL_STATE;
			}				
		}break;
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP )//&& !Is_Serious_Error())
			{
				workState = PREPARE_STATE;   
			}
		}break;
		default:
		{
			
		}
	}	
}
void WorkStateSwitchProcess(void)
{
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		CMControtLoopTaskInit();
		RemoteTaskInit();
	}
}
<<<<<<< HEAD:Framework/applications/tasks_cmcontrol.c
//底盘控制任务 没用到
extern int16_t yawZeroAngle;
=======
>>>>>>> parent of 0810d8b... Revert "注释：failed":Framework/applications/tasks_timed.c

  
int32_t GetQuadEncoderDiff(void)
{
  int32_t cnt = 0;    
	cnt = __HAL_TIM_GET_COUNTER(&htim5) - 0x0;
	//fw_printfln("%x",cnt);
	 //__HAL_TIM_SET_COUNTER(&htim5, 0x7fff);
	return cnt;
}

int RotateAdd = 0;
int Stuck = 0;
int32_t last_fdb = 0x0;
int32_t this_fdb = 0;
void ShooterMControlLoop(void)	
{				      			
	if(GetShootState() == SHOOTING && GetInputMode()==KEY_MOUSE_INPUT && Stuck==0)
	{
		ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;//打一发弹编码器输出脉冲数
	}

	//遥控器输入模式下，只要处于发射态，就一直转动
	if(GetShootState() == SHOOTING && GetInputMode() == REMOTE_INPUT && Stuck==0)
	{
		RotateAdd += 8;
		fw_printfln("ref = %f",ShootMotorPositionPID.ref);
		if(RotateAdd>OneShoot)
		{
			ShootMotorPositionPID.ref = ShootMotorPositionPID.ref+OneShoot;
			RotateAdd = 0;
		}
	}
	else if(GetShootState() == NOSHOOTING && GetInputMode() == REMOTE_INPUT)
	{
		RotateAdd = 0;
	}

	if(GetFrictionState()==FRICTION_WHEEL_ON)//拨盘转动前提条件：摩擦轮转动
	{
		this_fdb = GetQuadEncoderDiff(); 
		fw_printfln("this_fdb = %d",this_fdb);
		if(this_fdb<last_fdb-100)
		{
			ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb+(65535+this_fdb-last_fdb);
		}
		else
			ShootMotorPositionPID.fdb = ShootMotorPositionPID.fdb + this_fdb-last_fdb;
		
		last_fdb = this_fdb;
		fw_printfln("fdb = %f",ShootMotorPositionPID.fdb);
		ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
		
		if(ShootMotorPositionPID.output<0) //反馈大于参考，需要反转
		{
			setPlateMotorDir(REVERSE);
			ShootMotorPositionPID.output = -ShootMotorPositionPID.output;
		}
		else
			setPlateMotorDir(FORWARD);
		
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ShootMotorPositionPID.output);
	}
	
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);//摩擦轮不转，立刻关闭拨盘
	}
}


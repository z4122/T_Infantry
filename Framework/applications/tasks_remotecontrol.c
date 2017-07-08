/**
  ******************************************************************************
  * File Name          : tasks_remotecontrol.c
  * Description        : 遥控器处理任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include <usart.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "tasks_remotecontrol.h"
#include "drivers_uartrc_user.h"
#include "drivers_uartrc_low.h"
#include "utilities_debug.h"
#include "drivers_ramp.h"
#include "pid_regulator.h"
#include "tasks_timed.h"
#include "peripheral_define.h"
#include "pwm_server_motor.h"
#include "tasks_motor.h"
#include "utilities_minmax.h"

//限制最大最小值 待删除
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\


extern ChassisSpeed_Ref_t g_ChassisSpeedRef;
extern FrictionWheelState_e g_friction_wheel_state;
static RemoteSwitch_t s_switch1;   //遥控器右上角拨杆

extern RampGen_t g_frictionRamp;  //摩擦轮斜坡
extern RampGen_t g_LRSpeedRamp;   //左右速度斜坡，用于键盘
extern RampGen_t g_FBSpeedRamp;   //前后速度

extern RC_Ctl_t g_RC_CtrlData;    //遥控器信息结构体
extern xSemaphoreHandle xSemaphore_rcuart;
extern float g_yawAngleTarget, g_pitchAngleTarget;
extern uint8_t g_isGYRO_Rested;      //陀螺仪复位信号，待删
extern int g_isTwist_Started ;          //扭腰信号,待处理
/*********************************************************
* 遥控器控制task
* 采用串口接收中断释放信号量的方式提高实时性(理论上)
* 任务流程：
* 1.进入阻塞态，等待接收中断释放信号量
* 2.判断两帧间隔，正常应为14ms，若大于16ms认为接收出现错误。扔掉第一帧
* 3.将IOPool中数据读入数组进行处理
* 4.扔掉DMA缓冲区中错误数据，重新开启接收中断
* 5.若判断为错误帧，等待2ms错误数据读取完毕后扔掉，重新开启接收中断
**********************************************************/
void RControlTask(void const * argument)
{
	uint8_t data[18];
	static int s_countWhile = 0;//用来对while计数，如每300次print一次数据，用来调试
  static TickType_t s_lastFrametime;//上一帧时间
	static TickType_t s_thisFrametime;//这一帧时间
	static bool s_bFirstframe = 0;//第一帧标志
	while(1){
		xSemaphoreTake(xSemaphore_rcuart, osWaitForever);//进入阻塞态，等待串口中断释放信号量
		s_thisFrametime = xTaskGetTickCount();//获得当前时间，自上电后计时，单位ms，rtos提供
		if( ((s_thisFrametime - s_lastFrametime) <= 100) && (s_bFirstframe == 1))//若两帧间隔小于16ms，且不是第一帧，则认为数据正确
		{

			if(IOPool_hasNextRead(rcUartIOPool, 0))
				{ 
				IOPool_getNextRead(rcUartIOPool, 0);
				uint8_t *pData = IOPool_pGetReadData(rcUartIOPool, 0)->ch;
				for(uint8_t i = 0; i != 18; ++i)
				{
					data[i] = pData[i]; //将IOPool数据读到数组中
				}

				ProcessRemoteData(data);	//处理数据并产生命令

				if(s_countWhile >= 300){
					s_countWhile = 0;
//					fw_printf("ch0 = %d | ", g_RC_CtrlData.rc.ch0);
//					fw_printf("ch1 = %d | ", g_RC_CtrlData.rc.ch1);
//					fw_printf("ch2 = %d | ", g_RC_CtrlData.rc.ch2);
//					fw_printf("ch3 = %d \r\n", g_RC_CtrlData.rc.ch3);
//					
//					fw_printf("s1 = %d | ", g_RC_CtrlData.rc.s1);
//					fw_printf("s2 = %d \r\n", g_RC_CtrlData.rc.s2);
//					
//					fw_printf("x = %d | ", g_RC_CtrlData.mouse.x);
//					fw_printf("y = %d | ", g_RC_CtrlData.mouse.y);
//					fw_printf("z = %d | ", g_RC_CtrlData.mouse.z);
//					fw_printf("l = %d | ", g_RC_CtrlData.mouse.press_l);
//					fw_printf("r = %d \r\n", g_RC_CtrlData.mouse.press_r);
//					
//					fw_printf("key = %d \r\n", g_RC_CtrlData.key.v);
//					fw_printf("===========\r\n");
				}else{
					s_countWhile++;
				}
	    }
				HAL_UART_AbortReceive(&RC_UART);//扔掉缓冲区多余数据
				HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);//重新开启接收中断
		}
		else{ //若认为数据错误，或是第一帧
		fw_printfln("RC discarded");
		s_bFirstframe = 1;
		vTaskDelay(2 / portTICK_RATE_MS);//延迟2ms后，rtos提供
		HAL_UART_AbortReceive(&RC_UART);
		HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);
		}
		s_lastFrametime = s_thisFrametime;
	}//end of while
}

void ProcessRemoteData(uint8_t *pData)
{
	if(pData == NULL)
	{
			return;
	}
//将数组读到结构体中
	g_RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	g_RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	g_RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
											 ((int16_t)pData[4] << 10)) & 0x07FF;
	g_RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	g_RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	g_RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

	g_RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	g_RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	g_RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

	g_RC_CtrlData.mouse.press_l = pData[12];
	g_RC_CtrlData.mouse.press_r = pData[13];

	g_RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);//16 bits correspond to 16 keys
	
//根据右上角拨杆选择状态：遥控，键盘鼠标，停止
//遥控/键鼠，就是处理不同部分数据，可以同时处理
	SetInputMode(&g_RC_CtrlData.rc);
	
	switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{
			if(g_isGYRO_Rested == 2)
			{ //if gyro has been reseted，待修改状态机
			SetEmergencyFlag(NORMAL);
			RemoteControlProcess(&(g_RC_CtrlData.rc));//execute new order，遥控处理
			}
		}break;
		case KEY_MOUSE_INPUT:
		{
			if(g_isGYRO_Rested == 2)
			{
			MouseKeyControlProcess(&g_RC_CtrlData.mouse,&g_RC_CtrlData.key);
			SetEmergencyFlag(NORMAL);
			SetShootMode(AUTO);//用来做瞄准待修改
		  }
		}break;
		case STOP:
		{
			SetEmergencyFlag(EMERGENCY);
		}break;
		default: 
			SetEmergencyFlag(EMERGENCY);
		break;
	}
}
//遥控模式
void RemoteControlProcess(Remote *rc)
{
//	if(GetWorkState()!=PREPARE_STATE)
//	{
		SetShootMode(MANUL);//射击模式 手动/自动(自瞄、大符)
		
		//修改底盘速度值
		g_ChassisSpeedRef.forward_back_ref = (g_RC_CtrlData.rc.ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
		g_ChassisSpeedRef.left_right_ref   = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT; 
	  
		//修改云台位置目标
		g_pitchAngleTarget += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
		//限制yaw转动速度
		MINMAX(rc->ch2, 480, 1520);
		g_yawAngleTarget   -= (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT; 
//	}

	RemoteShootControl(&s_switch1, rc->s1);
}


uint8_t fb_move_flag = 0;
uint8_t fb_move_flag1 = 0;

extern uint8_t JUDGE_State;

#ifndef Infantry_4
  #define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.025f * 2
  #define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.025f * 2
#else
  #define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.025f * 3
  #define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.025f * 3
#endif

void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	static uint16_t forward_back_speed = 0;
	static uint16_t left_right_speed = 0;
    if(GetWorkState()!=PREPARE_STATE)
    {
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 

			
        g_pitchAngleTarget -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
        g_yawAngleTarget    -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
		//speed mode: normal speed/high speed 
		if(key->v & 0x10)
		{
			forward_back_speed =  LOW_FORWARD_BACK_SPEED;
			left_right_speed = LOW_LEFT_RIGHT_SPEED;
		}
		else if(key->v & 0x20)
		{
			forward_back_speed =  MIDDLE_FORWARD_BACK_SPEED;
			left_right_speed = MIDDLE_LEFT_RIGHT_SPEED;
		}
		else
		{
			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
		//movement process
		static int last_fb_ref = 0;
		if(key->v & 0x01)  // key: w
		{
			g_ChassisSpeedRef.forward_back_ref = forward_back_speed* g_FBSpeedRamp.Calc(&g_FBSpeedRamp);
			g_isTwist_Started = 0;
		}
		else if(key->v & 0x02) //key: s
		{
			g_ChassisSpeedRef.forward_back_ref = -forward_back_speed* g_FBSpeedRamp.Calc(&g_FBSpeedRamp);
			g_isTwist_Started = 0;
		}
		else
		{
			g_ChassisSpeedRef.forward_back_ref = 0;
			g_FBSpeedRamp.ResetCounter(&g_FBSpeedRamp);//���ڼ�ͣ����
			
				if((last_fb_ref > 0) && (g_ChassisSpeedRef.forward_back_ref == 0)){
				fb_move_flag = 80;
			}
				if((last_fb_ref < 0) && (g_ChassisSpeedRef.forward_back_ref == 0)){
				fb_move_flag = 80;
			}
		}
	 	last_fb_ref = g_ChassisSpeedRef.forward_back_ref;
		
		if(key->v & 0x04)  // key: d
		{
			g_ChassisSpeedRef.left_right_ref = -left_right_speed* g_LRSpeedRamp.Calc(&g_LRSpeedRamp);
			g_isTwist_Started = 0;
		}
		else if(key->v & 0x08) //key: a
		{
			g_ChassisSpeedRef.left_right_ref = left_right_speed* g_LRSpeedRamp.Calc(&g_LRSpeedRamp);
			g_isTwist_Started = 0;
		}
		else
		{
			g_ChassisSpeedRef.left_right_ref = 0;
			g_LRSpeedRamp.ResetCounter(&g_LRSpeedRamp);
		}
		
		
	if(JUDGE_State==1){
	  if(abs(g_ChassisSpeedRef.forward_back_ref) + abs(g_ChassisSpeedRef.left_right_ref) > 500){
				if(g_ChassisSpeedRef.forward_back_ref > 325){
				 g_ChassisSpeedRef.forward_back_ref =  325 +  (g_ChassisSpeedRef.forward_back_ref - 325) * 0.15f;
				}
				else if(g_ChassisSpeedRef.forward_back_ref < -325){
					g_ChassisSpeedRef.forward_back_ref =  -325 +  (g_ChassisSpeedRef.forward_back_ref + 325) * 0.15f;
				}
				if(g_ChassisSpeedRef.left_right_ref > 300){
				 g_ChassisSpeedRef.left_right_ref =  300 +  (g_ChassisSpeedRef.left_right_ref - 300) * 0.15f;
				}
				else if(g_ChassisSpeedRef.left_right_ref < -300){
					g_ChassisSpeedRef.left_right_ref =  -300 +  (g_ChassisSpeedRef.left_right_ref + 300) * 0.15f;
				}
			}
//������ϼ�������
				if ((mouse->x < -2.6) || (mouse->x > 2.6)){
				if(abs(g_ChassisSpeedRef.forward_back_ref) + abs(g_ChassisSpeedRef.left_right_ref) > 400){
				if(g_ChassisSpeedRef.forward_back_ref > 250){
				 g_ChassisSpeedRef.forward_back_ref =  250 +  (g_ChassisSpeedRef.forward_back_ref - 250) * 0.15f;
				}
				else if(g_ChassisSpeedRef.forward_back_ref < -250){
					g_ChassisSpeedRef.forward_back_ref =  -250 +  (g_ChassisSpeedRef.forward_back_ref + 250) * 0.15f;
				}
				if(g_ChassisSpeedRef.left_right_ref > 250){
				 g_ChassisSpeedRef.left_right_ref =  250 +  (g_ChassisSpeedRef.left_right_ref - 250) * 0.15f;
				}
				else if(g_ChassisSpeedRef.left_right_ref < -250){
					g_ChassisSpeedRef.left_right_ref =  -250 +  (g_ChassisSpeedRef.left_right_ref + 250) * 0.15f;
				}
			}
			}
		}
//			if ((mouse->x < -1.8)){
//				mouse->x = -1.8f;
//			}
//			if ((mouse->x > 1.8)){
//				mouse->x = 1.8f;
//			}
		if(key->v == 8192)//c
		{
			if(GetSlabState() == CLOSE)
		{
#ifdef Infantry_4
				pwm_server_motor_set_angle(0,0.f);
#endif
#ifdef Infantry_3
				pwm_server_motor_set_angle(0,0.f);
#endif
#ifdef Infantry_2
				pwm_server_motor_set_angle(0,0.f);
#endif
				SetSlabState(OPEN);
			//	fw_printfln("OPEN");	
			}
//			else if(GetSlabState() == OPEN)
//			{
//				pwm_server_motor_set_angle(0,0.f);
//				SetSlabState(CLOSE);
//			}
		}
		if(key->v == 8208)//c+Shift
			{
			if(GetSlabState() == OPEN)
			{
#ifdef Infantry_4
				pwm_server_motor_set_angle(0,104.f);
#endif
#ifdef Infantry_3
				pwm_server_motor_set_angle(0,110.f);
#endif
#ifdef Infantry_2
				pwm_server_motor_set_angle(0,100.f);
#endif
				SetSlabState(CLOSE);
			//fw_printfln("CLOSE");	
			}
		}
			
		if(key->v == 256)  // key: r
		{
			g_isTwist_Started = 1;
		}
		if(key->v == 272)  // key: r+Shift
		{
			g_isTwist_Started = 0;
		}
		
		
	
	//step2: gimbal ref calc
 /*   if(GetWorkState() == NORMAL_STATE)
    {
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
		
        g_pitchAngleTarget -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
        g_yawAngleTarget    -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;

	}
	*/
	/* not used to control, just as a flag */ 
    GimbalRef.pitch_speed_ref = mouse->y;    
    GimbalRef.yaw_speed_ref   = mouse->x;
	  MouseShootControl(mouse);
	}
	
}






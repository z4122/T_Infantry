/**
  ******************************************************************************
  * File Name          : drivers_uartrc.c
  * Description        : 遥控器串口
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 串口初始化
	* 串口数据读取
	* 数据处理函数
  ******************************************************************************
  */
#include "drivers_uartrc_user.h"
#include "drivers_uartrc_low.h"
#include "drivers_led_user.h"
#include "peripheral_laser.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "rtos_semaphore.h"
#include "drivers_ramp.h"
#include "peripheral_tim.h"
#include <stdlib.h>
#include <math.h>
#include "utilities_debug.h"
#include  "tim.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "peripheral_define.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartupper_user.h"
#include "stm32f4xx_hal_uart.h"
NaiveIOPoolDefine(rcUartIOPool, {0});

void InitRemoteControl(){
	//遥控器DMA接收开启(一次接收18个字节)
	if(HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18) != HAL_OK){
			Error_Handler();
	} 
//	__HAL_UART_ENABLE_IT(&RC_UART, UART_FLAG_IDLE);
	RemoteTaskInit();
}

void rcUartRxCpltCallback(){
	static portBASE_TYPE xHigherPriorityTaskWoken;
	 xHigherPriorityTaskWoken = pdFALSE; 
	 IOPool_getNextWrite(rcUartIOPool);
   xSemaphoreGiveFromISR(xSemaphore_rcuart, &xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken == pdTRUE ){
   portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	 }
 }



RC_Ctl_t g_RC_CtrlData;   //remote control data
ChassisSpeed_Ref_t g_ChassisSpeedRef; 
Gimbal_Ref_t GimbalRef; 
FrictionWheelState_e g_friction_wheel_state = FRICTION_WHEEL_OFF; 

volatile Shoot_State_e shootState = NOSHOOTING; 
InputMode_e g_eInputmode = REMOTE_INPUT;   

RampGen_t g_frictionRamp = RAMP_GEN_DAFAULT;  
RampGen_t g_LRSpeedRamp = RAMP_GEN_DAFAULT;   
RampGen_t g_FBSpeedRamp = RAMP_GEN_DAFAULT;   

void RemoteTaskInit()
{
	//斜坡函数初始化
	g_frictionRamp.SetScale(&g_frictionRamp, FRICTION_RAMP_TICK_COUNT);
	g_LRSpeedRamp.SetScale(&g_LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	g_FBSpeedRamp.SetScale(&g_FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	g_frictionRamp.ResetCounter(&g_frictionRamp);
	g_LRSpeedRamp.ResetCounter(&g_LRSpeedRamp);
	g_FBSpeedRamp.ResetCounter(&g_FBSpeedRamp);
  //速度初始化
	g_ChassisSpeedRef.forward_back_ref = 0.0f;
	g_ChassisSpeedRef.left_right_ref = 0.0f;
	g_ChassisSpeedRef.rotate_ref = 0.0f;
  //摩擦轮状态初始化
	SetFrictionState(FRICTION_WHEEL_OFF);
}
/*拨杆数据处理*/
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t s_switch_cnt = 0;

	/* 最新状态值 */
	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	/* 取最新值和上一次值 */
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);


	/* 最老的状态值索引 */
	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* 合并三个值 */
	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	/* 长按判断 */
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		s_switch_cnt++;	
	}
	else
	{
		s_switch_cnt = 0;
	}

	if(s_switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}

	//索引循环
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}
/*取得右上角拨杆数据*/
void SetInputMode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		g_eInputmode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		g_eInputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		g_eInputmode = STOP;
	}	
}

InputMode_e GetInputMode()
{
	return g_eInputmode;
}

/*
input: RemoteSwitch_t *sw, include the switch info
*/
#ifdef Infantry_4
#define FRICTION_WHEEL_MAX_DUTY             1500
#endif
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	//左上角拨杆状态，可以检测位置、动作
	GetRemoteSwitchAction(sw, val);
	switch(g_friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   
			{
				SetShootState(NOSHOOTING);
				g_frictionRamp.ResetCounter(&g_frictionRamp);
				g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				LASER_OFF();
				SetShootState(NOSHOOTING);
				SetFrictionWheelSpeed(1000);
				g_friction_wheel_state = FRICTION_WHEEL_OFF;
				g_frictionRamp.ResetCounter(&g_frictionRamp);
			}
			else
			{
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*g_frictionRamp.Calc(&g_frictionRamp)); 
				if(g_frictionRamp.IsOverflow(&g_frictionRamp))
				{
					g_friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				LASER_OFF();
				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				g_frictionRamp.ResetCounter(&g_frictionRamp);
				SetShootState(NOSHOOTING);
			}
			else if(sw->switch_value_raw == 2)
			{
				SetShootState(SHOOTING);
			}
			else
			{
				SetShootState(NOSHOOTING);
			}					 
		} break;				
	}
}
	 
void MouseShootControl(Mouse *mouse)
{
	static int16_t closeDelayCount = 0;   
	switch(g_friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(mouse->last_press_r == 0 && mouse->press_r == 1)   
			{
				SetShootState(NOSHOOTING);
				g_frictionRamp.ResetCounter(&g_frictionRamp);
				g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
				closeDelayCount = 0;
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>50)   
			{
				LASER_OFF();
				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				g_frictionRamp.ResetCounter(&g_frictionRamp);
				SetShootState(NOSHOOTING);
			}
			else
			{
				//				
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*g_frictionRamp.Calc(&g_frictionRamp)); 
				if(g_frictionRamp.IsOverflow(&g_frictionRamp))
				{
					g_friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			
			if(closeDelayCount>50)   //
			{
				LASER_OFF();
				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				g_frictionRamp.ResetCounter(&g_frictionRamp);
				SetShootState(NOSHOOTING);
			}			
			else if(mouse->last_press_l == 0 && mouse->press_l== 1)  //检测鼠标左键单击动作
			{
				SetShootState(SHOOTING);				
			}
			else
			{
				SetShootState(NOSHOOTING);				
			}					 
		} break;				
	}	
	mouse->last_press_r = mouse->press_r;
	mouse->last_press_l = mouse->press_l;
}






Shoot_State_e GetShootState()
{
	return shootState;
}

void SetShootState(Shoot_State_e v)
{
	shootState = v;
}

FrictionWheelState_e GetFrictionState()
{
	return g_friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
	g_friction_wheel_state = v;
}
void SetFrictionWheelSpeed(uint16_t x)
{
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_1, x);
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_2, x);
}
Shoot_Mode_e shootMode = MANUL;

Shoot_Mode_e GetShootMode()
{
	return shootMode;
}
void SetShootMode(Shoot_Mode_e v)
{
	shootMode = v;
}

Emergency_Flag emergency_Flag = NORMAL;

Emergency_Flag GetEmergencyFlag()
{
	return emergency_Flag;
}

void SetEmergencyFlag(Emergency_Flag v)
{
	emergency_Flag = v;
}

Move_Speed_e movespeed = NORMAL_s;

Move_Speed_e GetMoveSpeed()
{
	return movespeed;
}

void SetMoveSpeed(Move_Speed_e v)
{
	movespeed = v;
}

Slab_Mode_e slabmode = CLOSE;

Slab_Mode_e GetSlabState()
{
	return slabmode;
}

void SetSlabState(Slab_Mode_e v)
{
	slabmode = v;
}

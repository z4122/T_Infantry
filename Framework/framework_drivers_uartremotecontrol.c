#include "framework_drivers_uartremotecontrol_iopool.h"
#include "framework_drivers_uartremotecontrol_task.h"
#include "framework_drivers_uartremotecontrol.h"
#include "framework_drivers_led.h"
#include "usart.h"
#include "ramp.h"
#include "tim.h"
#include "framework_tasks_cmcontrol.h"

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)
/*****Begin define ioPool*****/
#define DataPoolInit {0}
#define ReadPoolSize 1
#define ReadPoolMap {0}
#define GetIdFunc 0 
#define ReadPoolInit {0, Empty, 1}

IOPoolDefine(rcUartIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);
	
#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/
void rcInit(){
	//遥控器DMA接收开启(一次接收18个字节)
	if(HAL_UART_Receive_DMA(&rcUart, IOPool_pGetWriteData(rcUartIOPool)->ch, 18) != HAL_OK){
			Error_Handler();
	} 
}
extern xSemaphoreHandle xSemaphore_rcuart;
/*遥控器串口回掉函数*/
void rcUartRxCpltCallback(){
	static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE; //givefromisr 要求变量
	xSemaphoreGiveFromISR(xSemaphore_rcuart, &xHigherPriorityTaskWoken);
	//TODO context switch
	IOPool_getNextWrite(rcUartIOPool);
	HAL_UART_Receive_DMA(&rcUart, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);
//上下文切换
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

RC_Ctl_t RC_CtrlData;   //remote control data
ChassisSpeed_Ref_t ChassisSpeedRef; //底盘电机目标速度
Gimbal_Ref_t GimbalRef; //云台目标
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF; //摩擦轮状态

static volatile Shoot_State_e shootState = NOSHOOTING; //拨盘电机状态
static InputMode_e inputmode = REMOTE_INPUT;   //输入模式设定

RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  //摩擦轮斜坡
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   //mouse左右移动斜坡
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   //mouse前后移动斜坡

/*遥控器左上角*/
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	/* 最新状态值 */
	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	/* 取最新值和上一次值 */
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);


	/* 最老的状态值的索引 */
	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* 合并三个值 */
	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	/* 长按判断 */
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}

	if(switch_cnt >= 40)
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
//return the state of the remote 0:no action 1:action 
uint8_t IsRemoteBeingAction(void)
{
	return (abs(ChassisSpeedRef.forward_back_ref)>=10 || abs(ChassisSpeedRef.left_right_ref)>=10 || fabs(GimbalRef.yaw_speed_ref)>=10 || fabs(GimbalRef.pitch_speed_ref)>=10);
}
/*遥控器右上角*/
void SetInputMode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		inputmode = STOP;
	}	
}

InputMode_e GetInputMode()
{
	return inputmode;
}

/*
input: RemoteSwitch_t *sw, include the switch info
*/

void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	GetRemoteSwitchAction(sw, val);
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //从关闭到start turning
			{
				SetShootState(NOSHOOTING);
				frictionRamp.ResetCounter(&frictionRamp);
				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
	//			LASER_ON(); 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //刚启动就被关闭
			{
//				LASER_OFF();
				SetShootState(NOSHOOTING);
				SetFrictionWheelSpeed(1000);
				friction_wheel_state = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else
			{
				//摩擦轮加速
				
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //关闭摩擦轮
			{
//				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
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
	int16_t closeDelayCount = 0;   //右键关闭摩擦轮3s延时计数
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(mouse->last_press_r == 0 && mouse->press_r == 1)   //从关闭到start turning
			{
				SetShootState(NOSHOOTING);
				frictionRamp.ResetCounter(&frictionRamp);
				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
//				LASER_ON(); 
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
			if(closeDelayCount>50)   //关闭摩擦轮
			{
//				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
			}
			else
			{
				//摩擦轮加速				
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					friction_wheel_state = FRICTION_WHEEL_ON; 	
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
			
			if(closeDelayCount>50)   //关闭摩擦轮
			{
//				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NOSHOOTING);
			}			
			else if(mouse->press_l== 1)  //按下左键，射击
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
	return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
	friction_wheel_state = v;
}
void SetFrictionWheelSpeed(uint16_t x)
{
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, x);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, x);
}
//遥控器输入值限制
void GimbalAngleLimit()
{
//	MINMAX(ChassisSpeedRef.forward_back_ref, -PITCH_MAX+7, PITCH_MAX);
//	MINMAX(GimbalRef.yaw_angle_dynamic_ref, GMYPositionPID.fdb - 60, GMYPositionPID.fdb + 60);
}



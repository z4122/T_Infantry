#include "framework_tasks_remotecontrol.h"
#include "framework_drivers_uartremotecontrol.h"
#include "framework_drivers_uartremotecontrol_iopool.h"
#include "framework_drivers_led.h"
#include "stdint.h"
#include "stddef.h"
#include "ramp.h"
#include "ControlTask.h"

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern Gimbal_Ref_t GimbalRef;
extern FrictionWheelState_e friction_wheel_state ;
static RemoteSwitch_t switch1;   //遥控器左侧拨杆
static volatile Shoot_State_e shootState = NOSHOOTING;
static InputMode_e inputmode = REMOTE_INPUT;   //输入模式设定

extern RampGen_t frictionRamp ;  //摩擦轮斜坡
extern RampGen_t LRSpeedRamp ;   //mouse左右移动斜坡
extern RampGen_t FBSpeedRamp  ;   //mouse前后移动斜坡

extern RC_Ctl_t RC_CtrlData; 
extern xSemaphoreHandle xSemaphore_rcuart;
void printRcTask(void const * argument){
	uint8_t data[18];
	while(1){
		xSemaphoreTake(xSemaphore_rcuart, osWaitForever);
		if(IOPool_hasNextRead(rcUartIOPool, 0)){
			IOPool_getNextRead(rcUartIOPool, 0);
			
			uint8_t *pData = IOPool_pGetReadData(rcUartIOPool, 0)->ch;
			for(uint8_t i = 0; i != 18; ++i){
				data[i] = pData[i];
			}
		
			RemoteDataProcess(data);
			if(RC_CtrlData.rc.s1 == 1){
				ledGStatus = on;
			}else if(RC_CtrlData.rc.s1 == 2){
				ledGStatus = blink;
			}else{
				ledGStatus = off;
			}
			if(RC_CtrlData.rc.s2 == 1){
				ledRStatus = on;
			}else if(RC_CtrlData.rc.s2 == 2){
				ledRStatus = blink;
			}else{
				ledRStatus = off;
			}
			
			
//			printf("ch0 = %d | ", RC_CtrlData.rc.ch0);
//			printf("ch1 = %d | ", RC_CtrlData.rc.ch1);
//			printf("ch2 = %d | ", RC_CtrlData.rc.ch2);
//			printf("ch3 = %d \r\n", RC_CtrlData.rc.ch3);
//			
//			printf("s1 = %d | ", RC_CtrlData.rc.s1);
//			printf("s2 = %d \r\n", RC_CtrlData.rc.s2);
//			
//			printf("x = %d | ", RC_CtrlData.mouse.x);
//			printf("y = %d | ", RC_CtrlData.mouse.y);
//			printf("z = %d | ", RC_CtrlData.mouse.z);
//			printf("l = %d | ", RC_CtrlData.mouse.press_l);
//			printf("r = %d \r\n", RC_CtrlData.mouse.press_r);
//			
//			printf("key = %d \r\n", RC_CtrlData.key.v);
//			printf("===========\r\n");

		}
	}
}

void RemoteTaskInit()
{
	//斜坡初始化
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	frictionRamp.ResetCounter(&frictionRamp);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	//底盘云台给定值初始化
	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	//摩擦轮运行状态初始化
	SetFrictionState(FRICTION_WHEEL_OFF);
}


void RemoteDataProcess(uint8_t *pData)
{
    if(pData == NULL)
    {
        return;
    }
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                         ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];
 
    RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
		
		SetInputMode(&RC_CtrlData.rc);
	
	//RemoteControlProcess(&(RC_CtrlData.rc));
	
	switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{
			//遥控器控制模式
			RemoteControlProcess(&(RC_CtrlData.rc));
		}break;
		case KEY_MOUSE_INPUT:
		{
			//遥控器控制模式
			MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
		}break;
		case STOP:
		{
			//紧急停车
		}break;
	}
}
//遥控器控制模式处理
void RemoteControlProcess(Remote *rc)
{
    if(GetWorkState()!=PREPARE_STATE)
    {
        ChassisSpeedRef.forward_back_ref = (RC_CtrlData.rc.ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
        ChassisSpeedRef.left_right_ref   = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT; 
    }

    if(GetWorkState() == NORMAL_STATE)
    {
        GimbalRef.pitch_angle_dynamic_ref += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
        GimbalRef.yaw_angle_dynamic_ref   += (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;      	
	}
	
	/* not used to control, just as a flag */ 
    GimbalRef.pitch_speed_ref = rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;    //speed_ref仅做输入量判断用
    GimbalRef.yaw_speed_ref   = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	GimbalAngleLimit();
	//遥控器拨杆数据处理	
	RemoteShootControl(&switch1, rc->s1);
		

}
//键盘鼠标控制模式处理
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	static uint16_t forward_back_speed = 0;
	static uint16_t left_right_speed = 0;
    if(GetWorkState()!=PREPARE_STATE)
    {
		//speed mode: normal speed/high speed
		if(key->v & 0x10)
		{
			forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
			left_right_speed = HIGH_LEFT_RIGHT_SPEED;
		}
		else
		{
			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
		//movement process
		if(key->v & 0x01)  // key: w
		{
			ChassisSpeedRef.forward_back_ref = forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
		}
		else if(key->v & 0x02) //key: s
		{
			ChassisSpeedRef.forward_back_ref = -forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
		}
		else
		{
			ChassisSpeedRef.forward_back_ref = 0;
			FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		}
		
		
		if(key->v & 0x04)  // key: d
		{
			ChassisSpeedRef.left_right_ref = -left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
		}
		else if(key->v & 0x08) //key: a
		{
			ChassisSpeedRef.left_right_ref = left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
		}
		else
		{
			ChassisSpeedRef.left_right_ref = 0;
			LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		}
	}
	//step2: gimbal ref calc
    if(GetWorkState() == NORMAL_STATE)
    {
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
		
        GimbalRef.pitch_angle_dynamic_ref -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
        GimbalRef.yaw_angle_dynamic_ref   += mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;

	}
	
	/* not used to control, just as a flag */ 
    GimbalRef.pitch_speed_ref = mouse->y;    //speed_ref仅做输入量判断用
    GimbalRef.yaw_speed_ref   = mouse->x;
	GimbalAngleLimit();	
	MouseShootControl(mouse);
	
}




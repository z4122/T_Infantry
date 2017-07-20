/**
  ******************************************************************************
  * File Name          : drivers_uartupper.c
  * Description        : 妙算通信
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 串口初始化
	* 接收回调函数
	* 自定义组帧协议
  ******************************************************************************
  */
#include "drivers_uartupper_low.h"
#include "drivers_uartupper_user.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "stdint.h"
#include "peripheral_define.h"
#include "utilities_debug.h"
#include "usart.h"
#include "rtos_semaphore.h"
#include "tasks_platemotor.h"

#include "tasks_timed.h"//zy

NaiveIOPoolDefine(ctrlUartIOPool, {0});

extern PID_Regulator_t ShootMotorPositionPID;

xdata_ctrlUart ctrlData; 

float zyYawAdd,zyPitchAdd,zyYawTarget,zyPitchTarget;
extern float yawAngleTarget, pitchAngleTarget;
Location_Number_s Location_Number[9] = {{0, 0},{0, 0},{0, 0},{0, 0},{0, 0},{0, 0},{0, 0},{0, 0}};
extern float yawRealAngle;
extern float pitchRealAngle;//张雁大符相关
uint8_t runeLocation = 4;

void manifoldUartRxCpltCallback()
{
	//ShootOneBullet();
	
	static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

	zyYawAdd=0;
	zyYawTarget=0;
	uint8_t *pData = &runeLocation;
	
	if(GetWorkState()==RUNE_STATE)
	{
		int temp=*pData;
		yawAngleTarget = Location_Number[temp].yaw_position;
		pitchAngleTarget = Location_Number[temp].pitch_position;
		fw_printfln("manifold callback:%x,",*pData);
		fw_printfln("manifold callback:pitchTarget,%f,yawTarget:%f",pitchAngleTarget,yawAngleTarget);
		
		ShootOneBullet();//拨盘啵一个
  }
	else
	{
		fw_printfln("manifold callback:%x",*pData);
	}

	HAL_UART_AbortReceive((&MANIFOLD_UART));
	if(HAL_UART_Receive_DMA(&MANIFOLD_UART, &runeLocation, 1) != HAL_OK)
	{
		Error_Handler();
		printf( "ManifoldUart error" );
	} 
	if( xHigherPriorityTaskWoken == pdTRUE )
	{
	 portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}


void InitManifoldUart()
{
	ctrlData.Success = 1;  
	//vRefreshLocation(0, 0);
	zyLocationInit(1.0,8.0);//1号-2.0,6.1
	if(HAL_UART_Receive_DMA(&MANIFOLD_UART, &runeLocation, 1) != HAL_OK){
		Error_Handler();
		printf( "InitManifoldUart error" );
	} 
}


void vInsert( uint8_t a[ ], uint8_t i, uint8_t n, uint8_t number){
    for (int j=n;j>i;j--){
        a[j]=a[j-1];
        }
        a[i]=number;
    if (i==n)
        a[i]=number;
}

void vCheck( uint8_t a[] ){
	for(uint8_t i = 1; i <= size_frame - 5; i++)
	{switch ( a[i] ){
		case byte_SOF    : vInsert( a, i, size_frame, byte_ESCAPE);
		                 a[i+1] = 0X00; 
		//fw_printfln("inchange");
		break;
		case byte_EOF    : vInsert( a, i, size_frame, byte_ESCAPE);
		                 a[i+1] = 0X01; //fw_printfln("inchange2");
		break;		
		case byte_ESCAPE : vInsert( a, i, size_frame, byte_ESCAPE);
		                 a[i+1] = 0X02; //fw_printfln("inchange3");
		break;
	}}
}

void vSendUart(xdata_ctrlUart data){
	uint8_t tempdata[size_frame] = {0};
	tempdata[0] = byte_SOF;
	tempdata[1] = data.dev_yaw >> 8;
	tempdata[2] = data.dev_yaw & 0x00ff;
  tempdata[3] = data.dev_pitch >> 8;
	tempdata[4] = data.dev_pitch & 0x00ff;	
	tempdata[5] = data.rune;
	tempdata[6] = data.rune_locate;
	tempdata[7] = data.target_dis >> 8;
	tempdata[8] = data.target_dis & 0x00ff;
	tempdata[9] = data.DLC;
	tempdata[10] = byte_EOF;
  vCheck( tempdata );
	for( uint8_t i = 0; i <= size_frame - 1; i++){
		fw_printf("%x",tempdata[i]);
	}
	printf("\r\n");
}

void vDeleteEscape(uint8_t *pData, uint8_t a){
   while ( a <= size_frame - 2 ){
	 *(pData + a) = *(pData + a + 1);
		 a ++;
	 }
}

void vCheckEscape(uint8_t *pData){
	uint8_t a = 1;
	while( *pData != byte_EOF && a <= size_frame -1){
  if( *(pData + a) == byte_ESCAPE ) {
	//	fw_printfln("in escapecheck");
		switch ( *(pData + a + 1) ){
			case 0x00 : *(pData + a + 1) = byte_SOF; break; 
			case 0x01 : *(pData + a + 1) = byte_EOF; break;
			case 0x02 : *(pData + a + 1) = byte_ESCAPE; break;
		}
		vDeleteEscape( pData, a );
	}
	a++;
  }
}

xdata_ctrlUart xUartprocess(uint8_t *pData){
	xdata_ctrlUart To_return;
	To_return.Success = 0;
	uint8_t a = 0; 
		if ( *pData != byte_SOF ) {
      To_return.Success = 0;
			return To_return;
		}
		vCheckEscape( pData );
		for(; a <= size_frame - 1; a++){
			if(*(pData + a) == byte_EOF) break;
		}
		if( *(pData + a ) != byte_EOF || *(pData + a - 1) != a - 2) {
    To_return.Success = 0; 
    return To_return;
		}
		else To_return.Success = 1;
		To_return.dev_yaw    = (*(pData + 1) << 8) + *(pData + 2);
		To_return.dev_pitch  = (*(pData + 3) << 8) + *(pData + 4);
		To_return.rune       = *(pData + 5);
		To_return.rune_locate= *(pData + 6);
		To_return.target_dis = (*(pData + 7) << 8) + *(pData + 8);
		To_return.DLC = *(pData + a - 1);
		return To_return;
}

Locate_State_e LocateState = Locating;

void SetLocateState(Locate_State_e v){
	LocateState = v;
}
Locate_State_e GetLocateState(void){
	return LocateState;
}
Rune_State_e RuneState = WAITING;

void SetRuneState(Rune_State_e v){
	RuneState = v;
}
Rune_State_e GetRuneState(void){
	return RuneState;
}

float dis_yaw = 9.7;
float dis_pitch = 15;//5.33;
float location_center_yaw = 0;
float location_center_pitch = 0;

//Location_Number_s Location_Number[9] = {{0, 0},{0, 0},{0, 0},{0, 0},{0, 0},{0, 0},{0, 0},{0, 0}};

void vRefreshLocation(float yaw_center, float pitch_center){
	Location_Number[0].yaw_position = yaw_center + dis_yaw;
	Location_Number[0].pitch_position = pitch_center + dis_pitch;
	Location_Number[1].yaw_position = yaw_center;
	Location_Number[1].pitch_position = pitch_center + dis_pitch;
	Location_Number[2].yaw_position = yaw_center - dis_yaw;
	Location_Number[2].pitch_position = pitch_center + dis_pitch;
	Location_Number[3].yaw_position = yaw_center + dis_yaw;
	Location_Number[3].pitch_position = pitch_center;
	Location_Number[4].yaw_position = yaw_center;
	Location_Number[4].pitch_position = pitch_center;
	Location_Number[5].yaw_position = yaw_center - dis_yaw;
	Location_Number[5].pitch_position = pitch_center;
	Location_Number[6].yaw_position = yaw_center + dis_yaw;
	Location_Number[6].pitch_position = pitch_center - dis_pitch;
	Location_Number[7].yaw_position = yaw_center;
	Location_Number[7].pitch_position = pitch_center - dis_pitch;
  Location_Number[8].yaw_position = yaw_center - dis_yaw;
	Location_Number[8].pitch_position = pitch_center - dis_pitch;
}

float pAddZy=7.54,pMinusZy=4.67,yAddZy=9.5,yMinusZy=10.8;
void zyLocationInit(float yaw_center,float pitch_center)
{
	pitch_center = pitch_center - 1.0f;
	Location_Number[0].yaw_position = yaw_center + yAddZy;
	Location_Number[0].pitch_position = pitch_center + pAddZy;
	Location_Number[1].yaw_position = yaw_center;
	Location_Number[1].pitch_position = pitch_center + pAddZy;
	Location_Number[2].yaw_position = yaw_center - yMinusZy;
	Location_Number[2].pitch_position = pitch_center + pAddZy;
	Location_Number[3].yaw_position = yaw_center + yAddZy;
	Location_Number[3].pitch_position = pitch_center;
	Location_Number[4].yaw_position = yaw_center;
	Location_Number[4].pitch_position = pitch_center;
	Location_Number[5].yaw_position = yaw_center - yMinusZy;
	Location_Number[5].pitch_position = pitch_center;
	Location_Number[6].yaw_position = yaw_center + yAddZy;
	Location_Number[6].pitch_position = pitch_center - pMinusZy;
	Location_Number[7].yaw_position = yaw_center;
	Location_Number[7].pitch_position = pitch_center - pMinusZy;
  Location_Number[8].yaw_position = yaw_center - yMinusZy;
	Location_Number[8].pitch_position = pitch_center - pMinusZy;
}

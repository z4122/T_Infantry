/**
  ******************************************************************************
  * File Name          : drivers_canmotor.c
  * Description        : ÁîµÊú∫CANÊÄªÁ∫øÈ©±Âä®ÂáΩÊï∞
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * CANÊÄªÁ∫øÂàùÂßãÂåñ
	* CANÊé•Êî∂Â§ÑÁêÜ
	* CANÂèëÈÄÅ‰ªªÂä°
  ******************************************************************************
  */
#include <cmsis_os.h>
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "peripheral_define.h"
#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "rtos_init.h"
#include "rtos_semaphore.h"

volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
float ZGyroModuleAngle = 0.0f;

//RxIOPool
NaiveIOPoolDefine(CMFLRxIOPool, {0});
NaiveIOPoolDefine(CMFRRxIOPool, {0});
NaiveIOPoolDefine(CMBLRxIOPool, {0});
NaiveIOPoolDefine(CMBRRxIOPool, {0});

NaiveIOPoolDefine(GMPITCHRxIOPool, {0});
NaiveIOPoolDefine(GMYAWRxIOPool, {0});

//TxIOPool
#define DataPoolInit \
	{ \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{CM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(CMTxIOPool, DataPoolInit);
#undef DataPoolInit 
	
#define DataPoolInit \
	{ \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{GM_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(GMTxIOPool, DataPoolInit);
#undef DataPoolInit 

#define DataPoolInit \
	{ \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{ZGYRO_TXID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
NaiveIOPoolDefine(ZGYROTxIOPool, DataPoolInit);
#undef DataPoolInit 

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])

uint8_t isRcanStarted_CMGM = 0, isRcanStarted_ZGYRO = 0;

CanRxMsgTypeDef CMGMCanRxMsg, ZGYROCanRxMsg;
	
void InitCanReception()
{
	CMGMMOTOR_CAN.pRxMsg = &CMGMCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&CMGMMOTOR_CAN, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_CMGM = 1;
	
	ZGYRO_CAN.pRxMsg = &ZGYROCanRxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 14;//14 - 27//14
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = 0;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.BankNumber = 14;
  HAL_CAN_ConfigFilter(&ZGYRO_CAN, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&ZGYRO_CAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_ZGYRO = 1;
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &CMGMMOTOR_CAN){
		switch(CMGMCanRxMsg.StdId){
			case CMFL_RXID:
				IOPool_pGetWriteData(CMFLRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMFLRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_getNextWrite(CMFLRxIOPool);
				break;
			case CMFR_RXID:
				IOPool_pGetWriteData(CMFRRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMFRRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_getNextWrite(CMFRRxIOPool);
				break;
			case CMBL_RXID:
				IOPool_pGetWriteData(CMBLRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMBLRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_getNextWrite(CMBLRxIOPool);
				break;
			case CMBR_RXID:
				IOPool_pGetWriteData(CMBRRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(CMBRRxIOPool)->RotateSpeed = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_getNextWrite(CMBRRxIOPool);
				break;
			case GMYAW_RXID:
				IOPool_pGetWriteData(GMYAWRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(GMYAWRxIOPool)->realIntensity = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_pGetWriteData(GMYAWRxIOPool)->giveIntensity = CanRxGetU16(CMGMCanRxMsg, 2);
				IOPool_getNextWrite(GMYAWRxIOPool);
				break;
			case GMPITCH_RXID:
				IOPool_pGetWriteData(GMPITCHRxIOPool)->angle = CanRxGetU16(CMGMCanRxMsg, 0);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->realIntensity = CanRxGetU16(CMGMCanRxMsg, 1);
				IOPool_pGetWriteData(GMPITCHRxIOPool)->giveIntensity = CanRxGetU16(CMGMCanRxMsg, 2);
				IOPool_getNextWrite(GMPITCHRxIOPool);
				break;
			default:
			fw_Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
			fw_Warning();
			isRcanStarted_CMGM = 0;
		}else{
			isRcanStarted_CMGM = 1;
		}
		if(g_bInited == 1){
			osSemaphoreRelease(CMGMCanRefreshSemaphoreHandle);
		}
	}
	else if(hcan == &ZGYRO_CAN)
	{
		switch(ZGYROCanRxMsg.StdId)
		{
			case ZGYRO_RXID:
			 {
				CanRxMsgTypeDef *msg = &ZGYROCanRxMsg;
				ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 
			 }
			 break;
			default:
			fw_Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&ZGYRO_CAN, CAN_FIFO0) != HAL_OK)
		{
			//fw_Warning();
			isRcanStarted_ZGYRO = 0;
		}else{
			isRcanStarted_ZGYRO = 1;
		}
		if(g_bInited == 1){
			osSemaphoreRelease(ZGYROCanRefreshSemaphoreHandle);
		}
	}
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan == &CMGMMOTOR_CAN){
		osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
	}
	else if(hcan == &ZGYRO_CAN)
	{
		osSemaphoreRelease(ZGYROCanTransmitSemaphoreHandle);
	}
}

void TransmitCMGMCan(void)
{
		if(IOPool_hasNextRead(CMTxIOPool, 0))
		{
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(CMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(CMTxIOPool, 0);
			
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
		
		if(IOPool_hasNextRead(GMTxIOPool, 0))
		{
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			
			IOPool_getNextRead(GMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(GMTxIOPool, 0);
			
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
		}
}

void TransmitGYROCAN(void){
	if(IOPool_hasNextRead(ZGYROTxIOPool, 0))
	{
			osSemaphoreWait(ZGYROCanTransmitSemaphoreHandle, osWaitForever);
		
			IOPool_getNextRead(ZGYROTxIOPool, 0);
			ZGYRO_CAN.pTxMsg = IOPool_pGetReadData(ZGYROTxIOPool, 0);
		
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&ZGYRO_CAN) != HAL_OK)
			{
				fw_Warning();
			}
			taskEXIT_CRITICAL();
	}
}




/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/
void EncoderProcess(volatile Encoder *v, Motor820RRxMsg_t * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = msg->angle;
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -7500)    //¡Ω¥Œ±‡¬Î∆˜µƒ∑¥¿°÷µ≤Ó±Ã´¥Û£¨±Ì æ»¶ ˝∑¢…˙¡À∏ƒ±‰
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>7500)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//º∆À„µ√µΩ¡¨–¯µƒ±‡¬Î∆˜ ‰≥ˆ÷µ
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//º∆À„µ√µΩΩ«∂»÷µ£¨∑∂Œß’˝∏∫Œﬁ«Ó¥Û
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//º∆À„ÀŸ∂»∆Ωæ˘÷µ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}
/*
***********************************************************************************************
*Name          :GetEncoderBias
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/

void GetEncoderBias(volatile Encoder *v, Motor820RRxMsg_t * msg)
{

            v->ecd_bias = msg->angle;  //±£¥Ê≥ı º±‡¬Î∆˜÷µ◊˜Œ™∆´≤Ó  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}

#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"

#include "cmsis_os.h"

#include "peripheral_define.h"

#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "rtos_init.h"
#include "rtos_semaphore.h"

static uint32_t can_count = 0;
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
	
void motorInit(){
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

//uint16_t pitchAngle = 0, yawAngle = 0;
//uint32_t flAngle = 0, frAngle = 0, blAngle = 0, brAngle = 0;
//uint16_t flSpeed = 0, frSpeed = 0, blSpeed = 0, brSpeed = 0;
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	//CanRxMsgTypeDef *temp = IOPool_pGetWriteData(motorCanRxIOPool);
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
			//fw_Warning();
			isRcanStarted_CMGM = 0;
		}else{
			isRcanStarted_CMGM = 1;
		}
		if(isInited == 1){
			osSemaphoreRelease(CMGMCanRefreshSemaphoreHandle);
		}
	}else if(hcan == &ZGYRO_CAN){
		switch(ZGYROCanRxMsg.StdId){
			case ZGYRO_RXID:{
				//LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
			CanRxMsgTypeDef *msg =	&ZGYROCanRxMsg;
			ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 
			}break;
				
			default:
			fw_Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&ZGYRO_CAN, CAN_FIFO0) != HAL_OK){
			//fw_Warning();
			isRcanStarted_ZGYRO = 0;
		}else{
			isRcanStarted_ZGYRO = 1;
		}
		if(isInited == 1){
			osSemaphoreRelease(ZGYROCanRefreshSemaphoreHandle);
		}
	}

}


void CMGMCanTransmitTask(void const * argument){
	while(1){
//		osSemaphoreWait(CMGMCanHaveTransmitSemaphoreHandle, osWaitForever);//osWaitForever
//		fw_printfln("in cantransmit");
		xSemaphoreTake(motorCanTransmitSemaphore, osWaitForever);
		//fw_printfln("osWaitForeverCMGMCanHaveTransmitSemaphoreHandle");
		if(IOPool_hasNextRead(CMTxIOPool, 0)){
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(CMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(CMTxIOPool, 0);
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
			}
			taskEXIT_CRITICAL();
		}
		if(IOPool_hasNextRead(GMTxIOPool, 0)){
			osSemaphoreWait(CMGMCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(GMTxIOPool, 0);
			CMGMMOTOR_CAN.pTxMsg = IOPool_pGetReadData(GMTxIOPool, 0);
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
			}
			taskEXIT_CRITICAL();
		}
		if(isRcanStarted_CMGM == 0){
				if(CMGMMOTOR_CAN.State == HAL_CAN_STATE_BUSY_RX){
					CMGMMOTOR_CAN.State = HAL_CAN_STATE_READY;
				}
				if(HAL_CAN_Receive_IT(&CMGMMOTOR_CAN, CAN_FIFO0) != HAL_OK){
					fw_Warning();
				}else{
					isRcanStarted_CMGM = 1;
				}
			}
	}
}

void ZGYROCanTransmitTask(void const * argument){
	while(1){
		osSemaphoreWait(ZGYROCanHaveTransmitSemaphoreHandle, osWaitForever);//osWaitForever
		if(IOPool_hasNextRead(ZGYROTxIOPool, 0)){
			osSemaphoreWait(ZGYROCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(ZGYROTxIOPool, 0);
			ZGYRO_CAN.pTxMsg = IOPool_pGetReadData(ZGYROTxIOPool, 0);
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&ZGYRO_CAN) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(ZGYROCanTransmitSemaphoreHandle);
			}
			taskEXIT_CRITICAL();
		}
		if(isRcanStarted_ZGYRO == 0){
				if(ZGYRO_CAN.State == HAL_CAN_STATE_BUSY_RX){
					ZGYRO_CAN.State = HAL_CAN_STATE_READY;
				}
				if(HAL_CAN_Receive_IT(&ZGYRO_CAN, CAN_FIFO0) != HAL_OK){
					fw_Warning();
				}else{
					isRcanStarted_ZGYRO = 1;
				}
			}
	}
}


void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &CMGMMOTOR_CAN){
		osSemaphoreRelease(CMGMCanTransmitSemaphoreHandle);
	}else if(hcan == &ZGYRO_CAN){
		osSemaphoreRelease(ZGYROCanTransmitSemaphoreHandle);
	}
}

void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
   	CanTxMsgTypeDef *tx_message = IOPool_pGetWriteData(CMTxIOPool);
		tx_message->StdId = CM_TXID;
    tx_message->Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message->Data[1] = (uint8_t)cm1_iq;
    tx_message->Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message->Data[3] = (uint8_t)cm2_iq;
    tx_message->Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message->Data[5] = (uint8_t)cm3_iq;
    tx_message->Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message->Data[7] = (uint8_t)cm4_iq;
		IOPool_getNextWrite(CMTxIOPool);
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
	if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
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
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
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

            v->ecd_bias = msg->angle;  //保存初始编码器值作为偏差  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}
/*
************************************************************************************************************************
*Name        : CanReceiveMsgProcess
* Description: This function process the can message representing the encoder data received from the CAN2 bus.
* Arguments  : msg     is a pointer to the can message.
* Returns    : void
* Note(s)    : none
************************************************************************************************************************
*/
void CANReceiveMsgProcess_820R(Motor820RRxMsg_t * msg, volatile Encoder * CMxEncoder)
{      
    can_count++;
		(can_count<=50) ? GetEncoderBias(CMxEncoder ,msg):EncoderProcess(CMxEncoder ,msg);       //获取到编码器的初始偏差值            
}

#include "framework_drivers_motorcan.h"

#include "framework_utilities_debug.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can.h"
#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "framework_tasks_cmcontrol.h"
#include "framework_drivers_mpu6050.h"

static uint32_t can_count = 0;
volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
float ZGyroModuleAngle = 0.0f;

/*****Begin define ioPool*****/
#define DataPoolInit {0}
#define ReadPoolSize 6
#define ReadPoolMap {MOTOR1_ID, MOTOR2_ID, MOTOR3_ID, MOTOR4_ID, MOTORYAW_ID, MOTORPITCH_ID}
#define GetIdFunc (data.StdId)
#define ReadPoolInit {{0, Empty, 1}, {2, Empty, 3}, {4, Empty, 5}, {6, Empty, 7}, {8, Empty, 9},{10, Empty, 11}}

IOPoolDefine(motorCanRxIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/
	
/*****Begin define ioPool*****/
#define DataPoolInit {0}
#define ReadPoolSize 1
#define ReadPoolMap {ZGYRO_ID}
#define GetIdFunc (data.StdId)
#define ReadPoolInit {{0, Empty, 1}}

IOPoolDefine(ZGYROCanRxIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

/*****Begin define ioPool*****/
#define DataPoolInit \
	{ \
		{MOTORCM_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{MOTORCM_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{MOTORGIMBAL_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{MOTORGIMBAL_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}}, \
		{0, 0, CAN_ID_STD, CAN_RTR_DATA, 8, {0}} \
	}
#define ReadPoolSize 2
#define ReadPoolMap {MOTORCM_ID, MOTORGIMBAL_ID}
#define GetIdFunc (data.StdId)
#define ReadPoolInit {{0, Empty, 1}, {2, Empty, 3}}

IOPoolDefine(motorCanTxIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

/*****CAN2配置*****/
extern uint8_t isRcanStarted;
extern uint8_t isRcanStarted_AM;
CanRxMsgTypeDef AMCanRxMsg;
void motorInit(){
	motorCan.pRxMsg = IOPool_pGetWriteData(motorCanRxIOPool);
	
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 14;//14 - 27
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&motorCan, &sFilterConfig);
	
	if(HAL_CAN_Receive_IT(&motorCan, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted = 1;
	
	ZGYROCAN.pRxMsg = IOPool_pGetWriteData(ZGYROCanRxIOPool);
	/*##-- Configure the CAN1 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 0;
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = 0;
  sFilterConfig2.FilterActivation = ENABLE;
  sFilterConfig2.BankNumber = 14;
//  HAL_CAN_ConfigFilter(&ZGYROCAN, &sFilterConfig2);
	
	if(HAL_CAN_Receive_IT(&ZGYROCAN, CAN_FIFO0) != HAL_OK){
		fw_Error_Handler(); 
	}
	isRcanStarted_AM = 1;
	
	gyroinit();
}


extern xSemaphoreHandle motorCanReceiveSemaphore;
/*CAN接收回调函数*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
	/*		static int countwhilec = 0;
		if(countwhilec >= 6000){
			countwhilec = 0;
				fw_printfln("can receive 6000");
		}else{
			countwhilec++;
		}*/
	if(hcan == &motorCan){
	IOPool_getNextWrite(motorCanRxIOPool);
	motorCan.pRxMsg = IOPool_pGetWriteData(motorCanRxIOPool);
	if(HAL_CAN_Receive_IT(&motorCan, CAN_FIFO0) != HAL_OK){
		fw_Warning();
		isRcanStarted = 0;
	}else{
		isRcanStarted = 1;
	}
}
	else if(hcan == &ZGYROCAN){
		fw_printfln("zgyrocan receive");
		IOPool_getNextWrite(ZGYROCanRxIOPool);
		ZGYROCAN.pRxMsg = IOPool_pGetWriteData(ZGYROCanRxIOPool);
		if(HAL_CAN_Receive_IT(&ZGYROCAN, CAN_FIFO0) != HAL_OK){
			fw_Warning();
			isRcanStarted_AM = 0;
		}else{
			isRcanStarted_AM = 1;
		}
	}
	xSemaphoreGiveFromISR(motorCanReceiveSemaphore, &xHigherPriorityTaskWoken);
	//上下文切换
	if( xHigherPriorityTaskWoken == pdTRUE ){
   portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}
extern osSemaphoreId motorCanTransmitSemaphoreHandle;
/*CAN发送回调函数*/
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
}

void gyroinit(void){
	CanTxMsgTypeDef tx_message;
    
    tx_message.StdId = 0x404;//send to gyro controll board
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->DLC = 8;

    
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x01;
    tx_message.Data[2] = 0x02;
    tx_message.Data[3] = 0x03;
    tx_message.Data[4] = 0x04;
    tx_message.Data[5] = 0x05;
    tx_message.Data[6] = 0x06;
    tx_message.Data[7] = 0x07;
			ZGYROCAN.pTxMsg = &tx_message;
			taskENTER_CRITICAL();
			if(HAL_CAN_Transmit_IT(&ZGYROCAN) != HAL_OK){
				fw_Warning();
			}
			taskEXIT_CRITICAL();
}
/********************************************************************************
   给底盘电调板发送指令，ID号为0x200８底盘返回ID为0x201-0x204
*********************************************************************************/
void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
   	CanTxMsgTypeDef *tx_message = IOPool_pGetWriteData(motorCanTxIOPool);
		tx_message->StdId = MOTORCM_ID;
    tx_message->Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message->Data[1] = (uint8_t)cm1_iq;
    tx_message->Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message->Data[3] = (uint8_t)cm2_iq;
    tx_message->Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message->Data[5] = (uint8_t)cm3_iq;
    tx_message->Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message->Data[7] = (uint8_t)cm4_iq;
		IOPool_getNextWrite(motorCanTxIOPool);
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
void EncoderProcess(volatile Encoder *v, CanRxMsgTypeDef * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
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

void GetEncoderBias(volatile Encoder *v, CanRxMsgTypeDef * msg)
{

            v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
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
extern float ZGyroModuleAngleMAX;
extern float ZGyroModuleAngleMIN;
void CanReceiveMsgProcess(CanRxMsgTypeDef * msg)
{      
        //GMYawEncoder.ecd_bias = yaw_ecd_bias;
        can_count++;
		switch(msg->StdId)
		{
				case MOTOR1_ID:
				{
//					LostCounterFeed(GetLostCounter(MOTOR1_ID));
					(can_count<=50) ? GetEncoderBias(&CM1Encoder ,msg):EncoderProcess(&CM1Encoder ,msg);       //获取到编码器的初始偏差值            
                    
				}break;
				case MOTOR2_ID:
				{
//					LostCounterFeed(GetLostCounter(MOTOR2_ID));
					(can_count<=50) ? GetEncoderBias(&CM2Encoder ,msg):EncoderProcess(&CM2Encoder ,msg);
				}break;
				case MOTOR3_ID:
				{
//					LostCounterFeed(GetLostCounter(MOTOR3_ID));
					(can_count<=50) ? GetEncoderBias(&CM3Encoder ,msg):EncoderProcess(&CM3Encoder ,msg);   
				}break;
				case MOTOR4_ID:
				{
//					LostCounterFeed(GetLostCounter(MOTOR4_ID));
				 	(can_count<=50) ? GetEncoderBias(&CM4Encoder ,msg):EncoderProcess(&CM4Encoder ,msg);
				}break;
				case MOTORYAW_ID:
				{
//					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
//					 GMYawEncoder.ecd_bias = yaw_ecd_bias;
					 EncoderProcess(&GMYawEncoder ,msg);    
				}break;
						// 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
					// if(can_count>=90 && can_count<=100)
/*					if(GetWorkState() == PREPARE_STATE)   //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
					 {
							 if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) <-4000)
							 {
								GMYawEncoder.ecd_bias = gAppParamStruct.GimbalCaliData.GimbalYawOffset + 8192;
							 }
							 else if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) > 4000)
							 {
								GMYawEncoder.ecd_bias = gAppParamStruct.GimbalCaliData.GimbalYawOffset - 8192;
							 }
					 }
				}break;
				case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
				{
					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
						//GMPitchEncoder.ecd_bias = pitch_ecd_bias;
						EncoderProcess(&GMPitchEncoder ,msg);
						//码盘中间值设定也需要修改
						 if(can_count<=100)
						 {
							 if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-4000)
							 {
								 GMPitchEncoder.ecd_bias = gAppParamStruct.GimbalCaliData.GimbalPitchOffset + 8192;
							 }
							 else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000)
							 {
								 GMPitchEncoder.ecd_bias = gAppParamStruct.GimbalCaliData.GimbalPitchOffset - 8192;
							 }
						 }
				}break;		
*/				
  			case ZGYRO_ID:
				{
//					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
					ZGyroModuleAngle = 0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 
				}break;
				
				default:
				{
				}
		}
		// check if deadlock, meeans the yaw angle is overflow //time should keep for a long time to avoid bug		
/*			if(!LostCounterOverflowCheck(fabs(GMYawEncoder.ecd_angle), 70.0f) || GetWorkState() == STOP_STATE)  //如果是停止模式，一直喂狗防止重新启动失败
			{
				LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
			}		
		*/
}
		

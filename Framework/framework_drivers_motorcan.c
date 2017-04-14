#include "framework_drivers_motorcan.h"

#include "framework_utilities_debug.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can.h"
#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "ControlTask.h"

static uint32_t can_count = 0;
volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)
//void motorInit(void){}
//void printMotorTask(void const * argument){while(1){osDelay(100);}}
//void controlMotorTask(void const * argument){while(1){osDelay(100);}}
//void motorCanTransmitTask(void const * argument){while(1){osDelay(100);}}

#define MOTOR1_ID 0x201u
#define MOTOR2_ID 0x202u
#define MOTOR3_ID 0x203u
#define MOTOR4_ID 0x204u
#define MOTORYAW_ID 0x205u
#define MOTORPITCH_ID 0x206u

/*****Begin define ioPool*****/
#define DataPoolInit {0}
#define ReadPoolSize 6
#define ReadPoolMap {MOTOR1_ID, MOTOR2_ID, MOTOR3_ID, MOTOR4_ID, MOTORYAW_ID, MOTORPITCH_ID}
#define GetIdFunc (data.StdId)
#define ReadPoolInit {{0, Empty, 1}, {2, Empty, 3}, {4, Empty, 5}, {6, Empty, 7}, {8, Empty, 9},{10, Empty, 11}}

IOPoolDeclare(motorCanRxIOPool, CanRxMsgTypeDef);
IOPoolDefine(motorCanRxIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

#define MOTORCM_ID 0x200u
#define MOTORGIMBAL_ID 0x1FFu
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

IOPoolDeclare(motorCanTxIOPool, CanTxMsgTypeDef);
IOPoolDefine(motorCanTxIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

#define motorCan hcan2

uint8_t isRcanStarted = 0;

void motorInit(){
	//wait for 820R
//	for(int i=0; i < 3000; i++)
//	{
//		int a=42000; //at 168MHz 42000 is ok
//		while(a--);
//	}
	
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
}
extern xSemaphoreHandle motorCanReceiveSemaphore;
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
	
	IOPool_getNextWrite(motorCanRxIOPool);
	motorCan.pRxMsg = IOPool_pGetWriteData(motorCanRxIOPool);
	xSemaphoreGiveFromISR(motorCanReceiveSemaphore, &xHigherPriorityTaskWoken);
	if(HAL_CAN_Receive_IT(&motorCan, CAN_FIFO0) != HAL_OK){
		//fw_Warning();
		isRcanStarted = 0;
	}else{
		isRcanStarted = 1;
	}
	/*上下文切换	
	if( xHigherPriorityTaskWoken == pdTRUE )
{
	portSWITCH_CONTEXT();
}*/
}

uint16_t yawAngle = 0, pitchAngle = 0;
void printMotorTask(void const * argument){
//	uint32_t id = 0;
//	uint16_t data0 = 0, data1 = 0, data2 = 0;
	while(1){
		fw_printfln("printMotorTask runing");
		xSemaphoreTake(motorCanReceiveSemaphore, osWaitForever);
		fw_printfln("CanReceiveProcessing");
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORYAW_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORYAW_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORYAW_ID);
			
//			id = pData->StdId;
//			data0 = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
//			data1 = ((uint16_t)pData->Data[2] << 8) + (uint16_t)pData->Data[3];
//			data2 = ((uint16_t)pData->Data[4] << 8) + (uint16_t)pData->Data[5];
			yawAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
			CanReceiveMsgProcess(pData);
		}
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORPITCH_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORPITCH_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORPITCH_ID);
			pitchAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
		}
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTOR1_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTOR1_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTOR1_ID);
			CanReceiveMsgProcess(pData);
		}
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTOR2_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTOR2_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTOR2_ID);
			CanReceiveMsgProcess(pData);
		}
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTOR3_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTOR3_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTOR3_ID);
			CanReceiveMsgProcess(pData);
		}
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTOR4_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTOR4_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTOR3_ID);
			CanReceiveMsgProcess(pData);
		}
		

//		int16_t yawZeroAngle = 1075;
//		float yawRealAngle = (yawAngle - yawZeroAngle) * 360 / 8191.0;
//		yawRealAngle = (yawRealAngle > 180) ? yawRealAngle - 360 : yawRealAngle;
//		yawRealAngle = (yawRealAngle < -180) ? yawRealAngle + 360 : yawRealAngle;
//		fw_printf("yawAngle = %5d | ", yawAngle);
//		fw_printf("yawRealAngle = %f | ", yawRealAngle);
//		int16_t pitchZeroAngle = 710;
//		float pitchRealAngle = (pitchZeroAngle - pitchAngle) * 360 / 8191.0;
//		pitchRealAngle = (pitchRealAngle > 180) ? pitchRealAngle - 360 : pitchRealAngle;
//		pitchRealAngle = (pitchRealAngle < -180) ? pitchRealAngle + 360 : pitchRealAngle;
//		fw_printf("pitchAngle = %5d | ", pitchAngle);
//		fw_printf("pitchRealAngle = %f\r\n", pitchRealAngle);
//		fw_printf("id = %x | ", id);
//		fw_printf("d0 = %5d | ", data0);
//		fw_printf("d1 = %5d | ", data1);
//		fw_printf("d5 = %5d \r\n", data2);
//		osDelay(500);
	}
}

int16_t yawZeroAngle = 1075, pitchZeroAngle = 710;
float yawAngleTarget = 0.0, pitchAngleTarget = 0.0;
extern float gYroX, gYroY, gYroZ;
extern xSemaphoreHandle motorCanTransmitSemaphore;
void controlMotorTask(void const * argument){
//	int16_t testSpeed = 10000;
//	while(1){
//		if(testSpeed <= 0){
//			testSpeed = 4000;
//		}else{
//			testSpeed -= 200;
//		}
//		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(motorCanTxIOPool);
//		pData->StdId = MOTORCM_ID;
//		pData->Data[0] = (uint8_t)(testSpeed >> 8);
//		pData->Data[1] = (uint8_t)testSpeed;
//		IOPool_getNextWrite(motorCanTxIOPool);
//		
//		osDelay(250);
//	}
		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	while(1){
		
//		yawAngleTarget = (yawAngleTarget < 3100) ? yawAngleTarget : 3100;
//		yawAngleTarget = (yawAngleTarget > -1016) ? yawAngleTarget : -1016;
//		pitchAngleTarget = (pitchAngleTarget < 1450) ? pitchAngleTarget : 1450;
//		pitchAngleTarget = (pitchAngleTarget > 70) ? pitchAngleTarget : 70;
		MINMAX(yawAngleTarget, -45, 45);
		MINMAX(pitchAngleTarget, -25, 25);
		
//		int16_t yawAngleS = yawAngle, pitchAngleS = pitchAngle;
		float yawRealAngle = (yawAngle - yawZeroAngle) * 360 / 8191.0;
		yawRealAngle = (yawRealAngle > 180) ? yawRealAngle - 360 : yawRealAngle;
		yawRealAngle = (yawRealAngle < -180) ? yawRealAngle + 360 : yawRealAngle;
		float pitchRealAngle = (pitchZeroAngle - pitchAngle) * 360 / 8191.0;
		pitchRealAngle = (pitchRealAngle > 180) ? pitchRealAngle - 360 : pitchRealAngle;
		pitchRealAngle = (pitchRealAngle < -180) ? pitchRealAngle + 360 : pitchRealAngle;
//angle		
		
//		float yawAngVTarget;
//		float yawAngleP = 1.2;
//		if(yawAngleS > 4000){yawAngleS -= 8191;}
//		yawAngVTarget = (yawAngleTarget - yawAngleS) * yawAngleP;
//		
//		float pitchAngVTarget;
//		float pitchAngleP = 1.2;
//		pitchAngVTarget = (pitchAngleTarget - pitchAngleS) * pitchAngleP;
		float yawAngVTarget;
		float yawAngleP = -20.0;
		yawAngVTarget = (yawAngleTarget - yawRealAngle) * yawAngleP;
		
		float pitchAngVTarget;
		float pitchAngleP = -10.0;
		pitchAngVTarget = (pitchAngleTarget - pitchRealAngle) * pitchAngleP;
//angV
		int16_t yawIntensity = 0;
		float yawAngVP = -15.0;
		float yawIntensityTemp = (yawAngVTarget - gYroZ) * yawAngVP;
		MINMAX(yawIntensityTemp, -4900, 4900);
		yawIntensity = (int16_t)yawIntensityTemp;
		
		int16_t pitchIntensity = 0;
		float pitchAngVP = 12.0;
		float pitchAngVI = 0.0;
		float pitchAngVD = 0;
		static float pitchAngVIntegration = 0.0f;
		static float pitchAngVLast = 0.0f;
		float pitchAngVError = pitchAngVTarget - gYroY;
		pitchAngVIntegration += pitchAngVError;
		float pitchIntensityTemp = pitchAngVError * pitchAngVP 
			+ pitchAngVIntegration * pitchAngVI
			+ (pitchAngVError - pitchAngVLast) * pitchAngVD;
		pitchAngVLast = pitchAngVError;
		MINMAX(pitchIntensityTemp, -4900, 4900);
		pitchIntensity = (int16_t)pitchIntensityTemp;
		
//		yawIntensity = 0;
//		pitchIntensity = 0;
//		fw_printf("yawI = %5d | ", yawIntensity);
		static int countwhile = 0;
		if(countwhile >= 2000){
			countwhile = 0;
//			fw_printf("pvde = %f \r\n", pitchAngVTarget - gYroY);
//			fw_printf("pvdl = %f \r\n", pitchAngVLast);
//			fw_printf("pvd = %f \r\n", (pitchAngVTarget - gYroY - pitchAngVLast) * pitchAngVD);
//			fw_printf("pvde2 = %f \r\n", pitchAngVTarget - gYroY);
//			fw_printf("pvdl2 = %f \r\n", pitchAngVLast);
//			fw_printf("--------\r\n");
			
//			fw_printf("yra = %f | ", yawRealAngle);
//			fw_printf("yI = %5d | ", yawIntensity);
//			fw_printf("yAVT = %f | ", yawAngVTarget);
//			fw_printf("gYroZ = %f \r\n", gYroZ);
			
//			fw_printf("pra = %f | ", pitchRealAngle);
//			fw_printf("pitI = %5d | ", pitchIntensity);
//			fw_printf("pAVT = %f | ", pitchAngVTarget);
//			fw_printf("gYroY = %f \r\n", gYroY);
			fw_printfln("controlMotortask runing");
		}else{
			countwhile++;
		}
		
		
//		yawIntensity = 0;
//		pitchIntensity = 0;
		
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(motorCanTxIOPool);
		pData->StdId = MOTORGIMBAL_ID;
/*		pData->Data[0] = (uint8_t)(yawIntensity >> 8);
		pData->Data[1] = (uint8_t)yawIntensity;
		pData->Data[2] = (uint8_t)(pitchIntensity >> 8);
		pData->Data[3] = (uint8_t)pitchIntensity;*/
			pData->Data[0] = 0;
		pData->Data[1] = 0;
		pData->Data[2] = 0;
		pData->Data[3] = 0;
		IOPool_getNextWrite(motorCanTxIOPool);
		//osDelay(250);
    Control_Task();
//		xSemaphoreGive(motorCanTransmitSemaphore);
		if( xSemaphoreGive( motorCanTransmitSemaphore ) != pdTRUE )
       {
       //    fw_printfln("xemaphoregive error");
       }
		vTaskDelayUntil( &xLastWakeTime, ( 3 / portTICK_RATE_MS ) );
	}
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


extern osSemaphoreId motorCanTransmitSemaphoreHandle;

void motorCanTransmitTask(void const * argument){
	//osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
	static int countwhile = 0;
	while(1){
		xSemaphoreTake(motorCanTransmitSemaphore, osWaitForever);
	/*	     if(xSemaphoreTake(motorCanTransmitSemaphore, osWaitForever) == pdTRUE )
       {
           fw_printfln("take error");
       }

		if(countwhile >= 500){
			countwhile = 0;
			fw_printfln("motorCanTransmitTask runing 500");
		}else{
			countwhile++;
		}*/
		fw_printfln("motorCanTransmitTask runing");
		
		if(IOPool_hasNextRead(motorCanTxIOPool, MOTORCM_ID)){
			//fw_printf("m1");
			osSemaphoreWait(motorCanTransmitSemaphoreHandle, osWaitForever);
			//fw_printf("m2");
			fw_printfln("CM CAN Transmit");
			IOPool_getNextRead(motorCanTxIOPool, MOTORCM_ID);
			motorCan.pTxMsg = IOPool_pGetReadData(motorCanTxIOPool, MOTORCM_ID);
			if(HAL_CAN_Transmit_IT(&motorCan) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
			}
		}
		if(IOPool_hasNextRead(motorCanTxIOPool, MOTORGIMBAL_ID)){
			//fw_printf("w1");
			osSemaphoreWait(motorCanTransmitSemaphoreHandle, osWaitForever);
			//fw_printf("w2");
			fw_printfln("Gimbal CAN Transmit");
			IOPool_getNextRead(motorCanTxIOPool, MOTORGIMBAL_ID);
			motorCan.pTxMsg = IOPool_pGetReadData(motorCanTxIOPool, MOTORGIMBAL_ID);
			if(HAL_CAN_Transmit_IT(&motorCan) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
			}
		}
		if(isRcanStarted == 0){
			if(motorCan.State == HAL_CAN_STATE_BUSY_RX){
				motorCan.State = HAL_CAN_STATE_READY;
			}
			if(HAL_CAN_Receive_IT(&motorCan, CAN_FIFO0) != HAL_OK){
				fw_Warning();
				fw_printf("canstate=%x\r\n", motorCan.State);
			}else{
				//fw_Warning();
				isRcanStarted = 1;
			}
		}
		
	}
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
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
				}
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
/*				case CAN_BUS1_ZGYRO_FEEDBACK_MSG_ID:
				{
					LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
					ZGyroModuleAngle = -0.01f*((int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16) | (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3])); 
				}break;
				*/
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
		

#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"

#include "cmsis_os.h"

#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "freertos_init.h"
#include "freertos_semaphore.h"

#include "tasks_testtasks.h"

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

#define motorCan hcan2

uint8_t isRcanStarted = 0;

void motorInit(){
	//===wait for 820R
	
	
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

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	IOPool_getNextWrite(motorCanRxIOPool);
	motorCan.pRxMsg = IOPool_pGetWriteData(motorCanRxIOPool);
	if(HAL_CAN_Receive_IT(&motorCan, CAN_FIFO0) != HAL_OK){
		//fw_Warning();
		isRcanStarted = 0;
	}else{
		isRcanStarted = 1;
	}
	if(isInited == 1){
		osSemaphoreRelease(motorCanReceiveSemaphoreHandle);
	}
}

int counttestsemwait = 0;
extern int counttestsemreleaseOk;
extern int counttestsemreleaseError;
void motorCanTransmitTask(void const * argument){
	//osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
	while(1){
		motorCanTransmitTasktaskcount++;
//		fw_printfln("w%d rO%d rE%d", counttestsemwait, counttestsemreleaseOk, counttestsemreleaseError);
		osSemaphoreWait(motorCanHaveTransmitSemaphoreHandle, osWaitForever);//osWaitForever
		counttestsemwait++;
		if(IOPool_hasNextRead(motorCanTxIOPool, MOTORCM_ID)){
			osSemaphoreWait(motorCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(motorCanTxIOPool, MOTORCM_ID);
			motorCan.pTxMsg = IOPool_pGetReadData(motorCanTxIOPool, MOTORCM_ID);
			if(HAL_CAN_Transmit_IT(&motorCan) != HAL_OK){
				fw_Warning();
				osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
			}
		}
		if(IOPool_hasNextRead(motorCanTxIOPool, MOTORGIMBAL_ID)){
			osSemaphoreWait(motorCanTransmitSemaphoreHandle, osWaitForever);
			IOPool_getNextRead(motorCanTxIOPool, MOTORGIMBAL_ID);
			motorCan.pTxMsg = IOPool_pGetReadData(motorCanTxIOPool, MOTORGIMBAL_ID);
			HAL_StatusTypeDef test = HAL_CAN_Transmit_IT(&motorCan);
			if(test != HAL_OK){
				fw_printfln("t%d", test);
//				fw_printfln("h%d", motorCan.State);
//				fw_Warning();
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
		//osDelay(1);
	}
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
}

#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "framework_drivers_motorcan.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can.h"
#include "framework_tasks_cmcontrol.h"
#include "pid_regulator.h"
#include "framework_tasks_ctrluart.h"
#include "framework_tasks_remotecontrol.h"
#include "pid_regulator.h"

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)
#define BIT_0 ( 1 << 0 )
#define BIT_3 ( 1 << 3 )
#define BIT_4 ( 1 << 4 )


uint16_t yawAngle = 0, pitchAngle = 0;
extern xSemaphoreHandle motorCanReceiveSemaphore;
extern EventGroupHandle_t xGMControl;
/*Can接收处理任务*/
void canReceiveTask(void const * argument){
	EventBits_t uxBits;
	while(1){
		xSemaphoreTake(motorCanReceiveSemaphore, osWaitForever);
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORYAW_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORYAW_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORYAW_ID);
			
			yawAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
			CanReceiveMsgProcess(pData);
			 uxBits = xEventGroupSetBits(
                        xGMControl,    // The event group being updated.
                        BIT_3 );// The bit being set.
			if( ( uxBits & ( BIT_3 ) ) == ( BIT_3  ) )
    {
        //  bit 4 remained set when the function returned.
    }
		}
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORPITCH_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORPITCH_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORPITCH_ID);
			
			pitchAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
			CanReceiveMsgProcess(pData);
			uxBits = xEventGroupSetBits(
                        xGMControl,    // The event group being updated.
                        BIT_4 );// The bit being set.
			if( ( uxBits & ( BIT_4 ) ) == ( BIT_4  ) )
    {
        //  bit 4 remained set when the function returned.
    }
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
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTOR4_ID);
			
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
	fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
	fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(30.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
	fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(10, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
	fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(15.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
	uint16_t lastYawAngle = 0, lastPitchAngle = 0;
	int16_t pitchIntensity = 0, yawIntensity = 0;
	uint8_t isLastAngleError = 0;
	uint8_t pitchReady = 0, yawReady = 0;


extern float gYroX, gYroY, gYroZ;
extern xSemaphoreHandle motorCanTransmitSemaphore;
extern Shoot_Mode_e shootMode;
/*云台控制任务*/
void GMControlTask(void const * argument){
	EventBits_t uxBits;
			portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	while(1){
//		uxBits = xEventGroupWaitBits(
//                xGMControl,    // The event group being tested.
//                BIT_0 | BIT_3| BIT_4,  // The bits within the event group to wait for.
//                pdFALSE,         // BIT_0 and BIT_4 should not be cleared before returning.
//                pdFALSE,        // wait for either bits
//                osWaitForever); // Wait a maximum of 100ms for either bit to be set.
		uxBits =  BIT_0 | BIT_3| BIT_4;
		if( ( uxBits & ( BIT_0 ) ) == ( BIT_0 ) )
    {
        xEventGroupClearBits(
                            xGMControl,    // The event group being updated.
                            BIT_0 );// The bits being cleared.// xEventGroupWaitBits() returned because both bits were set.
    }
		float yawRealAngle = (yawAngle - yawZeroAngle) * 360 / 8191.0;
		yawRealAngle = (yawRealAngle > 180) ? yawRealAngle - 360 : yawRealAngle;
		yawRealAngle = (yawRealAngle < -180) ? yawRealAngle + 360 : yawRealAngle;
		float pitchRealAngle = (pitchZeroAngle - pitchAngle) * 360 / 8191.0;
		pitchRealAngle = (pitchRealAngle > 180) ? pitchRealAngle - 360 : pitchRealAngle;
		pitchRealAngle = (pitchRealAngle < -180) ? pitchRealAngle + 360 : pitchRealAngle;
		
		if( GetShootMode() == AUTO){
		if(IOPool_hasNextRead(upperGimbalIOPool, 0)){
			IOPool_getNextRead(upperGimbalIOPool, 0);
			yawAngleTarget = yawRealAngle + IOPool_pGetReadData(upperGimbalIOPool, 0)->yawAdd;
			pitchAngleTarget = pitchRealAngle + IOPool_pGetReadData(upperGimbalIOPool, 0)->pitchAdd;
		}
	 }
		MINMAX(yawAngleTarget, -45, 45);
		MINMAX(pitchAngleTarget, -25, 25);
		
	 if( ( uxBits & ( BIT_4 ) ) == ( BIT_4 ) )
    {
        xEventGroupClearBits(
                            xGMControl,    // The event group being updated.
                            BIT_4 );// The bits being cleared.// xEventGroupWaitBits() returned because both bits were set.
    	//position		
			pitchPositionPID.target = pitchAngleTarget;
			pitchPositionPID.feedback = pitchRealAngle;
			pitchPositionPID.Calc(&pitchPositionPID);
			//speed
			pitchSpeedPID.target = pitchPositionPID.output;
			pitchSpeedPID.feedback = -gYroY;
			pitchSpeedPID.Calc(&pitchSpeedPID);
			pitchIntensity = -(int16_t)pitchSpeedPID.output;
			
			pitchReady = 1;
		}
	 	
	 		if( ( uxBits & ( BIT_3 ) ) == ( BIT_3 ) )
    {
      xEventGroupClearBits(
                            xGMControl,    // The event group being updated.
                            BIT_3 );// The bits being cleared.// xEventGroupWaitBits() returned because both bits were set.
    //position		
			yawPositionPID.target = yawAngleTarget;
			yawPositionPID.feedback = yawRealAngle;
			yawPositionPID.Calc(&yawPositionPID);
			//speed
			yawSpeedPID.target = yawPositionPID.output;
			yawSpeedPID.feedback = -gYroZ;
			yawSpeedPID.Calc(&yawSpeedPID);
			yawIntensity = (int16_t)yawSpeedPID.output;
			
			yawReady = 1;
		}
		static int countwhile = 0;
		if(countwhile >= 500){
			countwhile = 0;
			fw_printfln("%f",yawAngleTarget);
			fw_printfln("%d",yawIntensity);
		}else{
			countwhile++;
		}
		
		
//		yawIntensity = 0;
//		pitchIntensity = 0;
		
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(motorCanTxIOPool);
		pData->StdId = MOTORGIMBAL_ID;
		pData->Data[0] = (uint8_t)(yawIntensity >> 8);
		pData->Data[1] = (uint8_t)yawIntensity;
		pData->Data[2] = (uint8_t)(pitchIntensity >> 8);
		pData->Data[3] = (uint8_t)pitchIntensity;
		IOPool_getNextWrite(motorCanTxIOPool);
		pitchReady = yawReady = 0;
		xSemaphoreGive(motorCanTransmitSemaphore);
		//osDelay(250);
			vTaskDelayUntil( &xLastWakeTime, ( 1 / portTICK_RATE_MS ) );
	}
}

extern osSemaphoreId motorCanTransmitSemaphoreHandle;
uint8_t isRcanStarted = 0;
/*Can发送任务*/
void motorCanTransmitTask(void const * argument){
	//osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
	static int countwhile = 0;
	while(1){
		xSemaphoreTake(motorCanTransmitSemaphore, osWaitForever);
		if(countwhile >= 500){
			countwhile = 0;
			fw_printfln("motorCanTransmitTask runing 500");
		}else{
			countwhile++;
		}
		if(IOPool_hasNextRead(motorCanTxIOPool, MOTORGIMBAL_ID)){
		//fw_printf("w1");
			osSemaphoreWait(motorCanTransmitSemaphoreHandle, osWaitForever);
			//fw_printf("w2");
			IOPool_getNextRead(motorCanTxIOPool, MOTORGIMBAL_ID);
			motorCan.pTxMsg = IOPool_pGetReadData(motorCanTxIOPool, MOTORGIMBAL_ID);
			if(HAL_CAN_Transmit_IT(&motorCan) != HAL_OK){
				fw_Warning();
			osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
		}
		}
		if(IOPool_hasNextRead(motorCanTxIOPool, MOTORCM_ID)){
			//fw_printf("m1");
			osSemaphoreWait(motorCanTransmitSemaphoreHandle, osWaitForever);
			//fw_printf("m2");
			IOPool_getNextRead(motorCanTxIOPool, MOTORCM_ID);
			motorCan.pTxMsg = IOPool_pGetReadData(motorCanTxIOPool, MOTORCM_ID);
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

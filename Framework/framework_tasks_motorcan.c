#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "framework_drivers_motorcan.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can.h"
#include "framework_tasks_cmcontrol.h"

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)

uint16_t yawAngle = 0, pitchAngle = 0;
extern xSemaphoreHandle motorCanReceiveSemaphore;
/*Can接收处理任务*/
void printMotorTask(void const * argument){
	while(1){
		xSemaphoreTake(motorCanReceiveSemaphore, osWaitForever);
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORYAW_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORYAW_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORYAW_ID);
			
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
extern float gYroX, gYroY, gYroZ;
extern xSemaphoreHandle motorCanTransmitSemaphore;
/*控制任务，循环*/
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
		if(countwhile >= 500){
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
//			fw_printfln("controlMotortask runing");
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
		xSemaphoreGive(motorCanTransmitSemaphore);
		//osDelay(250);
		WorkStateFSM();
	  WorkStateSwitchProcess();
    CMControl_Task();

		if( xSemaphoreGive( motorCanTransmitSemaphore ) != pdTRUE )
       {
          fw_printfln("xemaphoregive error");
       }
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

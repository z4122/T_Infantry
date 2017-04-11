#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "freertos_semaphore.h"

#include "tasks_testtasks.h"
#include "utilities_debug.h"
#include "tasks_upper.h"
#include "drivers_led_user.h"

#include "stdint.h"

//uint16_t yawAngle = 0, pitchAngle = 0;
//void printMotorTask(void const * argument){
//	while(1){
//		//printMotorTasktaskcount++;
////		osSemaphoreWait(motorCanReceiveSemaphoreHandle, osWaitForever);
////		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORYAW_ID)){
////			IOPool_getNextRead(motorCanRxIOPool, MOTORYAW_ID);
////			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORYAW_ID);
////			yawAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
////		}
////		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORPITCH_ID)){
////			IOPool_getNextRead(motorCanRxIOPool, MOTORPITCH_ID);
////			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORPITCH_ID);
////			pitchAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
////		}
////		osSemaphoreRelease(refreshGimbalSemaphoreHandle);
//	}
//}

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)

extern float gYroX, gYroY, gYroZ;
extern int forPidDebug;

int counttestsemreleaseOk = 0;
int counttestsemreleaseError = 0;
void controlMotorTask(void const * argument){
	float yawAngleTarget = 0.0, pitchAngleTarget = 8.0;
	int16_t yawZeroAngle = 1075, pitchZeroAngle = 710;
	while(1){
		controlMotorTaskTasktaskcount++;
		osSemaphoreWait(refreshGimbalSemaphoreHandle, osWaitForever);
		
		static uint16_t yawAngle = 0, pitchAngle = 0;
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORYAW_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORYAW_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORYAW_ID);
			yawAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
		}
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORPITCH_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORPITCH_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORPITCH_ID);
			pitchAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
		}
		
		MINMAX(yawAngleTarget, -45, 45);
		MINMAX(pitchAngleTarget, -25, 25);
		
		float yawRealAngle = (yawAngle - yawZeroAngle) * 360 / 8192.0;
		yawRealAngle = (yawRealAngle > 180) ? yawRealAngle - 360 : yawRealAngle;
		yawRealAngle = (yawRealAngle < -180) ? yawRealAngle + 360 : yawRealAngle;
		float pitchRealAngle = (pitchZeroAngle - pitchAngle) * 360 / 8192.0;
		pitchRealAngle = (pitchRealAngle > 180) ? pitchRealAngle - 360 : pitchRealAngle;
		pitchRealAngle = (pitchRealAngle < -180) ? pitchRealAngle + 360 : pitchRealAngle;
		
		if(IOPool_hasNextRead(upperGimbalIOPool, 0)){
			IOPool_getNextRead(upperGimbalIOPool, 0);
			yawAngleTarget += IOPool_pGetReadData(upperGimbalIOPool, 0)->yawAdd;
			pitchAngleTarget += IOPool_pGetReadData(upperGimbalIOPool, 0)->pitchAdd;
		}
//angle		
		float yawAngVTarget;
		float yawAngleP = -30.0;
		yawAngVTarget = (yawAngleTarget - yawRealAngle) * yawAngleP;
		
		float pitchAngVTarget;
		float pitchAngleP = -10.0;
		pitchAngVTarget = (pitchAngleTarget - pitchRealAngle) * pitchAngleP;
//angV
		static float lastgYroY = 0;
		static int countlastGY = 0;
		if(lastgYroY != gYroY){
			lastgYroY = gYroY;
			countlastGY = 0;
		}else{
			countlastGY++;
		}
		if(countlastGY > 100){
			ledRStatus = on;
		}
		
		int16_t yawIntensity = 0;
		float yawAngVP = -70.0;
		float yawIntensityTemp = (yawAngVTarget - gYroZ) * yawAngVP;
		MINMAX(yawIntensityTemp, -4900, 4900);
		yawIntensity = (int16_t)yawIntensityTemp;
		
		int16_t pitchIntensity = 0;
		float pitchAngVP = 70.0;
		float pitchAngVI = 0.0;
		float pitchAngVD = 0.0;//
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
		
		static int countwhile = 0;
		static int yIoutCount = 0, pIoutCount = 0, totalCount = 0;
		totalCount++;
		if(yawIntensity == 4900 | yawIntensity == -4900){
			yIoutCount++;
		}
		if(pitchIntensity == 4900 | pitchIntensity == -4900){
			pIoutCount++;
		}
		if(countwhile >= 500){
			countwhile = 0;
//			fw_printf("pvde = %f \r\n", pitchAngVTarget - gYroY);
//			fw_printf("pvdl = %f \r\n", pitchAngVLast);
//			fw_printf("pvd = %f \r\n", (pitchAngVTarget - gYroY - pitchAngVLast) * pitchAngVD);
//			fw_printf("pvde2 = %f \r\n", pitchAngVTarget - gYroY);
//			fw_printf("pvdl2 = %f \r\n", pitchAngVLast);
//			fw_printf("--------\r\n");
			if(forPidDebug == 1){
				fw_printf("yIoc = %d\t", yIoutCount);
				fw_printf("pIoc = %d\t", pIoutCount);
				fw_printf("count = %d\t", totalCount);
				yIoutCount = 0, pIoutCount = 0, totalCount = 0;
				
				fw_printf("ya = %d\t", yawAngle);
				fw_printf("yra = %f\t", yawRealAngle);
				fw_printf("yI = %5d\t", yawIntensity);
				fw_printf("yAVT = %f\t", yawAngVTarget);
				fw_printf("gYroZ = %f\t", gYroZ);
				
				fw_printf("pa = %d\t", pitchAngle);
				fw_printf("pra = %f\t", pitchRealAngle);
				fw_printf("pitI = %5d\t", pitchIntensity);
				fw_printf("pAVT = %f\t", pitchAngVTarget);
				fw_printf("gYroY = %f \r\n", gYroY);
			}
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
//		fw_printfln("pI:%d", pitchIntensity);
		IOPool_getNextWrite(motorCanTxIOPool);
//			uint16_t cm1I = 1000;
//			uint16_t cm2I = 1000;
//			uint16_t cm3I = 1000;
//			uint16_t cm4I = 1000;
//			CanTxMsgTypeDef *pData = IOPool_pGetWriteData(motorCanTxIOPool);
//			pData->StdId = MOTORCM_ID;//====MOTORGIMBAL_ID
//			pData->Data[0] = (uint8_t)(cm1I >> 8);
//			pData->Data[1] = (uint8_t)cm1I;
//			pData->Data[2] = (uint8_t)(cm2I >> 8);
//			pData->Data[3] = (uint8_t)cm2I;
//			pData->Data[4] = (uint8_t)(cm3I >> 8);
//			pData->Data[5] = (uint8_t)cm3I;
//			pData->Data[6] = (uint8_t)(cm4I >> 8);
//			pData->Data[7] = (uint8_t)cm4I;
//		//		fw_printfln("pI:%d", pitchIntensity);
//			IOPool_getNextWrite(motorCanTxIOPool);
		
		if(osSemaphoreRelease(motorCanHaveTransmitSemaphoreHandle) == osErrorOS){
			counttestsemreleaseError++;
		}else{
			counttestsemreleaseOk++;
		}
	}
}

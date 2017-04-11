#include "framework_tasks_motor.h"
#include "framework_drivers_canmotor_user.h"
#include "framework_freertos_semaphore.h"

#include "framework_tasks_testtasks.h"
#include "framework_utilities_debug.h"

#include "stdint.h"

uint16_t yawAngle = 0, pitchAngle = 0;
void printMotorTask(void const * argument){
	while(1){
		printMotorTasktaskcount++;
		osSemaphoreWait(motorCanReceiveSemaphoreHandle, osWaitForever);
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
		osSemaphoreRelease(refreshGimbalSemaphoreHandle);
	}
}

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)

int16_t yawZeroAngle = 1075, pitchZeroAngle = 710;
float yawAngleTarget = 0.0, pitchAngleTarget = 8.0;
extern float gYroX, gYroY, gYroZ;

int counttestsemreleaseOk = 0;
int counttestsemreleaseError = 0;
void controlMotorTask(void const * argument){
	while(1){
		controlMotorTaskTasktaskcount++;
		osSemaphoreWait(refreshGimbalSemaphoreHandle, osWaitForever);
		
		MINMAX(yawAngleTarget, -45, 45);
		MINMAX(pitchAngleTarget, -25, 25);
		
		float yawRealAngle = (yawAngle - yawZeroAngle) * 360 / 8191.0;
		yawRealAngle = (yawRealAngle > 180) ? yawRealAngle - 360 : yawRealAngle;
		yawRealAngle = (yawRealAngle < -180) ? yawRealAngle + 360 : yawRealAngle;
		float pitchRealAngle = (pitchZeroAngle - pitchAngle) * 360 / 8191.0;
		pitchRealAngle = (pitchRealAngle > 180) ? pitchRealAngle - 360 : pitchRealAngle;
		pitchRealAngle = (pitchRealAngle < -180) ? pitchRealAngle + 360 : pitchRealAngle;
//angle		
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
		
//		static int countwhile = 0;
//		if(countwhile >= 500){
//			countwhile = 0;
////			fw_printf("pvde = %f \r\n", pitchAngVTarget - gYroY);
////			fw_printf("pvdl = %f \r\n", pitchAngVLast);
////			fw_printf("pvd = %f \r\n", (pitchAngVTarget - gYroY - pitchAngVLast) * pitchAngVD);
////			fw_printf("pvde2 = %f \r\n", pitchAngVTarget - gYroY);
////			fw_printf("pvdl2 = %f \r\n", pitchAngVLast);
////			fw_printf("--------\r\n");
//			
////			fw_printf("yra = %f | ", yawRealAngle);
////			fw_printf("yI = %5d | ", yawIntensity);
////			fw_printf("yAVT = %f | ", yawAngVTarget);
////			fw_printf("gYroZ = %f \r\n", gYroZ);
//			
////			fw_printf("pra = %f | ", pitchRealAngle);
////			fw_printf("pitI = %5d | ", pitchIntensity);
////			fw_printf("pAVT = %f | ", pitchAngVTarget);
////			fw_printf("gYroY = %f \r\n", gYroY);
//		}else{
//			countwhile++;
//		}
		
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

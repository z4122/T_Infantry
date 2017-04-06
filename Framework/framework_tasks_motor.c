#include "framework_tasks_motor.h"
#include "framework_drivers_canmotor_user.h"
#include "framework_freertos_semaphore.h"

#include "framework_tasks_testtasks.h"
#include "framework_utilities_debug.h"

#include "stdint.h"

uint16_t yawAngle = 0, pitchAngle = 0;
void printMotorTask(void const * argument){
//	uint32_t id = 0;
//	uint16_t data0 = 0, data1 = 0, data2 = 0;
	while(1){
		printMotorTasktaskcount++;
		osSemaphoreWait(motorCanReceiveSemaphoreHandle, osWaitForever);
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORYAW_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORYAW_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORYAW_ID);
			
//			id = pData->StdId;
//			data0 = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
//			data1 = ((uint16_t)pData->Data[2] << 8) + (uint16_t)pData->Data[3];
//			data2 = ((uint16_t)pData->Data[4] << 8) + (uint16_t)pData->Data[5];
			yawAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
		}
		if(IOPool_hasNextRead(motorCanRxIOPool, MOTORPITCH_ID)){
			IOPool_getNextRead(motorCanRxIOPool, MOTORPITCH_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(motorCanRxIOPool, MOTORPITCH_ID);
			pitchAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
		}
		osSemaphoreRelease(refreshGimbalSemaphoreHandle);
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

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)

int16_t yawZeroAngle = 1075, pitchZeroAngle = 710;
float yawAngleTarget = 0.0, pitchAngleTarget = 8.0;
extern float gYroX, gYroY, gYroZ;

int counttestsemreleaseOk = 0;
int counttestsemreleaseError = 0;
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
	while(1){
		controlMotorTaskTasktaskcount++;
		osSemaphoreWait(refreshGimbalSemaphoreHandle, osWaitForever);
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
		
//		yawIntensity = 0;
//		pitchIntensity = 0;
//		fw_printf("yawI = %5d | ", yawIntensity);
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
		
		if(osSemaphoreRelease(motorCanHaveTransmitSemaphoreHandle) == osErrorOS){
			counttestsemreleaseError++;
		}else{
			counttestsemreleaseOk++;
		}
		//osDelay(250);
	}
}

#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "framework_drivers_motorcan.h"
#include "framework_tasks_motorcan.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "can.h"
#include "framework_tasks_cmcontrol.h"
#include "pid_regulator.h"
#include "framework_tasks_ctrluart.h"
#include "framework_tasks_remotecontrol.h"
#include "pid_regulator.h"

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)

uint16_t yawAngle = 0, pitchAngle = 0;
extern xSemaphoreHandle motorCanReceiveSemaphore;
/*Can接收处理任务*/
void canReceivelTask(void const * argument){
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
		
		if(IOPool_hasNextRead(ZGYROCanRxIOPool, ZGYRO_ID)){
			IOPool_getNextRead(ZGYROCanRxIOPool, ZGYRO_ID);
			CanRxMsgTypeDef *pData = IOPool_pGetReadData(ZGYROCanRxIOPool, MOTORYAW_ID);
			
			yawAngle = ((uint16_t)pData->Data[0] << 8) + (uint16_t)pData->Data[1];
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

	int16_t yawZeroAngle = 1075, pitchZeroAngle = 550;
  float magZeroAngle = -142;
  extern float yaw_angle;
	float yawAngleTarget = 0.0, pitchAngleTarget = 0.0;
	fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(20.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
	fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(25.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
	fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(15, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
	fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(20, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
	uint16_t lastYawAngle = 0, lastPitchAngle = 0;
	int16_t pitchIntensity = 0, yawIntensity = 0;
	uint8_t isLastAngleError = 0;
	uint8_t pitchReady = 0, yawReady = 0;

extern float gYroX, gYroY, gYroZ;
extern xSemaphoreHandle motorCanTransmitSemaphore;
extern Shoot_Mode_e shootMode;
extern float yawAdd, pitchAdd;
extern _Bool CReceive;
extern float ZGyroModuleAngle;
float yawRealAngle = 0;
/*控制任务，循环*/
void GMControlTask(void const * argument){
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
//angle		
		WorkStateFSM();
	  WorkStateSwitchProcess();
		yawRealAngle = (yawAngle - yawZeroAngle) * 360 / 8191.0;
		yawRealAngle = (yawRealAngle > 180) ? yawRealAngle - 360 : yawRealAngle;
		yawRealAngle = (yawRealAngle < -180) ? yawRealAngle + 360 : yawRealAngle;
		float pitchRealAngle = (pitchZeroAngle - pitchAngle) * 360 / 8191.0;
		pitchRealAngle = (pitchRealAngle > 180) ? pitchRealAngle - 360 : pitchRealAngle;
		pitchRealAngle = (pitchRealAngle < -180) ? pitchRealAngle + 360 : pitchRealAngle;
		
		if( GetShootMode() == AUTO){
//		if(IOPool_hasNextRead(upperGimbalIOPool, 0)){ 
//			IOPool_getNextRead(upperGimbalIOPool, 0);
////			yawAngleTarget = yawRealAngle - IOPool_pGetReadData(upperGimbalIOPool, 0)->yawAdd;
////			pitchAngleTarget = pitchRealAngle + IOPool_pGetReadData(upperGimbalIOPool, 0)->pitchAdd;
//		float yaw =  MINMAX(IOPool_pGetReadData(upperGimbalIOPool, 0)->yawAdd,-2.0f,2.0f);
//			float pitch = MINMAX(IOPool_pGetReadData(upperGimbalIOPool,  0)->pitchAdd,-2.0f,2.0f);
//			yawAngleTarget = yawRealAngle - yaw;
//			pitchAngleTarget = pitchRealAngle + pitch;
//		}
			if(CReceive){
			IOPool_getNextRead(upperGimbalIOPool, 0);
			yawAngleTarget = yawRealAngle - (yawAdd*0.5);
      pitchAngleTarget = pitchRealAngle + (pitchAdd*0.8) + 2.7;
				CReceive = 0;
	//		fw_printfln("yaw%f pitch%f",yawAngleTarget,pitchAngleTarget);
		  }
	 }
		MINMAX(yawAngleTarget, -45, 45);
		MINMAX(pitchAngleTarget, -25, 25);
	 static uint8_t normalFlag = 0;   //正常工作模式标志
	static uint8_t standbyFlag = 1;  //IMU工作模式标志
	static uint32_t modeChangeDelayCnt = 0;
	static float angleSave = 0.0f;    //用于切换模式时保存切换前的角度值，用于角度给定值切换
//	switch(GetWorkState())
//	{
//		case NORMAL_STATE:
//		{
//			if(standbyFlag == 1)
//			{
//				standbyFlag = 0;
//				normalFlag = 1;
//				yawAngleTarget= angleSave;   //修改设定值为STANDBY状态下记录的最后一个ZGYROMODULEAngle值
//				modeChangeDelayCnt = 0;   //delay清零
//			}
//			yawPositionPID.target = yawAngleTarget*1.5;   //设定给定值
//			yawPositionPID.feedback = ZGyroModuleAngle; 					//设定反馈值
//			angleSave = yaw_angle;   //时刻保存IMU的值用于从NORMAL向STANDBY模式切换
//		}break;
//		case STANDBY_STATE:   //IMU模式
//		{
//			modeChangeDelayCnt++;
//			if(modeChangeDelayCnt < STATE_SWITCH_DELAY_TICK)    //delay的这段时间与NORMAL_STATE一样
//			{
//				yawPositionPID.target = yawAngleTarget*1.5;;   //设定给定值
//				yawPositionPID.feedback = ZGyroModuleAngle; 					//设定反馈值
//				angleSave = yaw_angle;
//			}
//			else     //delay时间到，切换模式到IMU
//			{
//				if(normalFlag == 1)   //修改模式标志
//				{
//					normalFlag = 0;
//					standbyFlag = 1;
//					yawAngleTarget = angleSave;    //保存的是delay时间段内保存的
//				}
//				yawPositionPID.target = yawAngleTarget*1.5;;   //设定给定值
//				yawPositionPID.feedback = yaw_angle; 					//设定反馈值	
//				angleSave = ZGyroModuleAngle;           //IMU模式时，保存ZGyro的值供模式切换时修改给定值使用						
//			}
//		}break;
//		default:
//		break;
//	}
	 //		yawRealAngle = 2*(yaw_angle - magZeroAngle);
	float yawRealAngle1 = (yaw_angle - magZeroAngle);
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
	 	//position		
			yawPositionPID.target = yawAngleTarget*1.5;
	//		yawPositionPID.feedback = yawRealAngle;
	    yawPositionPID.feedback = (ZGyroModuleAngle - 71.8);
			yawPositionPID.Calc(&yawPositionPID);
			//speed
			yawSpeedPID.target = yawPositionPID.output;
			yawSpeedPID.feedback = -gYroZ;
			yawSpeedPID.Calc(&yawSpeedPID);
			yawIntensity = (int16_t)yawSpeedPID.output;
			
			yawReady = 1;
	
		
//		yawIntensity = 0;
//		pitchIntensity = 0;
		if(GetEmergencyFlag() == EMERGENCY){
			yawIntensity = 0;
			pitchIntensity = 0;
		}
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(motorCanTxIOPool);
		pData->StdId = MOTORGIMBAL_ID;
		pData->Data[0] = (uint8_t)(yawIntensity >> 8);
		pData->Data[1] = (uint8_t)yawIntensity;
		pData->Data[2] = (uint8_t)(pitchIntensity >> 8);
		pData->Data[3] = (uint8_t)pitchIntensity;
		IOPool_getNextWrite(motorCanTxIOPool);
		pitchReady = yawReady = 0;
		xSemaphoreGive(motorCanTransmitSemaphore);
		static int countwhile = 0;
		if(countwhile >= 1000){
			countwhile = 0;
//			fw_printfln("%f",yawAngleTarget);
//			fw_printfln("%d",yawIntensity);
//			fw_printfln("encoder %f\r\n mag %f", yawRealAngle,yawRealAngle1);
		}else{
			countwhile++;
		}
		//osDelay(250);
    //	fw_printfln("%ld",StackResidue);
		vTaskDelayUntil( &xLastWakeTime, ( 1 / portTICK_RATE_MS ) );
		
	}
}

extern osSemaphoreId motorCanTransmitSemaphoreHandle;
uint8_t isRcanStarted = 0;
uint8_t isRcanStarted_AM = 0;
/*Can发送任务*/
void motorCanTransmitTask(void const * argument){
	//osSemaphoreRelease(motorCanTransmitSemaphoreHandle);
	static int countwhile = 0;
	while(1){
		xSemaphoreTake(motorCanTransmitSemaphore, osWaitForever);
		if(countwhile >= 5000){
			countwhile = 0;
//			fw_printfln("motorCanTransmitTask runing 5000");
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
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
	fw_printfln("%s", pcTaskName);
}


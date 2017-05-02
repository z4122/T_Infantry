#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"

#include "utilities_debug.h"
#include "tasks_upper.h"
#include "drivers_led_user.h"
#include "utilities_minmax.h"
#include "application_pidfunc.h"
#include "application_motorcontrol.h"
#include "drivers_sonar_user.h"

#include "stdint.h"

//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
PID_Regulator_t pitchPositionPID = PID_INIT(15.0, 0.00, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
PID_Regulator_t yawPositionPID = PID_INIT(15.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
PID_Regulator_t pitchSpeedPID = PID_INIT(25.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 2000.0);
PID_Regulator_t yawSpeedPID = PID_INIT(40.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);

PID_Regulator_t testPositionPID = PID_INIT(6.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t testSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t platePositionPID = PID_INIT(40.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t plateSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);//0.0, 0.00003

PID_Regulator_t UDFLPositionPID = PID_INIT(60.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t UDFLSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 8000.0);
PID_Regulator_t UDFRPositionPID = PID_INIT(60.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t UDFRSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 8000.0);
PID_Regulator_t UDBLPositionPID = PID_INIT(60.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t UDBLSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 8000.0);
PID_Regulator_t UDBRPositionPID = PID_INIT(60.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t UDBRSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 8000.0);

PID_Regulator_t CMFLPositionPID = PID_INIT(100.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t CMFLSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
PID_Regulator_t CMFRPositionPID = PID_INIT(100.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t CMFRSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
PID_Regulator_t CMBLPositionPID = PID_INIT(100.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t CMBLSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);
PID_Regulator_t CMBRPositionPID = PID_INIT(100.0, 0.0, 0.0, 1000000.0, 1000000.0, 1000000.0, 1000000.0);
PID_Regulator_t CMBRSpeedPID = PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4900.0);

extern float gYroXs, gYroYs, gYroZs;
//extern int forPidDebug;

//extern uint16_t pitchAngle, yawAngle;
//extern uint32_t flAngle, frAngle, blAngle, brAngle;
//extern uint16_t flSpeed, frSpeed, blSpeed, brSpeed;

int8_t flUpDown = 0, frUpDown = 0, blUpDown = 0, brUpDown = 0, allUpDown = 0;
void CMGMControlTask(void const * argument){
	while(1){
		//osSemaphoreWait(imurefreshGimbalSemaphoreHandle, osWaitForever);
		osSemaphoreWait(CMGMCanRefreshSemaphoreHandle, osWaitForever);

		
		static float yawAngleTarget = 0.0;
		static float pitchAngleTarget = 0.0;
		if(IOPool_hasNextRead(upperIOPool, 0)){
			IOPool_getNextRead(upperIOPool, 0);
			yawAngleTarget += IOPool_pGetReadData(upperIOPool, 0)->yawAdd;
			pitchAngleTarget += IOPool_pGetReadData(upperIOPool, 0)->pitchAdd;
		}
		
		
		if(IOPool_hasNextRead(GMYAWRxIOPool, 0)){
			
			uint16_t yawZeroAngle = 1075;///1075//4906
			float yawRealAngle = 0.0;
			int16_t yawIntensity = 0;
			
			IOPool_getNextRead(GMYAWRxIOPool, 0); 
			//fw_printfln("YawAngle= %d", IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
			yawRealAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yawZeroAngle) * 360 / 8192.0;
			NORMALIZE_ANGLE180(yawRealAngle);
			MINMAX(yawAngleTarget, -45, 45);
			//position		
			yawPositionPID.target = yawAngleTarget;
			yawPositionPID.feedback = yawRealAngle;
			yawPositionPID.Calc(&yawPositionPID);
			//speed
			yawSpeedPID.target = yawPositionPID.output;
			yawSpeedPID.feedback = -gYroZs;//-
			yawSpeedPID.Calc(&yawSpeedPID);
			yawIntensity = (int16_t)yawSpeedPID.output;
			
//			static int countWhileYaw = 0;
//			if(countWhileYaw > 500){
//				countWhileYaw = 0;
//				fw_printfln("yawIntensity %d", yawIntensity);
//			}else{
//				countWhileYaw++;
//			}
			
			setMotor(GMYAW, yawIntensity);
		}
		if(IOPool_hasNextRead(GMPITCHRxIOPool, 0)){
			
			uint16_t pitchZeroAngle = 3180;//3180//3708
			float pitchRealAngle = 0.0;
			int16_t pitchIntensity = 0;
			
			IOPool_getNextRead(GMPITCHRxIOPool, 0);
			//fw_printfln("PitAngle= %d", IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			pitchRealAngle = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle - pitchZeroAngle) * 360 / 8192.0;
			NORMALIZE_ANGLE180(pitchRealAngle);
			MINMAX(pitchAngleTarget, -25, 25);
			//position		
			pitchPositionPID.target = pitchAngleTarget;
			pitchPositionPID.feedback = -pitchRealAngle;
			pitchPositionPID.Calc(&pitchPositionPID);
			//speed
			pitchSpeedPID.target = pitchPositionPID.output;
			pitchSpeedPID.feedback = -gYroXs;//-			
			pitchSpeedPID.Calc(&pitchSpeedPID);
			pitchIntensity = (int16_t)pitchSpeedPID.output;
		
//			static int countwhilepitch = 0;
//			if(countwhilepitch > 300){
//				countwhilepitch = 0;
//				fw_printf("gYroX= %f\t", gYroXs);
//				fw_printf("PitAngle= %d\t", IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
//				fw_printfln("pitchIntensity= %d", pitchIntensity);
//			}else{
//				countwhilepitch++;
//			}
			
			setMotor(GMPITCH, pitchIntensity);
		}
		
//		if(IOPool_hasNextRead(CMFLRxIOPool, 0)){
//			IOPool_getNextRead(CMFLRxIOPool, 0);
//			static float testRealAngle = 0.0;
//			static float testRealAngleLast = 0.0;
//			
//			static float testAngleTarget = 360.0 * 36 * 2;
//			static int countwhileCMFL = 0;
//			if(countwhileCMFL > 7000){
//				countwhileCMFL = 0;
//				if(testAngleTarget > 0){
//					testAngleTarget = 0.0;
//				}else{
//					testAngleTarget = 360.0 * 36 * 2;
//				}
//			}else{
//				countwhileCMFL++;
//			}
//			
//			float testRealAngleCurr = (IOPool_pGetReadData(CMFLRxIOPool, 0)->angle) * 360 / 8192.0;
//			float testRotateSpeed = (IOPool_pGetReadData(CMFLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
//			
//			if(testRealAngleCurr - testRealAngleLast > 180){
//				testRealAngle += testRealAngleCurr - 360 - testRealAngleLast;
//			}else if(testRealAngleCurr - testRealAngleLast < -180){
//				testRealAngle += testRealAngleCurr + 360 - testRealAngleLast;
//			}else{
//				testRealAngle += testRealAngleCurr - testRealAngleLast;
//			}
//			testRealAngleLast = testRealAngleCurr;
//			
//			//position		
//			testPositionPID.target = testAngleTarget;
//			testPositionPID.feedback = testRealAngle;
//			testPositionPID.Calc(&testPositionPID);
//			//speed
//			testSpeedPID.target = testPositionPID.output;
//			testSpeedPID.feedback = testRotateSpeed;
//			testSpeedPID.Calc(&testSpeedPID);
//			int16_t testIntensity = (int16_t)testSpeedPID.output;
//			
//			setMotor(CMFL, testIntensity);
//		}


//		if(IOPool_hasNextRead(CMBRRxIOPool, 0)){
//			IOPool_getNextRead(CMBRRxIOPool, 0);
//			static float plateRealAngle = 0.0;
//			static float plateRealAngleLast = 0.0;
//			
//			static float plateAngleTarget = 360.0 * 12 * 2;
//			static int countwhileCMBR = 0;
//			if(countwhileCMBR > 250){
//				countwhileCMBR = 0;
//				plateAngleTarget += 360.0 * 12 * 2;
//			}else{
//				countwhileCMBR++;
//			}
//			
//			float plateRealAngleCurr = (IOPool_pGetReadData(CMBRRxIOPool, 0)->angle) * 360 / 8192.0;
//			float plateRotateSpeed = (IOPool_pGetReadData(CMBRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
//			
//			if(plateRealAngleCurr - plateRealAngleLast > 180){
//				plateRealAngle += plateRealAngleCurr - 360 - plateRealAngleLast;
//			}else if(plateRealAngleCurr - plateRealAngleLast < -180){
//				plateRealAngle += plateRealAngleCurr + 360 - plateRealAngleLast;
//			}else{
//				plateRealAngle += plateRealAngleCurr - plateRealAngleLast;
//			}
//			plateRealAngleLast = plateRealAngleCurr;
//			
//			//position		
//			platePositionPID.target = plateAngleTarget;
//			platePositionPID.feedback = plateRealAngle;
//			platePositionPID.Calc(&platePositionPID);
//			//speed
//			plateSpeedPID.target = platePositionPID.output;
//			plateSpeedPID.feedback = plateRotateSpeed;
//			plateSpeedPID.Calc(&plateSpeedPID);
//			int16_t plateIntensity = (int16_t)plateSpeedPID.output;

//			
//			setMotor(CMBR, plateIntensity);
//		}

/*
		if(IOPool_hasNextRead(CMFLRxIOPool, 0)){
			IOPool_getNextRead(CMFLRxIOPool, 0);
			static float UDFLRealAngle = 0.0;
			static float UDFLRealAngleLast = 0.0;
			static float UDFLAngleTarget = 0.0;//360.0 * 12 * 2
			
			UDFLAngleTarget += flUpDown * 360;
			flUpDown = 0;
			
			float UDFLRealAngleCurr = (IOPool_pGetReadData(CMFLRxIOPool, 0)->angle) * 360 / 8192.0;
			float UDFLRotateSpeed = (IOPool_pGetReadData(CMFLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(UDFLRealAngleCurr - UDFLRealAngleLast > 180){
				UDFLRealAngle += UDFLRealAngleCurr - 360 - UDFLRealAngleLast;
			}else if(UDFLRealAngleCurr - UDFLRealAngleLast < -180){
				UDFLRealAngle += UDFLRealAngleCurr + 360 - UDFLRealAngleLast;
			}else{
				UDFLRealAngle += UDFLRealAngleCurr - UDFLRealAngleLast;
			}
			UDFLRealAngleLast = UDFLRealAngleCurr;
			
			//position		
			UDFLPositionPID.target = UDFLAngleTarget;
			UDFLPositionPID.feedback = UDFLRealAngle;
			UDFLPositionPID.Calc(&UDFLPositionPID);
			//speed
			UDFLSpeedPID.target = UDFLPositionPID.output;
			UDFLSpeedPID.feedback = UDFLRotateSpeed;
			UDFLSpeedPID.Calc(&UDFLSpeedPID);
			int16_t UDFLIntensity = (int16_t)UDFLSpeedPID.output;

//			static int countwhileudfl = 0;
//			if(countwhileudfl >= 500){
//				countwhileudfl = 0;
//				fw_printfln("UDFLI= %d", UDFLIntensity);
//			}else{
//				countwhileudfl++;
//			}
			
			setMotor(CMFL, UDFLIntensity);
		}
		if(IOPool_hasNextRead(CMFRRxIOPool, 0)){
			IOPool_getNextRead(CMFRRxIOPool, 0);
			static float UDFRRealAngle = 0.0;
			static float UDFRRealAngleLast = 0.0;
			static float UDFRAngleTarget = 0.0;//360.0 * 12 * 2
			
			
			UDFRAngleTarget += frUpDown * 360;
			frUpDown = 0;
			
			float UDFRRealAngleCurr = (IOPool_pGetReadData(CMFRRxIOPool, 0)->angle) * 360 / 8192.0;
			float UDFRRotateSpeed = (IOPool_pGetReadData(CMFRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(UDFRRealAngleCurr - UDFRRealAngleLast > 180){
				UDFRRealAngle += UDFRRealAngleCurr - 360 - UDFRRealAngleLast;
			}else if(UDFRRealAngleCurr - UDFRRealAngleLast < -180){
				UDFRRealAngle += UDFRRealAngleCurr + 360 - UDFRRealAngleLast;
			}else{
				UDFRRealAngle += UDFRRealAngleCurr - UDFRRealAngleLast;
			}
			UDFRRealAngleLast = UDFRRealAngleCurr;
			
			//position		
			UDFRPositionPID.target = UDFRAngleTarget;
			UDFRPositionPID.feedback = UDFRRealAngle;
			UDFRPositionPID.Calc(&UDFRPositionPID);
			//speed
			UDFRSpeedPID.target = UDFRPositionPID.output;
			UDFRSpeedPID.feedback = UDFRRotateSpeed;
			UDFRSpeedPID.Calc(&UDFRSpeedPID);
			int16_t UDFRIntensity = (int16_t)UDFRSpeedPID.output;

			setMotor(CMFR, UDFRIntensity);
		}
		if(IOPool_hasNextRead(CMBLRxIOPool, 0)){
			IOPool_getNextRead(CMBLRxIOPool, 0);
			static float UDBLRealAngle = 0.0;
			static float UDBLRealAngleLast = 0.0;
			static float UDBLAngleTarget = 0.0;//360.0 * 12 * 2
			
			UDBLAngleTarget += blUpDown * 360;
			blUpDown = 0;
			
			float UDBLRealAngleCurr = (IOPool_pGetReadData(CMBLRxIOPool, 0)->angle) * 360 / 8192.0;
			float UDBLRotateSpeed = (IOPool_pGetReadData(CMBLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(UDBLRealAngleCurr - UDBLRealAngleLast > 180){
				UDBLRealAngle += UDBLRealAngleCurr - 360 - UDBLRealAngleLast;
			}else if(UDBLRealAngleCurr - UDBLRealAngleLast < -180){
				UDBLRealAngle += UDBLRealAngleCurr + 360 - UDBLRealAngleLast;
			}else{
				UDBLRealAngle += UDBLRealAngleCurr - UDBLRealAngleLast;
			}
			UDBLRealAngleLast = UDBLRealAngleCurr;
			
			//position		
			UDBLPositionPID.target = UDBLAngleTarget;
			UDBLPositionPID.feedback = UDBLRealAngle;
			UDBLPositionPID.Calc(&UDBLPositionPID);
			//speed
			UDBLSpeedPID.target = UDBLPositionPID.output;
			UDBLSpeedPID.feedback = UDBLRotateSpeed;
			UDBLSpeedPID.Calc(&UDBLSpeedPID);
			int16_t UDBLIntensity = (int16_t)UDBLSpeedPID.output;

			setMotor(CMBL, UDBLIntensity);
		}
		if(IOPool_hasNextRead(CMBRRxIOPool, 0)){
			IOPool_getNextRead(CMBRRxIOPool, 0);
			static float UDBRRealAngle = 0.0;
			static float UDBRRealAngleLast = 0.0;
			static float UDBRAngleTarget = 0.0;//360.0 * 12 * 2
			
			UDBRAngleTarget += brUpDown * 360;
			brUpDown = 0;
			
			float UDBRRealAngleCurr = (IOPool_pGetReadData(CMBRRxIOPool, 0)->angle) * 360 / 8192.0;
			float UDBRRotateSpeed = (IOPool_pGetReadData(CMBRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			if(UDBRRealAngleCurr - UDBRRealAngleLast > 180){
				UDBRRealAngle += UDBRRealAngleCurr - 360 - UDBRRealAngleLast;
			}else if(UDBRRealAngleCurr - UDBRRealAngleLast < -180){
				UDBRRealAngle += UDBRRealAngleCurr + 360 - UDBRRealAngleLast;
			}else{
				UDBRRealAngle += UDBRRealAngleCurr - UDBRRealAngleLast;
			}
			UDBRRealAngleLast = UDBRRealAngleCurr;
			
			//position		
			UDBRPositionPID.target = UDBRAngleTarget;
			UDBRPositionPID.feedback = UDBRRealAngle;
			UDBRPositionPID.Calc(&UDBRPositionPID);
			//speed
			UDBRSpeedPID.target = UDBRPositionPID.output;
			UDBRSpeedPID.feedback = UDBRRotateSpeed;
			UDBRSpeedPID.Calc(&UDBRSpeedPID);
			int16_t UDBRIntensity = (int16_t)UDBRSpeedPID.output;

			setMotor(CMBR, UDBRIntensity);
		}*/
		
		/*
		static uint8_t isApproach = 1;
		
		if(IOPool_hasNextRead(SonarIOPool, 0)){
			IOPool_getNextRead(SonarIOPool, 0);
			static int countApproach = 0;
			if(countApproach >= 250){
				countApproach = 0;
				isApproach = -isApproach;
			}else{
				countApproach++;
			}
			//fw_printfln("sonarValue[0] %f", IOPool_pGetReadData(SonarIOPool, 0)->sonarValue[0]);
		}
		
			
		if(IOPool_hasNextRead(CMFLRxIOPool, 0)){
			IOPool_getNextRead(CMFLRxIOPool, 0);
			float CMFLPositionTarget;
			
			if(isApproach == 1){
				CMFLPositionTarget = 10;
			}else{
				CMFLPositionTarget = 100;
			}
			
			float CMFLRotateSpeed = (IOPool_pGetReadData(CMFLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			//position		
			CMFLPositionPID.target = CMFLPositionTarget;
			CMFLPositionPID.feedback = IOPool_pGetReadData(SonarIOPool, 0)->sonarValue[0];
			CMFLPositionPID.Calc(&CMFLPositionPID);
			//speed
			CMFLSpeedPID.target = CMFLPositionPID.output;
			CMFLSpeedPID.feedback = -CMFLRotateSpeed;
			CMFLSpeedPID.Calc(&CMFLSpeedPID);
			int16_t CMFLIntensity = -(int16_t)CMFLSpeedPID.output;

//			static int countwhileudfl = 0;
//			if(countwhileudfl >= 500){
//				countwhileudfl = 0;
//				fw_printfln("CMFLIntensity %d", CMFLIntensity);
//			}else{
//				countwhileudfl++;
//			}
			
			
			setMotor(CMFL, CMFLIntensity);
		}
		if(IOPool_hasNextRead(CMFRRxIOPool, 0)){
			IOPool_getNextRead(CMFRRxIOPool, 0);
			float CMFRPositionTarget;
			
			if(isApproach == 1){
				CMFRPositionTarget = 10;
			}else{
				CMFRPositionTarget = 100;
			}
			
			float CMFRRotateSpeed = (IOPool_pGetReadData(CMFRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			//position		
			CMFRPositionPID.target = CMFRPositionTarget;
			CMFRPositionPID.feedback = IOPool_pGetReadData(SonarIOPool, 0)->sonarValue[0];
			CMFRPositionPID.Calc(&CMFRPositionPID);
			//speed
			CMFRSpeedPID.target = CMFRPositionPID.output;
			CMFRSpeedPID.feedback = CMFRRotateSpeed;
			CMFRSpeedPID.Calc(&CMFRSpeedPID);
			int16_t CMFRIntensity = (int16_t)CMFRSpeedPID.output;

			setMotor(CMFR, CMFRIntensity);
		}
		if(IOPool_hasNextRead(CMBLRxIOPool, 0)){
			IOPool_getNextRead(CMBLRxIOPool, 0);
			float CMBLPositionTarget;
			
			if(isApproach == 1){
				CMBLPositionTarget = 10;
			}else{
				CMBLPositionTarget = 100;
			}
			
			float CMBLRotateSpeed = (IOPool_pGetReadData(CMBLRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			//position		
			CMBLPositionPID.target = CMBLPositionTarget;
			CMBLPositionPID.feedback = IOPool_pGetReadData(SonarIOPool, 0)->sonarValue[0];
			CMBLPositionPID.Calc(&CMBLPositionPID);
			//speed
			CMBLSpeedPID.target = CMBLPositionPID.output;
			CMBLSpeedPID.feedback = -CMBLRotateSpeed;
			CMBLSpeedPID.Calc(&CMBLSpeedPID);
			int16_t CMBLIntensity = -(int16_t)CMBLSpeedPID.output;

			setMotor(CMBL, CMBLIntensity);
		}
		if(IOPool_hasNextRead(CMBRRxIOPool, 0)){
			IOPool_getNextRead(CMBRRxIOPool, 0);
			float CMBRPositionTarget;
			
			if(isApproach == 1){
				CMBRPositionTarget = 10;
			}else{
				CMBRPositionTarget = 100;
			}
			
			float CMBRRotateSpeed = (IOPool_pGetReadData(CMBRRxIOPool, 0)->RotateSpeed) * 6;//度每秒(* 360 / 60.0)
			
			//position		
			CMBRPositionPID.target = CMBRPositionTarget;
			CMBRPositionPID.feedback = IOPool_pGetReadData(SonarIOPool, 0)->sonarValue[0];
			CMBRPositionPID.Calc(&CMBRPositionPID);
			//speed
			CMBRSpeedPID.target = CMBRPositionPID.output;
			CMBRSpeedPID.feedback = CMBRRotateSpeed;
			CMBRSpeedPID.Calc(&CMBRSpeedPID);
			int16_t CMBRIntensity = (int16_t)CMBRSpeedPID.output;

			setMotor(CMBR, CMBRIntensity);
		}*/
		
		static int countwhile = 0;
		if(countwhile >= 300){
			countwhile = 0;
//			if(forPidDebug == 1){
//				fw_printf("yIoc = %d\t", yIoutCount);
//				fw_printf("pIoc = %d\t", pIoutCount);
//				fw_printf("count = %d\t", totalCount);
//				yIoutCount = 0, pIoutCount = 0, totalCount = 0;
				
//				fw_printf("ya = %d\t", yawAngle);
//				fw_printf("yra = %f\t", yawRealAngle);
//				fw_printf("yI = %5d\t", yawIntensity);
//				fw_printf("gYroZ = %f\t", gYroZ);
//				
//				fw_printf("pa = %d\t", pitchAngle);
//				fw_printf("pra = %f\t", pitchRealAngle);
//				fw_printf("pitI = %5d\t", pitchIntensity);
//				fw_printf("gYroY = %f \r\n", gYroY);
//			}
		}else{
			countwhile++;
		}		
		
	}
}

void AMControlTask(void const * argument){
	while(1){
		osSemaphoreWait(AMCanRefreshSemaphoreHandle, osWaitForever);
	}
}

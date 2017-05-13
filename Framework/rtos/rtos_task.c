#include "rtos_task.h"
#include "rtos_init.h"

#include "drivers_buzzer_low.h"
#include "drivers_imu_low.h"

//#include "utilities_iopool.h"
#include "drivers_led_low.h"
#include "tasks_remotecontrol.h"
#include "tasks_upper.h"
#include "tasks_cmcontrol.h"
#include "drivers_canmotor_low.h"
#include "tasks_motor.h"
#include "drivers_sonar_low.h"
//#include "drivers_mpu6050_low.h"
//#include "tasks_mpu6050.h"

//#include "tasks_testtasks.h"

#include "utilities_debug.h"

osThreadId ledGreenTaskHandle;
osThreadId ledRedTaskHandle;
osThreadId buzzerTaskHandle;
//IMU
osThreadId printIMUTaskHandle;
//UART
osThreadId RControlTaskHandle;
osThreadId getCtrlUartTaskHandle;
//Motor
osThreadId GMControlTaskHandle;
osThreadId TimerTaskHandle;

osThreadId CMGMCanTransmitTaskHandle;
osThreadId AMCanTransmitTaskHandle;

osThreadId sonarTaskHandle;

//extern osThreadId testFlashTaskHandle;

//#include "drivers_flash.h"
//osThreadId testFlashTaskHandle;

void rtos_addThreads(){
//	osThreadDef(testFlashTask, testFlashTask, osPriorityNormal, 0, 128);
//  testFlashTaskHandle = osThreadCreate(osThread(testFlashTask), NULL);
	
	osThreadDef(ledGreenTask, ledGreenTask, osPriorityNormal, 0, 128);
  ledGreenTaskHandle = osThreadCreate(osThread(ledGreenTask), NULL);
	osThreadDef(ledRedTask, ledRedTask, osPriorityNormal, 0, 128);
  ledRedTaskHandle = osThreadCreate(osThread(ledRedTask), NULL);
	
	osThreadDef(buzzerTask, buzzerTask, osPriorityNormal, 0, 128);
  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);
	
	osThreadDef(printIMUTask, printIMUTask, osPriorityHigh, 0, 128);
  printIMUTaskHandle = osThreadCreate(osThread(printIMUTask), NULL);

	
	osThreadDef(RControlTask, RControlTask, osPriorityAboveNormal , 0, 512);
  RControlTaskHandle = osThreadCreate(osThread(RControlTask), NULL);
	
	osThreadDef(getCtrlUartTask, getCtrlUartTask, osPriorityAboveNormal, 0, 512);
  getCtrlUartTaskHandle = osThreadCreate(osThread(getCtrlUartTask), NULL);

	osThreadDef(GMC_Task, CMGMControlTask, osPriorityAboveNormal, 0, 1024);
  GMControlTaskHandle = osThreadCreate(osThread(GMC_Task), NULL);
	
	osThreadDef(Timer_Task, Timer_2ms_lTask, osPriorityAboveNormal, 0, 256);
  TimerTaskHandle = osThreadCreate(osThread(Timer_Task), NULL);
	
	osThreadDef(CMGMC_T_Task, CMGMCanTransmitTask, osPriorityRealtime, 0, 512);
  CMGMCanTransmitTaskHandle = osThreadCreate(osThread(CMGMC_T_Task), NULL);
	osThreadDef(AMC_T_Task, ZGYROCanTransmitTask, osPriorityRealtime, 0, 512);
  AMCanTransmitTaskHandle = osThreadCreate(osThread(AMC_T_Task), NULL);
}

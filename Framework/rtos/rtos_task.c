#include "rtos_task.h"
#include "rtos_init.h"

#include "drivers_buzzer_low.h"
#include "drivers_imu_low.h"

//#include "utilities_iopool.h"
#include "drivers_led_low.h"
//#include "tasks_remotecontrol.h"
#include "tasks_upper.h"
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
osThreadId printRcTaskHandle;
osThreadId getCtrlUartTaskHandle;
//Motor
osThreadId CMGMControlTaskHandle;
osThreadId AMControlTaskHandle;

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

	
//	osThreadDef(printRcTask, printRcTask, osPriorityNormal, 0, 128);
//  printRcTaskHandle = osThreadCreate(osThread(printRcTask), NULL);
	
	osThreadDef(getCtrlUartTask, getCtrlUartTask, osPriorityNormal, 0, 512);
  getCtrlUartTaskHandle = osThreadCreate(osThread(getCtrlUartTask), NULL);

	osThreadDef(CMGMC_Task, CMGMControlTask, osPriorityAboveNormal, 0, 640);
  CMGMControlTaskHandle = osThreadCreate(osThread(CMGMC_Task), NULL);
	osThreadDef(AMC_Task, AMControlTask, osPriorityAboveNormal, 0, 640);
  AMControlTaskHandle = osThreadCreate(osThread(AMC_Task), NULL);
	
	osThreadDef(CMGMC_T_Task, CMGMCanTransmitTask, osPriorityRealtime, 0, 512);
  CMGMCanTransmitTaskHandle = osThreadCreate(osThread(CMGMC_T_Task), NULL);
	osThreadDef(AMC_T_Task, AMCanTransmitTask, osPriorityRealtime, 0, 512);
  AMCanTransmitTaskHandle = osThreadCreate(osThread(AMC_T_Task), NULL);
	
//	osThreadDef(sonarTask, sonarTask, osPriorityNormal, 0, 128);
//  sonarTaskHandle = osThreadCreate(osThread(sonarTask), NULL);

//	fw_printfln("ledGreenTaskHandle = %x", (uint16_t)ledGreenTaskHandle);
//	fw_printfln("ledRedTaskHandle = %x", (uint16_t)ledRedTaskHandle);
//	fw_printfln("buzzerTaskHandle = %x", (uint16_t)buzzerTaskHandle);
//	fw_printfln("printIMUTaskHandle = %x", (uint16_t)printIMUTaskHandle);
//	fw_printfln("printRcTaskHandle = %x", (uint16_t)printRcTaskHandle);
//	fw_printfln("printCtrlUartTaskHandle = %x", (uint16_t)printCtrlUartTaskHandle);
//	fw_printfln("CMGMControlTaskHandle = %x", (uint16_t)CMGMControlTaskHandle);
//	fw_printfln("AMControlTaskHandle = %x", (uint16_t)AMControlTaskHandle);
//	fw_printfln("CMGMCanTransmitTaskHandle = %x", (uint16_t)CMGMCanTransmitTaskHandle);
//	fw_printfln("AMCanTransmitTaskHandle = %x", (uint16_t)AMCanTransmitTaskHandle);
}

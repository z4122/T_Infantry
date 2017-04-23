#include "framework_freertos_task.h"
#include "framework_freertos_init.h"

#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "framework_drivers_led_task.h"
#include "framework_drivers_motorcan.h"
#include "framework_drivers_mpu6050.h"
#include "framework_drivers_uart.h"

#include "framework_tasks_remotecontrol.h"
#include "framework_tasks_ctrluart.h"
#include "framework_tasks_motorcan.h"
#include "framework_tasks_cmcontrol.h"

osThreadId ledGTaskHandle;
osThreadId ledRTaskHandle;
osThreadId RControlTaskHandle;
osThreadId printMotorTaskHandle;
osThreadId GMControlTaskHandle;
osThreadId CMControlTaskHandle;
osThreadId motorCanTransmitTaskHandle;
osThreadId printMPU6050TaskHandle;
osThreadId readMPU6050TaskHandle;
osThreadId CtrlUartTaskHandle;

void fw_freertos_addThreads(){
	osThreadDef(ledGTask, ledGTask, osPriorityNormal , 0, 128);
  ledGTaskHandle = osThreadCreate(osThread(ledGTask), NULL);
	osThreadDef(ledRTask, ledRTask, osPriorityNormal , 0, 128);
  ledRTaskHandle = osThreadCreate(osThread(ledRTask), NULL);
	
	osThreadDef(RControlTask, RControlTask, osPriorityAboveNormal , 0, 128);
  RControlTaskHandle = osThreadCreate(osThread(RControlTask), NULL);
	
	osThreadDef(canReceivelTask, canReceivelTask, osPriorityAboveNormal, 0, 128);
  printMotorTaskHandle = osThreadCreate(osThread(canReceivelTask), NULL);
	osThreadDef(GMControlTask, GMControlTask, osPriorityNormal, 0, 248);
  GMControlTaskHandle = osThreadCreate(osThread(GMControlTask), NULL);
	osThreadDef(CMControlTask, CMControlTask, osPriorityNormal, 0, 248);
  CMControlTaskHandle = osThreadCreate(osThread(CMControlTask), NULL);
	osThreadDef(motorCanTransmitTask, motorCanTransmitTask, osPriorityAboveNormal, 0, 128);
  motorCanTransmitTaskHandle = osThreadCreate(osThread(motorCanTransmitTask), NULL);
	
	osThreadDef(printMPU6050Task, printMPU6050Task, osPriorityNormal, 0, 128);
  printMPU6050TaskHandle = osThreadCreate(osThread(printMPU6050Task), NULL);
	osThreadDef(readMPU6050Task, readMPU6050Task, osPriorityNormal, 0, 128);
  readMPU6050TaskHandle = osThreadCreate(osThread(readMPU6050Task), NULL);
	
	osThreadDef(CtrlUartTask, CtrlUartTask, osPriorityNormal, 0, 128);
  CtrlUartTaskHandle = osThreadCreate(osThread(CtrlUartTask), NULL);
	fw_printfln("taskint finished");
}

#include "framework_freertos_task.h"
#include "framework_freertos_init.h"

#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "framework_drivers_led_task.h"
#include "framework_tasks_remotecontrol.h"
#include "framework_drivers_motorcan.h"
#include "framework_drivers_mpu6050.h"
#include "framework_drivers_uart.h"

osThreadId ledGTaskHandle;
osThreadId ledRTaskHandle;
osThreadId printRcTaskHandle;
osThreadId printMotorTaskHandle;
osThreadId controlMotorTaskTaskHandle;
osThreadId motorCanTransmitTaskHandle;
osThreadId printMPU6050TaskHandle;
osThreadId readMPU6050TaskHandle;
osThreadId printCtrlUartTaskHandle;

void fw_freertos_addThreads(){
	osThreadDef(ledGTask, ledGTask, osPriorityNormal, 0, 128);
  ledGTaskHandle = osThreadCreate(osThread(ledGTask), NULL);
	osThreadDef(ledRTask, ledRTask, osPriorityNormal, 0, 128);
  ledRTaskHandle = osThreadCreate(osThread(ledRTask), NULL);
	
	osThreadDef(printRcTask, printRcTask, osPriorityNormal, 0, 128);
  printRcTaskHandle = osThreadCreate(osThread(printRcTask), NULL);
	
	osThreadDef(printMotorTask, printMotorTask, osPriorityNormal, 0, 128);
  printMotorTaskHandle = osThreadCreate(osThread(printMotorTask), NULL);
	osThreadDef(controlMotorTask, controlMotorTask, osPriorityNormal, 0, 128);
  controlMotorTaskTaskHandle = osThreadCreate(osThread(controlMotorTask), NULL);
	osThreadDef(motorCanTransmitTask, motorCanTransmitTask, osPriorityNormal, 0, 128);
  motorCanTransmitTaskHandle = osThreadCreate(osThread(motorCanTransmitTask), NULL);
	
	osThreadDef(printMPU6050Task, printMPU6050Task, osPriorityNormal, 0, 128);
  printMPU6050TaskHandle = osThreadCreate(osThread(printMPU6050Task), NULL);
	osThreadDef(readMPU6050Task, readMPU6050Task, osPriorityNormal, 0, 128);
  readMPU6050TaskHandle = osThreadCreate(osThread(readMPU6050Task), NULL);
	
	osThreadDef(printCtrlUartTask, printCtrlUartTask, osPriorityNormal, 0, 128);
  printCtrlUartTaskHandle = osThreadCreate(osThread(printCtrlUartTask), NULL);
}

#include "freertos_task.h"
#include "freertos_init.h"

#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "drivers_led_low.h"
#include "tasks_remotecontrol.h"
#include "tasks_upper.h"
#include "drivers_canmotor_low.h"
#include "tasks_motor.h"
#include "drivers_mpu6050_low.h"
#include "tasks_mpu6050.h"

#include "tasks_testtasks.h"


osThreadId ledGTaskHandle;
osThreadId ledRTaskHandle;
osThreadId printRcTaskHandle;
osThreadId printMotorTaskHandle;
osThreadId controlMotorTaskTaskHandle;
osThreadId motorCanTransmitTaskHandle;
osThreadId printMPU6050TaskHandle;
osThreadId readMPU6050TaskHandle;
osThreadId printCtrlUartTaskHandle;

osThreadId printTasksTaskHandle;

void fw_freertos_addThreads(){
	osThreadDef(ledGTask, ledGTask, osPriorityNormal, 0, 128);
  ledGTaskHandle = osThreadCreate(osThread(ledGTask), NULL);
	osThreadDef(ledRTask, ledRTask, osPriorityNormal, 0, 128);
  ledRTaskHandle = osThreadCreate(osThread(ledRTask), NULL);
	
	osThreadDef(printRcTask, printRcTask, osPriorityNormal, 0, 128);
  printRcTaskHandle = osThreadCreate(osThread(printRcTask), NULL);
	
	osThreadDef(printMotorTask, printMotorTask, osPriorityAboveNormal, 0, 128);
  printMotorTaskHandle = osThreadCreate(osThread(printMotorTask), NULL);
	osThreadDef(controlMotorTask, controlMotorTask, osPriorityHigh, 0, 128);
  controlMotorTaskTaskHandle = osThreadCreate(osThread(controlMotorTask), NULL);
	
	osThreadDef(motorCanTransmitTask, motorCanTransmitTask, osPriorityRealtime, 0, 128);
  motorCanTransmitTaskHandle = osThreadCreate(osThread(motorCanTransmitTask), NULL);
	
	osThreadDef(printMPU6050Task, printMPU6050Task, osPriorityHigh, 0, 128);
  printMPU6050TaskHandle = osThreadCreate(osThread(printMPU6050Task), NULL);
	osThreadDef(readMPU6050Task, readMPU6050Task, osPriorityHigh, 0, 128);
  readMPU6050TaskHandle = osThreadCreate(osThread(readMPU6050Task), NULL);
	
	osThreadDef(printCtrlUartTask, printCtrlUartTask, osPriorityNormal, 0, 128);
  printCtrlUartTaskHandle = osThreadCreate(osThread(printCtrlUartTask), NULL);
	
	osThreadDef(printTasksTask, printTasksTask, osPriorityNormal, 0, 128);
  printTasksTaskHandle = osThreadCreate(osThread(printTasksTask), NULL);
}

#ifndef FRAMEWORK_TASKS_TESTTASKS_H
#define FRAMEWORK_TASKS_TESTTASKS_H

//extern osThreadId ledGTaskHandle;
//extern osThreadId ledRTaskHandle;
//extern osThreadId printRcTaskHandle;
//extern osThreadId printMotorTaskHandle;
//extern osThreadId controlMotorTaskTaskHandle;
//extern osThreadId motorCanTransmitTaskHandle;
//extern osThreadId printMPU6050TaskHandle;
//extern osThreadId readMPU6050TaskHandle;
//extern osThreadId printCtrlUartTaskHandle;

extern int ledGTasktaskcount;
extern int ledRTasktaskcount;
extern int printRcTasktaskcount;
extern int printMotorTasktaskcount;
extern int controlMotorTaskTasktaskcount;
extern int motorCanTransmitTasktaskcount;
extern int printMPU6050Tasktaskcount;
extern int readMPU6050Tasktaskcount;
extern int printCtrlUartTasktaskcount;

void printTasksTask(void const * argument);

#endif

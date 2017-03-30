#ifndef FRAMEWORK_MOTORCAN_H
#define FRAMEWORK_MOTORCAN_H

void motorInit(void);
void printMotorTask(void const * argument);
void controlMotorTask(void const * argument);
void motorCanTransmitTask(void const * argument);

#endif

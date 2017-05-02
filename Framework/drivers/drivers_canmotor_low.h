#ifndef DRIVERS_CANMOTOR_LOW_H
#define DRIVERS_CANMOTOR_LOW_H

void motorInit(void);

void CMGMCanTransmitTask(void const * argument);
void AMCanTransmitTask(void const * argument);

#endif

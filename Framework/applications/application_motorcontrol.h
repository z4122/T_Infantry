#ifndef APPLICATION_MOTORCONTROL_H
#define APPLICATION_MOTORCONTROL_H

#include "stdint.h"

typedef enum {CMFL, CMFR, CMBL, CMBR, GMYAW, GMPITCH} MotorId;

void setMotor(MotorId motorId, int16_t Intensity);

#endif

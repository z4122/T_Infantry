#ifndef APPLICATION_MOTORCONTROL_H
#define APPLICATION_MOTORCONTROL_H

#include "stdint.h"

typedef enum {CMFL, CMFR, CMBL, CMBR, GMYAW, GMPITCH, AM1UDFL, AM1UDFR, AM1UDBL, AM1UDBR, AM2PLATE, AM2GETBULLET} MotorId;

void setMotor(MotorId motorId, int16_t Intensity);

#endif

#ifndef FRAMEWORK_DRIVERS_MPU6050_USER_H
#define FRAMEWORK_DRIVERS_MPU6050_USER_H

#include "utilities_iopool.h"
IOPoolDeclare(mpuI2CIOPool, struct{uint8_t ch[20];});

#endif

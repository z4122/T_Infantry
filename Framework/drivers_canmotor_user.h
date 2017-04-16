#ifndef FRAMEWORK_DRIVERS_CANMOTOR_USER_H
#define FRAMEWORK_DRIVERS_CANMOTOR_USER_H

#include "utilities_iopool.h"
#include "can.h"

#define MOTOR1_ID 0x201u
#define MOTOR2_ID 0x202u
#define MOTOR3_ID 0x203u
#define MOTOR4_ID 0x204u
#define MOTORYAW_ID 0x205u
#define MOTORPITCH_ID 0x206u

#define MOTORCM_ID 0x200u
#define MOTORGIMBAL_ID 0x1FFu

IOPoolDeclare(motorCanRxIOPool, CanRxMsgTypeDef);
IOPoolDeclare(motorCanTxIOPool, CanTxMsgTypeDef);

#endif

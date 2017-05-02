#ifndef DRIVERS_CANMOTOR_USER_H
#define DRIVERS_CANMOTOR_USER_H

#include "utilities_iopool.h"
#include "can.h"

//RxID
#define CMFL_RXID 0x201u
#define CMFR_RXID 0x202u
#define CMBL_RXID 0x203u
#define CMBR_RXID 0x204u

#define GMYAW_RXID 0x205u
#define GMPITCH_RXID 0x206u

#define AMUDFL_RXID 0x201u
#define AMUDFR_RXID 0x202u
#define AMUDBL_RXID 0x203u
#define AMUDBR_RXID 0x204u
#define AMPLATE_RXID 0x205u
#define AMGETBULLET_RXID 0x206u

//TxID
#define CM_TXID 0x200u
#define GM_TXID 0x1FFu
#define AM1_TXID 0x200u
#define AM2_TXID 0x1FFu

//RxIOPool
typedef struct{
	uint16_t angle;
	int16_t realIntensity;
	int16_t giveIntensity;
}Motor6623RxMsg_t;
//6623--[0,1]Angle;[2,3]RealIntensity;[4,5]GiveIntensity;
IOPoolDeclare(GMPITCHRxIOPool, Motor6623RxMsg_t);
IOPoolDeclare(GMYAWRxIOPool, Motor6623RxMsg_t);

typedef struct{
	uint16_t angle;
	int16_t RotateSpeed;//RPM
}Motor820RRxMsg_t;
//820R--[0,1]Angle;[2,3]RotateSpeed;
IOPoolDeclare(CMFLRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(CMFRRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(CMBLRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(CMBRRxIOPool, Motor820RRxMsg_t);

//820R--[0,1]Angle;[2,3]RotateSpeed;
IOPoolDeclare(AMUDFLRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AMUDFRRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AMUDBLRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AMUDBRRxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AMPLATERxIOPool, Motor820RRxMsg_t);
IOPoolDeclare(AMGETBULLETRxIOPool, Motor820RRxMsg_t);

//TxIOPool
IOPoolDeclare(CMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(GMTxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(AM1TxIOPool, CanTxMsgTypeDef);
IOPoolDeclare(AM2TxIOPool, CanTxMsgTypeDef);


#endif

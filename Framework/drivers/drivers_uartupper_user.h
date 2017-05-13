#ifndef DRIVERS_UARTUPPER_USER_H
#define DRIVERS_UARTUPPER_USER_H

#include "utilities_iopool.h"
#define size_frame 14  //定义妙算串口帧长度/字节
#define byte_SOF 0x7d    //起始字节	
#define byte_EOF 0x7e    //终止字节
#define byte_ESCAPE 0xff //转义字节

typedef struct{
	uint16_t dev_yaw;
	uint16_t dev_pitch;
	uint8_t rune_locate;	
	uint8_t rune;
	uint16_t target_dis;
	uint8_t DLC;
	uint8_t Success;
}xdata_ctrlUart;

typedef enum{
	Locating = 0,
	Located = 1,
}Locate_State_e;
void SetLocateState(Locate_State_e v);
Locate_State_e GetLocateState(void);

typedef enum{
	AIMING = 0,
	NOAIMING = 1,
}Rune_State_e;
void SetRuneState(Rune_State_e v);
Rune_State_e GetRuneState(void);

typedef struct{
	float yaw_position;
	float pitch_position;
}Location_Number_s;

void vRefreshLocation(float yaw_center, float pitch_center);
	
IOPoolDeclare(ctrlUartIOPool, struct{uint8_t ch[size_frame];});

void ctrlUartRxCpltCallback(void);

#endif

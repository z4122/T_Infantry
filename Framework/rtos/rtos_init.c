#include "rtos_init.h"

#include "utilities_debug.h"
#include "drivers_canmotor_low.h"
//#include "drivers_mpu6050_low.h"
#include "peripheral_tim.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartrc_low.h"
#include "tasks_cmcontrol.h"
#include "drivers_imu_low.h"
#include "utilities_tim.h"

#include "drivers_buzzer_low.h"

uint8_t isInited = 0;
void rtos_init(){
  //wait for devices
//	for(int i=0; i < 3000; i++)
//	{
//		int a=42000; //at 168MHz 42000 is ok
//		while(a--);
//	}
	playMusicWhenInit();
	fw_userTimeEnable();
	MPU6500_Init();
	IST8310_Init();
	ctrlUartInit();
	RemoteTaskInit();
	UserTimerInit();
	CMControtLoopTaskInit();
	motorInit();
//	mpu6050Init();
//	Init_Quaternion();
	fw_printfln("init success");
}

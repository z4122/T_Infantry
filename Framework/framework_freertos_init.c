#include "framework_freertos_init.h"

#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "framework_drivers_led_user.h"
#include "framework_drivers_canmotor_low.h"
#include "framework_drivers_mpu6050_low.h"
#include "framework_drivers_uartupper_low.h"
#include "framework_drivers_uartremotecontrol_low.h"

uint8_t isInited = 0;
void fw_freertos_init(){
	//wait for devices
	for(int i=0; i < 3000; i++)
	{
		int a=42000; //at 168MHz 42000 is ok
		while(a--);
	}
	
	rcInit();
	ctrlUartInit();
	motorInit();
	mpu6050Init();
	Init_Quaternion();
	fw_printfln("init success");
}

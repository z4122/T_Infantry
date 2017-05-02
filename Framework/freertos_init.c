#include "freertos_init.h"

#include "utilities_debug.h"
#include "utilities_iopool.h"
#include "drivers_led_user.h"
#include "drivers_canmotor_low.h"
#include "drivers_mpu6050_low.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartremotecontrol_low.h"

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

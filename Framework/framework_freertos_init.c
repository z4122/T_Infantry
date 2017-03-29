#include "framework_freertos_init.h"

#include "framework_debug.h"
#include "framework_iopool.h"
#include "framework_led.h"
#include "framework_remotecontrol.h"
#include "framework_motorcan.h"
#include "framework_mpu6050.h"
#include "framework_uart.h"

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

#include "framework_freertos_init.h"

#include "framework_utilities_debug.h"
#include "framework_utilities_iopool.h"
#include "framework_drivers_led.h"
#include "framework_drivers_uartremotecontrol_task.h"
#include "framework_drivers_motorcan.h"
#include "framework_drivers_mpu6050.h"
#include "framework_drivers_uart.h"
#include "framework_tasks_remotecontrol.h"
#include "framework_tasks_cmcontrol.h"

void fw_freertos_init(){
	//wait for devices
	for(int i=0; i < 3000; i++)
	{
		int a=42000; //at 168MHz 42000 is ok
		while(a--);
	}
	
	rcInit();
	ctrlUartInit();
	mpu6050Init();
	Init_Quaternion();
	fw_printfln("init success");
  motorInit();
	
	//ÒÆÖ²º¯Êý
	RemoteTaskInit();
	ControtLoopTaskInit();
}

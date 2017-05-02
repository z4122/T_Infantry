#include "peripheral_gpio.h"
#include "gpio.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_init.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//fw_printfln("HAL_GPIO_EXTI_Callback %d", GPIO_Pin);
	switch(GPIO_Pin){
		case MPU_INT:
			//fw_printfln("MPU_INT");
			if(isInited == 1){
				osSemaphoreRelease(refreshMPU6500SemaphoreHandle);
			}
			break;
		case IST_INT:
			//osSemaphoreRelease(refreshMPU6050SemaphoreHandle);
			break;
		default:
			fw_Warning();
	}
}

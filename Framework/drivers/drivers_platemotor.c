#include "gpio.h"
#include "utilities_debug.h"
#include "tim.h"
#include "drivers_platemotor.h"

void plateMotorInit(void){
	HAL_GPIO_WritePin(PM_Dir_Ctrl1_GPIO_Port,PM_Dir_Ctrl1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PM_Dir_Ctrl2_GPIO_Port,PM_Dir_Ctrl2_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	fw_printf("PMInit Success\t\n");
	
}

void setPlateMotorDir(RotateDir_e dir)
{
	if(dir==FORWARD)
	{
		HAL_GPIO_WritePin(PM_Dir_Ctrl1_GPIO_Port,PM_Dir_Ctrl1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PM_Dir_Ctrl2_GPIO_Port,PM_Dir_Ctrl2_Pin,GPIO_PIN_RESET);
	}
	if(dir==REVERSE)
	{
		HAL_GPIO_WritePin(PM_Dir_Ctrl1_GPIO_Port,PM_Dir_Ctrl1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PM_Dir_Ctrl2_GPIO_Port,PM_Dir_Ctrl2_Pin,GPIO_PIN_SET);
	}
}


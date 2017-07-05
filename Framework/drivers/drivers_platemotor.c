#include <gpio.h>
#include <tim.h>
#include "utilities_debug.h"
#include "drivers_platemotor.h"

RotateDir_e PlateMotorDir = FORWARD;

void plateMotorInit(void){
	setPlateMotorDir(FORWARD);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim5, 0x0);
	fw_printf("PMInit Success\t\n");
	
}

void setPlateMotorDir(RotateDir_e dir)
{
	if(dir==FORWARD)
	{
		PlateMotorDir = FORWARD;
		HAL_GPIO_WritePin(PM_Dir_Ctrl1_GPIO_Port,PM_Dir_Ctrl1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PM_Dir_Ctrl2_GPIO_Port,PM_Dir_Ctrl2_Pin,GPIO_PIN_SET);
	}
	if(dir==REVERSE)
	{
		PlateMotorDir = REVERSE;
		HAL_GPIO_WritePin(PM_Dir_Ctrl1_GPIO_Port,PM_Dir_Ctrl1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(PM_Dir_Ctrl2_GPIO_Port,PM_Dir_Ctrl2_Pin,GPIO_PIN_RESET);
	}
}

RotateDir_e getPlateMotorDir(){
	return PlateMotorDir;
}

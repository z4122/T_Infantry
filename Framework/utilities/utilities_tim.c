#include "utilities_tim.h"

#include "peripheral_define.h"
#include "tim.h"

uint64_t timeMicros = 0;
uint64_t fw_getTimeMicros(void){
	return timeMicros + __HAL_TIM_GET_COUNTER(&USER_TIM);
}

void fw_userTimeEnable(void){
	HAL_TIM_Base_Start_IT(&USER_TIM);
	//__HAL_TIM_ENABLE(&USER_TIM);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &USER_TIM){
		timeMicros += 0xFFFF;
	}
}

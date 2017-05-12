#include "peripheral_tim.h"
#include "cmsis_os.h"
#include "tim.h"
#include "peripheral_define.h"
#include "pwm_server_motor.h"
void UserTimerInit(void)
{
//	HAL_TIM_Encoder_Start(&PLATE_TIM, TIM_CHANNEL_ALL);
//	HAL_TIM_PWM_Start(&PLATE_MOTOR_TIM , TIM_CHANNEL_1);//²¦ÅÌµç»ú
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&FRICTION_TIM, TIM_CHANNEL_2);
	pwm_server_motor_init(0);
}

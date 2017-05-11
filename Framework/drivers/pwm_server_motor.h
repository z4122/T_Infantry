#ifndef _pwm_server_motor_
#define _pwm_server_motor_
#include "stm32f4xx_hal.h"
#include "main.h" 
#include "tim.h"

//motorIndex: 对新主控，电源线朝右放置，上面一排的PWM驱动口,从左往右依次为0~7；下面一排PWM驱动口，从左往右依次为8~15
void pwm_server_motor_init(uint8_t motorIndex);
void pwm_server_motor_deinit(uint8_t motorIndex);
//angle:0~180 in degrees
void pwm_server_motor_set_angle(uint8_t motorIndex,float angle);

#endif

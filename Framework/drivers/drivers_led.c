/**
  ******************************************************************************
  * File Name          : drivers_led.c
  * Description        : LED闪烁任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * LED任务运行在最低优先级，可以用来判断程序是否宕机
	* 也可用来显示运行情况(todo)
  ******************************************************************************
  */
	
#include <cmsis_os.h>
#include <gpio.h>
#include "drivers_led_user.h"
#include "drivers_led_low.h"
#include "peripheral_gpio.h"
#include "OLED.h"


#define ledGreenOn() HAL_GPIO_WritePin(GREEN_PIN, GREEN_GPIO_PORT, GPIO_PIN_RESET)
#define ledGreenOff() HAL_GPIO_WritePin(GREEN_PIN, GREEN_GPIO_PORT, GPIO_PIN_SET)
#define ledRedOn() HAL_GPIO_WritePin(RED_PIN, RED_GPIO_PORT, GPIO_PIN_RESET)
#define ledRedOff() HAL_GPIO_WritePin(RED_PIN, RED_GPIO_PORT, GPIO_PIN_SET)

LedStatus_t ledGreenStatus = blink, ledRedStatus = blink;

void ledGreenTask(void const * argument){
	while(1){
		if(ledGreenStatus == on){
			ledGreenOn();
		}else if(ledGreenStatus == off){
			ledGreenOff();
		}else if(ledGreenStatus == blink){
			ledGreenOn();
			static int16_t OLEDcnt = 0;
			OLEDcnt ++;
			if(OLEDcnt>2000) OLEDcnt = 0;
			Oled_Clear();
			Oled_Putstr(1,1,"TPP2018");
			Oled_Putstr(2,1,"cnt");
			Oled_Putnum(3,2,OLEDcnt);
			osDelay(1000);
		}
	}
}

void ledRedTask(void const * argument){
	while(1){
		if(ledRedStatus == on){
			ledRedOn();
		}else if(ledRedStatus == off){
			ledRedOff();
		}else if(ledRedStatus == blink){
			ledRedOn();
			osDelay(66);
			ledRedOff();
			osDelay(88);
		}
	}
}

/**
  ******************************************************************************
  * File Name          : drivers_uart.c
  * Description        : 电机串口驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 串口接收终端回调函数：遥控器
	* 妙算通讯
	* 裁判系统读取
  ******************************************************************************
  */
#include <usart.h>
#include "drivers_uart.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartjudge_low.h"
#include "peripheral_define.h"
#include "utilities_debug.h"


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &RC_UART){
		rcUartRxCpltCallback();
		
	}else if(UartHandle == &CTRL_UART){
		ctrlUartRxCpltCallback();
	}
	else if(UartHandle == &JUDGE_UART){
		judgeUartRxCpltCallback();
	}
}   

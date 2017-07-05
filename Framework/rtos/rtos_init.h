/**
  ******************************************************************************
  * File Name          : rtos_init.h
  * Description        : FreeRTOS初始化
  ******************************************************************************
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  ******************************************************************************
  */
#ifndef RTOS_INIT_H
#define RTOS_INIT_H

#include <stdint.h>
#include <stdbool.h>

extern bool isInited;
void rtos_init(void);
void rtos_addSemaphores(void);
void rtos_addThreads(void);

#endif

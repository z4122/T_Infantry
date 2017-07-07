/**
  ******************************************************************************
  * File Name          : tasks_motor.h
  * Description        : 电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 使用define对于不同车辆的参数进行区分
	* 主要为云台初始位置时的编码器位置
	* 以及其他可能存在的性能差异
  ******************************************************************************
  */
#ifndef TASKS_MOTOR_H
#define TASKS_MOTOR_H

//#define MOTOR_ARMED
//#define Infantry_1_Aim
//#define Infantry_2//5
//#define Infantry_3//4
#define Infantry_4//1
void CMGMControlTask(void const * argument);

#endif

/**
  ******************************************************************************
  * File Name          : utilities_iopool.h
  * Description        : IOPOOL
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * IOPOOL 
	* 注释存在于骆庭晟队长脑海中
	* 有志者可以自己读
  ******************************************************************************
  */
#include "utilities_iopool.h"
#include "utilities_debug.h"

ReadPoolIndex_t getReadPoolIndexPrototype(Id_t id, uint8_t readPoolSize, const Id_t* const readPoolMap){
	ReadPoolIndex_t i;
	for(i = 0; i != readPoolSize; ++i){
		if(readPoolMap[i] == id){
			return i;
		}
	}
	fw_Error_Handler();
	return i;
}

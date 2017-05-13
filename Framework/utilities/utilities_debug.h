#ifndef UTILITIES_DEBUG_H
#define UTILITIES_DEBUG_H

//调试开关
#define _FW_DEBUG

#ifdef _FW_DEBUG
	#include "stdio.h"
	#define fw_printf(...) printf(__VA_ARGS__)
	
	extern void Error_Handler(void);
	#define fw_printfln(format, ...) fw_printf(format"\r\n", ##__VA_ARGS__)
	#define fw_Error_Handler() {fw_printf("%s(%d): ", __FILE__, __LINE__);Error_Handler();}
	#define fw_Warning() {fw_printf("%s(%d): \r\n", __FILE__, __LINE__);}
#else
	#define fw_printf(...)  
	#define fw_printfln(format, ...)
	#define fw_Error_Handler()
	#define fw_Warning()
#endif



#endif

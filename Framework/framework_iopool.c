#include "framework_iopool.h"

extern void Error_Handler(void);
#include "framework_debug.h"

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

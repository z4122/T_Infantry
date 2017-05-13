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

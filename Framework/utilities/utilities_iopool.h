#ifndef UTILITIES_IOPOOL_H
#define UTILITIES_IOPOOL_H

#include <stdint.h>


typedef uint16_t Id_t;
typedef uint8_t DataIndex_t;
typedef uint8_t ReadPoolIndex_t;
typedef enum{Empty, NextRead, Locked} ExchangeStatus_t;

typedef struct{
	DataIndex_t forRead;
	ExchangeStatus_t exchangeStatus;
	DataIndex_t forExchange;
}ReadPool_t;

#define DataTypeDef(ioPool, dataType) typedef dataType ioPool##_Data_t

#define IOPoolDef(ioPool) \
struct{ \
	uint8_t readPoolSize; \
	Id_t *readPoolMap; \
	ioPool##_Data_t *data; \
	ReadPool_t *readPool; \
	DataIndex_t forWrite; \
}

#define getIdDec(ioPool) \
Id_t ioPool##_getId(ioPool##_Data_t data)

#define getIdDef(ioPool, function) \
Id_t ioPool##_getId(ioPool##_Data_t data){ \
		return function; \
}

ReadPoolIndex_t getReadPoolIndexPrototype(Id_t id, uint8_t readPoolSize, const Id_t* const readPoolMap);
#define getReadPoolIndex(ioPool, id) \
	getReadPoolIndexPrototype(id, ioPool.readPoolSize, ioPool.readPoolMap)


#define IOPool_hasNextRead(ioPool, id) \
	(ioPool.readPool[getReadPoolIndex(ioPool, id)].exchangeStatus == NextRead)

#define IOPool_getNextRead(ioPool, id) { \
	ReadPool_t *readPool = ioPool.readPool + getReadPoolIndex(ioPool, id); \
	if(readPool->exchangeStatus == NextRead){ \
		readPool->exchangeStatus = Locked; \
		DataIndex_t temp = readPool->forRead; \
		readPool->forRead = readPool->forExchange; \
		readPool->forExchange = temp; \
		readPool->exchangeStatus = Empty; \
	} \
}

#define IOPool_pGetReadData(ioPool, id) \
	(ioPool.data + ioPool.readPool[getReadPoolIndex(ioPool, id)].forRead)

#define IOPool_getNextWrite(ioPool) { \
	ReadPool_t *readPool = ioPool.readPool \
		+ getReadPoolIndex(ioPool, ioPool##_getId(ioPool.data[ioPool.forWrite])); \
	if(readPool->exchangeStatus != Locked){ \
		readPool->exchangeStatus = Locked; \
		DataIndex_t temp = ioPool.forWrite; \
		ioPool.forWrite = readPool->forExchange; \
		readPool->forExchange = temp; \
		readPool->exchangeStatus = NextRead; \
	} \
}

#define IOPool_pGetWriteData(ioPool) \
	(ioPool.data + ioPool.forWrite)


#define IOPoolDeclare( \
	IOPoolName, \
	DataType \
) \
DataTypeDef(IOPoolName, DataType); \
typedef IOPoolDef(IOPoolName) IOPoolName##_t; \
getIdDec(IOPoolName); \
extern IOPoolName##_t IOPoolName;

#define NaiveIOPoolDefine( \
	IOPoolName, \
	DataPoolInit \
) \
Id_t IOPoolName##_readPoolMap[] = {0}; \
IOPoolName##_Data_t IOPoolName##_data[1 * 2 + 1] = DataPoolInit; \
ReadPool_t IOPoolName##_readPool[1] = {0, Empty, 1}; \
IOPoolName##_t IOPoolName = { \
	1, \
	IOPoolName##_readPoolMap, \
	IOPoolName##_data, \
	IOPoolName##_readPool, \
	1 * 2 \
}; \
getIdDef(IOPoolName, 0);

//#define IOPoolDefine( \
//	IOPoolName, \
//	DataPoolInit, \
//	ReadPoolSize, \
//	ReadPoolMap, \
//	GetIdFunc, \
//	ReadPoolInit \
//) \
//Id_t IOPoolName##_readPoolMap[] = ReadPoolMap; \
//IOPoolName##_Data_t IOPoolName##_data[ReadPoolSize * 2 + 1] = DataPoolInit; \
//ReadPool_t IOPoolName##_readPool[ReadPoolSize] = ReadPoolInit; \
//IOPoolName##_t IOPoolName = { \
//	ReadPoolSize, \
//	IOPoolName##_readPoolMap, \
//	IOPoolName##_data, \
//	IOPoolName##_readPool, \
//	ReadPoolSize * 2 \
//}; \
//getIdDef(IOPoolName, GetIdFunc);

#endif

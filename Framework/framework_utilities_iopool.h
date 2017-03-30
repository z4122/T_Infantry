#ifndef FRAMEWORK_IOPOOL_H
#define FRAMEWORK_IOPOOL_H

///*****Begin define ioPool*****/
//#define IOPoolName0 _name_ 
//#define DataType _type_
//#define DataPoolInit {0}
//#define ReadPoolSize _size_
//#define ReadPoolMap {_ID0_, _ID1_, ...}
//#define GetIdFunc _(data.StdId)_ 
//#define ReadPoolInit {{0, Empty, 1}, {2, Empty, 3}, ...}

//DefineIOPool(IOPoolName0, DataType, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

//#undef DataType
//#undef DataPoolInit 
//#undef ReadPoolSize 
//#undef ReadPoolMap
//#undef GetIdFunc
//#undef ReadPoolInit
///*****End define ioPool*****/

#include <stdint.h>

#define DefineIOPool( \
	IOPoolName, \
	DataType, \
	DataPoolInit, \
	ReadPoolSize, \
	ReadPoolMap, \
	GetIdFunc, \
	ReadPoolInit \
) \
const uint8_t ReadPoolSizeDef(IOPoolName) = ReadPoolSize; \
const Id_t ReadPoolMapDef(IOPoolName)[ReadPoolSizeDef(IOPoolName)] = ReadPoolMap; \
DataTypeDef(IOPoolName, DataType); \
getIdDef(IOPoolName, GetIdFunc); \
typedef IOPoolDef(IOPoolName) IOPoolName##_t; \
IOPoolName##_t IOPoolName = {DataPoolInit, ReadPoolInit, ReadPoolSizeDef(IOPoolName) * 2}; 

typedef uint16_t Id_t;
typedef uint8_t DataIndex_t;
typedef uint8_t ReadPoolIndex_t;
typedef enum{Empty, NextRead, Locked} ExchangeStatus_t;

typedef struct{
	DataIndex_t forRead;
	ExchangeStatus_t exchangeStatus;
	DataIndex_t forExchange;
}ReadPool_t;

ReadPoolIndex_t getReadPoolIndexPrototype(Id_t id, uint8_t readPoolSize, const Id_t* const readPoolMap);

#define ReadPoolSizeDef(ioPool) ioPool##_READPOOLSIZE
#define ReadPoolMapDef(ioPool) ioPool##_READPOOLMAP
#define DataTypeDef(ioPool, dataType) typedef dataType ioPool##_Data_t

#define IOPoolDef(ioPool) \
struct{ \
	ioPool##_Data_t data[ioPool##_READPOOLSIZE * 2 + 1]; \
	ReadPool_t readPool[ioPool##_READPOOLSIZE]; \
	DataIndex_t forWrite; \
}

#define getIdDef(ioPool, function) \
Id_t ioPool##_getId(ioPool##_Data_t data){ \
		return function; \
}

#define getReadPoolIndex(ioPool, id) \
	getReadPoolIndexPrototype(id, ioPool##_READPOOLSIZE, ioPool##_READPOOLMAP)

#define IOPool_pGetWriteData(ioPool) \
	(ioPool.data + ioPool.forWrite)

#define IOPool_pGetReadData(ioPool, id) \
	(ioPool.data + ioPool.readPool[getReadPoolIndex(ioPool, id)].forRead)

#define IOPool_hasNextRead(ioPool, id) \
	(ioPool.readPool[getReadPoolIndex(ioPool, id)].exchangeStatus == NextRead)

#define IOPool_getNextRead(ioPool, id) { \
	ReadPool_t *readPool = ioPool.readPool + getReadPoolIndex(ioPool, id); \
	readPool->exchangeStatus = Locked; \
	DataIndex_t temp = readPool->forRead; \
	readPool->forRead = readPool->forExchange; \
	readPool->forExchange = temp; \
	readPool->exchangeStatus = Empty; \
}

#define IOPool_getNextWrite(ioPool) { \
	ReadPool_t *readPool = ioPool.readPool \
		+ getReadPoolIndex(ioPool, ioPool##_getId(ioPool.data[ioPool.forWrite])); \
	if(readPool->exchangeStatus != Locked){ \
		DataIndex_t temp = ioPool.forWrite; \
		ioPool.forWrite = readPool->forExchange; \
		readPool->forExchange = temp; \
		readPool->exchangeStatus = NextRead; \
	} \
}

#endif

#include "framework_drivers_mpu6050_low.h"
#include "framework_drivers_mpu6050_user.h"
#include "framework_drivers_mpu6050_address.h"

#include "cmsis_os.h"
#include "i2c.h"
#include "framework_utilities_debug.h"


/*****Begin define ioPool*****/
#define DataPoolInit {0}
#define ReadPoolSize 1
#define ReadPoolMap {0}
#define GetIdFunc 0 
#define ReadPoolInit {0, Empty, 1}

IOPoolDefine(mpuI2CIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

#define mpuI2C hi2c1

void I2C_Error_Handler(I2C_HandleTypeDef *hi2c){
//	fw_printfln("I2C_Error_Handler");
//	HAL_I2C_DeInit(hi2c);//释放IO口为GPIO,复位句柄状态标志
//	HAL_I2C_Init(hi2c);//重新初始化I2C控制器
}

void IIC_WriteData(uint8_t dev_addr, uint8_t reg_addr, uint8_t data){
	if(HAL_I2C_Mem_Write(&mpuI2C, dev_addr, reg_addr, 1, &data, 1, 5) != HAL_OK){
		//fw_printfln("WriteData_Error");
		I2C_Error_Handler(&mpuI2C);
		if(HAL_I2C_Mem_Write(&mpuI2C, dev_addr, reg_addr, 1, &data, 1, 5) != HAL_OK){
			Error_Handler();
		}
	}
}	

void mpu6050Init(void){
	//MPU6050 Init
	//fw_printfln("MPU6050 init");
	uint8_t readBuff = 0;
	if(HAL_I2C_Mem_Read(&mpuI2C, MPU6050_DEVICE_ADDRESS, WHO_AM_I, 1, &readBuff, 1, 5) != HAL_OK){
		//fw_printfln("MPU6050_WHO_AM_I");
		I2C_Error_Handler(&mpuI2C);
		if(HAL_I2C_Mem_Read(&mpuI2C, MPU6050_DEVICE_ADDRESS, WHO_AM_I, 1, &readBuff, 1, 5) != HAL_OK){
			Error_Handler();
		}
	}
	//fw_printfln("MPU6050_WHO_AM_I success");
	if(readBuff != MPU6050_ID){
		Error_Handler();
	}
	//fw_printfln("MPU6050_wrtieData");
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x03);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x10);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x00);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x00);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,MPU6050_RA_USER_CTRL,0x00);
	
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x01);
	IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01);
	//fw_printfln("MPU6050_wrtieData success");
	
	//HMC5883 Init
	//IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I,&temp_data,1)
	//MPU6050_ReadData(uint8_t Slave_Addr, uint8_t Reg_Addr, uint8_t * Data, uint8_t Num)
	//IIC_ReadData(Slave_Addr,Reg_Addr,Data,Num)
	//MPU6050_ReadData(HMC5883_ADDRESS, HMC58X3_R_IDA, &tmp_ch, 1)
	//IIC_ReadData(HMC5883_ADDRESS,HMC58X3_R_IDA,tmp_ch,1)
	//fw_printfln("HMC5883_WHO_AM_I");
  if(HAL_I2C_Mem_Read(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_IDA, 1, &readBuff, 1, 5) != HAL_OK){
		//fw_printfln("HMC5883_WHO_AM_I_Error");
		I2C_Error_Handler(&mpuI2C);
		if(HAL_I2C_Mem_Read(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_IDA, 1, &readBuff, 1, 5) != HAL_OK){
			Error_Handler();
		}
	}
	//fw_printfln("HMC5883_WHO_AM_I success");
	if(readBuff != HMC5883_DEVICE_ID_A){
		Error_Handler();
	}
	//fw_printfln("HMC5883_WHO_AM_I device success");
	IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,0x70);
	//osDelay(5);
	IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFB,0xA0);
	//osDelay(5);
	IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_MODE,0x00);    //这里初始化为0x00 连续模式
	//wait the response of the hmc5883 stabalizes, 6 milliseconds 
	//osDelay(6);
	//set DOR
	IIC_WriteData(HMC5883_ADDRESS, HMC58X3_R_CONFA,6<<2);   //75HZ更新
	
	fw_printfln("success MPU6050 init");
}


float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
//初始化IMU数据
#define BOARD_DOWN 1   //板子正面朝下摆放

#include "math.h"
void Init_Quaternion()//根据测量数据，初始化q0,q1,q2.q3，从而加快收敛速度
{
	int16_t hx,hy,hz;
	//fw_printfln("wait for mpuI2CIOPool");
	if(HAL_I2C_Mem_Read(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_XM, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch + 14, 6, 100) != HAL_OK){
				fw_Error_Handler();
//			I2C_Error_Handler(&mpuI2C);
//				
//			if(HAL_I2C_Mem_Read_DMA(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_XM, 1, 
//			IOPool_pGetWriteData(mpuI2CIOPool)->ch + 14, 6) != HAL_OK){
//				Error_Handler();
//			}
		}
	IOPool_getNextWrite(mpuI2CIOPool);
	//fw_printfln("??get mpuI2CIOPool??");
	//fw_printfln("begin mpuI2CIOPool getnextread");
	IOPool_getNextRead(mpuI2CIOPool, 0);
	//fw_printfln("IOPool_getNextRead");
	uint8_t *pData = IOPool_pGetReadData(mpuI2CIOPool, 0)->ch;
	//	fw_printfln("pData =");
	hx = (int16_t)(((int16_t)pData[14]) << 8) | pData[15];
	hy = (int16_t)(((int16_t)pData[16]) << 8) | pData[17];
	hz = (int16_t)(((int16_t)pData[18]) << 8) | pData[19];
		//fw_printfln("hz");
	if(hz){}
	
	//fw_printfln("b");
	#ifdef BOARD_DOWN
	if(hx<0 && hy <0)   //OK
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx<0 && hy > 0) //OK
	{
		if(fabs((float)hx/hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)   //OK
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)     //OK
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if(hx<0 && hy <0)
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx<0 && hy > 0)
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx>0 && hy > 0)
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs((float)hx/hy)>=1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
	
	//根据hx hy hz来判断q的值，取四个相近的值做逼近即可,初始值可以由欧拉角转换到四元数计算得到
	 //fw_printfln("Init_Quaternion finish");
}


extern osSemaphoreId readMPU6050SemaphoreHandle;
extern osSemaphoreId refreshMPU6050SemaphoreHandle;
void readMPU6050Task(void const * argument){
	while(1){
		//fw_printfln("wait refresh");
		osSemaphoreWait(refreshMPU6050SemaphoreHandle, osWaitForever);
		//fw_printfln("wait read");
		osSemaphoreWait(readMPU6050SemaphoreHandle, osWaitForever);
		//fw_printfln("begin read dma");
		if(HAL_I2C_Mem_Read_DMA(&mpuI2C, MPU6050_DEVICE_ADDRESS, MPU6050_DATA_START, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch, 14) != HAL_OK){
			I2C_Error_Handler(&mpuI2C);
				
			if(HAL_I2C_Mem_Read_DMA(&mpuI2C, MPU6050_DEVICE_ADDRESS, MPU6050_DATA_START, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch, 14) != HAL_OK){
				Error_Handler();
			}
		}
		//fw_printfln("wait read2");
		osSemaphoreWait(readMPU6050SemaphoreHandle, osWaitForever);
		//fw_printfln("begin read dma2");
		if(HAL_I2C_Mem_Read_DMA(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_XM, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch + 14, 6) != HAL_OK){
			I2C_Error_Handler(&mpuI2C);
				
			if(HAL_I2C_Mem_Read_DMA(&mpuI2C, HMC5883_ADDRESS, HMC58X3_R_XM, 1, 
			IOPool_pGetWriteData(mpuI2CIOPool)->ch + 14, 6) != HAL_OK){
				Error_Handler();
			}
		}
		IOPool_getNextWrite(mpuI2CIOPool);
	}
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	//IOPool_getNextWrite(mpuI2CIOPool);
	//fw_printfln("release read");
	osSemaphoreRelease(readMPU6050SemaphoreHandle);
//	if(HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_DEVICE_ADDRESS, MPU6050_DATA_START, 1, 
//		IOPool_pGetWriteData(mpuI2CIOPool)->ch, 14) != HAL_OK){
//		Error_Handler();
//	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_4){
		//fw_printfln("release refresh");
		osSemaphoreRelease(refreshMPU6050SemaphoreHandle);
	}
}

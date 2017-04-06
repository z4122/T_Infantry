#include "framework_tasks_mpu6050.h"
#include "framework_drivers_mpu6050_user.h"
#include "framework_freertos_semaphore.h"
#include "framework_tasks_testtasks.h"

#include <stdint.h>
#include <math.h>

extern float q0, q1 ,q2 ,q3;
float invSqrt(float x);

uint32_t Get_Time_Micros(void);
float gx, gy, gz, ax, ay, az, mx, my, mz;
float gYroX, gYroY, gYroZ;
void printMPU6050Task(void const * argument){
	while(1){
		printMPU6050Tasktaskcount++;
		osSemaphoreWait(refreshIMUSemaphoreHandle, osWaitForever);
		if(IOPool_hasNextRead(mpuI2CIOPool, 0)){
			IOPool_getNextRead(mpuI2CIOPool, 0);
			uint8_t *pData = IOPool_pGetReadData(mpuI2CIOPool, 0)->ch;
			
			int16_t myax = (int16_t)(((int16_t)pData[0]) << 8) | pData[1];
			int16_t myay = (int16_t)(((int16_t)pData[2]) << 8) | pData[3];
			int16_t myaz = (int16_t)(((int16_t)pData[4]) << 8) | pData[5];

			int16_t mygx = (int16_t)(((int16_t)pData[8]) << 8) | pData[9];
			int16_t mygy = (int16_t)(((int16_t)pData[10]) << 8) | pData[11];
			int16_t mygz = (int16_t)(((int16_t)pData[12]) << 8) | pData[13];
			
			int16_t mymy = (int16_t)(((int16_t)pData[14]) << 8) | pData[15];
			int16_t mymz = (int16_t)(((int16_t)pData[16]) << 8) | pData[17];
			int16_t mymx = (int16_t)(((int16_t)pData[18]) << 8) | pData[19];
			
//			static int16_t mymaxmx = -30000, myminmx = 30000;
//			static int16_t mymaxmy = -30000, myminmy = 30000;
//			static int16_t mymaxmz = -30000, myminmz = 30000;
//			
//			if(mymx > mymaxmx)mymaxmx = mymx;
//			if(mymx < myminmx)myminmx = mymx;
//			if(mymy > mymaxmy)mymaxmy = mymy;
//			if(mymy < myminmy)myminmy = mymy;
//			if(mymz > mymaxmz)mymaxmz = mymz;
//			if(mymz < myminmz)myminmz = mymz;

			
			float mygetqval[9];
			mygetqval[0] = (float)myax;
			mygetqval[1] = (float)myay;
			mygetqval[2] = (float)myaz;
			
			mygetqval[3] = (float)mygx / 32.8f;
			mygetqval[4] = (float)mygy / 32.8f;
			mygetqval[5] = (float)mygz / 32.8f;
			
			mygetqval[6] = (float)mymx - 38.0f;
			mygetqval[7] = (float)mymy - 102.5f;
			mygetqval[8] = (float)mymz - 15.5f;
			
			gYroX = mygetqval[3];
			gYroY = mygetqval[4] - (-0.5);
			gYroZ = mygetqval[5];
			
#define Kp 2.0f
#define Ki 0.01f 
#define M_PI  (float)3.1415926535
			static uint32_t lastUpdate, now;
			static float exInt, eyInt, ezInt;

			float norm;
			float hx, hy, hz, bx, bz;
			float vx, vy, vz, wx, wy, wz;
			float ex, ey, ez, halfT;
			float tempq0,tempq1,tempq2,tempq3;

			float q0q0 = q0*q0;
			float q0q1 = q0*q1;
			float q0q2 = q0*q2;
			float q0q3 = q0*q3;
			float q1q1 = q1*q1;
			float q1q2 = q1*q2;
			float q1q3 = q1*q3;
			float q2q2 = q2*q2;   
			float q2q3 = q2*q3;
			float q3q3 = q3*q3;   

			gx = mygetqval[3] * M_PI/180;
			gy = mygetqval[4] * M_PI/180;
			gz = mygetqval[5] * M_PI/180;
			ax = mygetqval[0];
			ay = mygetqval[1];
			az = mygetqval[2];
			mx = mygetqval[6];
			my = mygetqval[7];
			mz = mygetqval[8];		

			now = Get_Time_Micros();  //读取时间 单位是us   
			if(now<lastUpdate)
			{
			//halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
			}
			else	
			{
					halfT =  ((float)(now - lastUpdate) / 2000000.0f);
			}
			lastUpdate = now;	//更新时间
			//快速求平方根算法
			norm = invSqrt(ax*ax + ay*ay + az*az);       
			ax = ax * norm;
			ay = ay * norm;
			az = az * norm;
			//把加计的三维向量转成单位向量。
			norm = invSqrt(mx*mx + my*my + mz*mz);          
			mx = mx * norm;
			my = my * norm;
			mz = mz * norm; 
			// compute reference direction of flux
			hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
			hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
			hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
			bx = sqrt((hx*hx) + (hy*hy));
			bz = hz; 
			// estimated direction of gravity and flux (v and w)
			vx = 2.0f*(q1q3 - q0q2);
			vy = 2.0f*(q0q1 + q2q3);
			vz = q0q0 - q1q1 - q2q2 + q3q3;
			wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
			wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
			wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
			// error is sum of cross product between reference direction of fields and direction measured by sensors
			ex = (ay*vz - az*vy) + (my*wz - mz*wy);
			ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
			ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

			if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
			{
					exInt = exInt + ex * Ki * halfT;
					eyInt = eyInt + ey * Ki * halfT;	
					ezInt = ezInt + ez * Ki * halfT;
					// 用叉积误差来做PI修正陀螺零偏
					gx = gx + Kp*ex + exInt;
					gy = gy + Kp*ey + eyInt;
					gz = gz + Kp*ez + ezInt;
			}
			// 四元数微分方程
			tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
			tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
			tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
			tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

			// 四元数规范化
			norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
			q0 = tempq0 * norm;
			q1 = tempq1 * norm;
			q2 = tempq2 * norm;
			q3 = tempq3 * norm;
			
			float q[4];
			q[0] = q0; //返回当前值
			q[1] = q1;
			q[2] = q2;
			q[3] = q3;
			float angles[3];
			angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw        -pi----pi
			angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch    -pi/2    --- pi/2 
			angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll       -pi-----pi  

			osSemaphoreRelease(refreshGimbalSemaphoreHandle);
//			static int countPrint = 0;
//			if(countPrint > 50){
//				countPrint = 0;
//				
////				fw_printf("mx max = %d | min = %d\r\n", mymaxmx, myminmx);
////				fw_printf("my max = %d | min = %d\r\n", mymaxmy, myminmy);
////				fw_printf("mz max = %d | min = %d\r\n", mymaxmz, myminmz);
////				fw_printf("========================\r\n");
//				
////				fw_printf("now = %d \r\n", now);
////				fw_printf("xxx = %d \r\n", 2147483647);
////				fw_printf("halfT = %f \r\n", halfT);
//				static float last_yaw_temp, yaw_temp;
//				static int yaw_count = 0;
//				last_yaw_temp = yaw_temp;
//				yaw_temp = angles[0]; 
//				if(yaw_temp-last_yaw_temp>=330)  //yaw轴角度经过处理后变成连续的
//				{
//					yaw_count--;
//				}
//				else if (yaw_temp-last_yaw_temp<=-330)
//				{
//					yaw_count++;
//				}
//				float yaw_angle = yaw_temp + yaw_count*360;
//				
////				fw_printf("yaw_angle = %f | ", yaw_angle);
////				fw_printf("angles0 = %f | ", angles[0]);
////				fw_printf("angles1 = %f | ", angles[1]);
////				fw_printf("angles2 = %f\r\n", angles[2]);
////				fw_printf("========================\r\n");
//				
////				fw_printf("ax = %d | ", myax);
////				fw_printf("ay = %d | ", myay);
////				fw_printf("az = %d\r\n", myaz);
////				
////				fw_printf("gx = %d | ", mygx);
////				fw_printf("gy = %d | ", mygy);
////				fw_printf("gz = %d\r\n", mygz);
//			
////				fw_printf("mx = %d | ", mymx);
////				fw_printf("my = %d | ", mymy);
////				fw_printf("mz = %d\r\n", mymz);
////				fw_printf("========================\r\n");
//			}else{
//				countPrint++;
//			}

//			fw_printf("ax = %d | ", ax);
//			fw_printf("ay = %d | ", ay);
//			fw_printf("az = %d\r\n", az);
//			
//			fw_printf("gx = %d | ", gx);
//			fw_printf("gy = %d | ", gy);
//			fw_printf("gz = %d\r\n", gz);
//			
//			fw_printf("mx = %d | ", mx);
//			fw_printf("my = %d | ", my);
//			fw_printf("mz = %d\r\n", mz);
//			fw_printf("========================\r\n");
		}
		//osDelay(500);
	}
}

#ifndef FRAMEWORK_MPU6050_H
#define FRAMEWORK_MPU6050_H

void Init_Quaternion(void);
void mpu6050Init(void);
void printMPU6050Task(void const * argument);
void readMPU6050Task(void const * argument);

void minmax_refresh( int a, int *min, int *max);
void minmax_refresh_f( float a, float *min, float *max);

#endif

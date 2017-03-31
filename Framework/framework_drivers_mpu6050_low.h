#ifndef FRAMEWORK_DRIVERS_MPU6050_LOW_H
#define FRAMEWORK_DRIVERS_MPU6050_LOW_H

void Init_Quaternion(void);
void mpu6050Init(void);
void readMPU6050Task(void const * argument);

#endif

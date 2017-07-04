#ifndef DRIVERS_PLATEMOTOR_H
#define DRIVERS_PLATEMOTOR_H


typedef enum{REVERSE, FORWARD,}RotateDir_e;
void plateMotorInit(void);
void setPlateMotorDir(RotateDir_e dir);
RotateDir_e getPlateMotorDir(void);


#endif

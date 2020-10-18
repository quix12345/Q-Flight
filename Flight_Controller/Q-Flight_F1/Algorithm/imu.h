#include "main.h"
#ifndef _IMU_H_
#define	_IMU_H_

#define GYRO_TO_RAD(x) ((float)x*0.0001331580545) //gyro*250/2^15/57.3
#define GYRO_TO_DEG(x) ((float)x*0.0076293945) //gyro*250/2^15

uint8_t Acc_Calibrate(short ax,short ay,short az,uint8_t enable);
uint8_t Gyro_Calibrate(short gx,short gy,short gz,uint8_t enable);
void IMU_Calibrate(short *ax,short *ay,short *az);
void IMUupdate(short *gx,short *gy,short *gz,short ax,short ay,short az,float *roll,float *pitch,float *yaw);

#endif

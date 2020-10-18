#include "main.h"

#ifndef _HMC5583L_H
#define _HMC5583L_H
#define HMC5883L_Addr 0x3C
#define HMC5883L_DEVICE_ID  0x48
#define MAG_X_BIAS 39.303148
#define MAG_Y_BIAS 13.177245
#define MAG_Z_BIAS -97.100891
#define MAG_X_K 64.940295
#define MAG_Y_K 93.965232
#define MAG_Z_K 110.088758 

void HMC5883L_Init(void);
void HMC5883L_Read(void);
void HMC5883L_StateMachine(void);
#endif

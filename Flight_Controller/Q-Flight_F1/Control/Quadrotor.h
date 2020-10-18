
#ifndef QUADROTOR_H
#define QUADROTOR_H

#include "main.h"
#include "stm32f1xx_hal.h"
#define QUAD_TIM htim1
#define QUAD_IIC hi2c1

struct Quadrotor{

	I2C_HandleTypeDef I2cHandle;
	TIM_HandleTypeDef TimHandle;

};

extern void PWN_Init_Motors(TIM_HandleTypeDef htim);

#endif

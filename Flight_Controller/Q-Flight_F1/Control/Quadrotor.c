#include "Quadrotor.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h>
//void Init_Quadrotor(){
//	struct 	quadrotor *	quadrotor;
////	quadrotor=malloc(sizeof(struct Quadrotor));
////	quadrotor->TimHandle=htim1;
////	quadrotor->I2CHandle=QUAD_IIC;
////	Init_Motors(quadrotor->TimHandle);
//}

void PWN_Init_Motors(TIM_HandleTypeDef htim){
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_4);
}

float velocity_estimator(float acceleration,float dt){
	static float previous_acceleration=0;
	float speed=(acceleration+previous_acceleration)*0.5*dt;
	return speed;
}

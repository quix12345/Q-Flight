#include "main.h"
#include "PWN.h"

void PWN_SetAngle(TIM_HandleTypeDef htim,int angle){
	float target_value = ((float)angle/45)*468.75f + 625;
	__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_1,(int)target_value);
}

void PWN_Init(TIM_HandleTypeDef htim){
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_4);
}

void PWN_SetModulation(TIM_HandleTypeDef htim,uint16_t target_value,uint8_t channel){
	target_value=constrain(target_value,1000,2000);
	switch(channel){
		case 1:
			__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_1,target_value);
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_2,target_value);
			break;
		case 3:
			__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_3,target_value);
			break;
		case 4:
			__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_4,target_value);
			break;
		case 0:
		default:
			__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_1,target_value);
			__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_2,target_value);
			__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_3,target_value);
			__HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_4,target_value);
			break;
	}
	
}

void PWN_Calibrate_all(TIM_HandleTypeDef htim){
	for(int i=1;i<5;i++){
		PWN_SetModulation(htim,2000,i);
	}
	HAL_Delay(3000);
	for(uint16_t pwn_modulation=2000;pwn_modulation>1000;pwn_modulation-=10){
		for(int i=1;i<5;i++){
			PWN_SetModulation(htim,pwn_modulation,i);
		}
		HAL_Delay(10);
	}
	HAL_Delay(500);
}

void PWN_Control_Motors(TIM_HandleTypeDef htim,uint16_t motor1,uint16_t motor2,uint16_t motor3,uint16_t motor4){
	PWN_SetModulation(htim,motor1,1);
	PWN_SetModulation(htim,motor2,2);
	PWN_SetModulation(htim,motor3,3);
	PWN_SetModulation(htim,motor4,4);
}

void PWN_Motor_Force_Stop(TIM_HandleTypeDef htim){
//	PWN_SetModulation(htim,1000,1);
//	PWN_SetModulation(htim,1000,2);
//	PWN_SetModulation(htim,1000,3);
//	PWN_SetModulation(htim,1000,4);
	PWN_SetModulation(htim,1000,0);
}

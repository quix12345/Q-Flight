#include "main.h"

#ifndef __PWN_H__
void PWN_SetAngle(TIM_HandleTypeDef htim,int angle);
void PWN_Init(TIM_HandleTypeDef htim);
void PWN_SetModulation(TIM_HandleTypeDef htim,uint16_t target_value,uint8_t channel);
void PWN_Calibrate_all(TIM_HandleTypeDef htim);
void PWN_Motor_Force_Stop(TIM_HandleTypeDef htim);
void PWN_Control_Motors(TIM_HandleTypeDef htim,uint16_t motor1,uint16_t motor2,uint16_t motor3,uint16_t motor4);
#endif

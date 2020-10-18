/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fifo.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include "mpu9250.h"
#include "ahrs.h"
#include "PWN.h"
#include "PID.h"
#include "Quadrotor.h"
#include "fifo.h"
#include "Verbose.h"
#include "SysSetting.h"
#include "math.h"
#include "maths.h"
#include "bmp280.h"
#include "filter2.h"
#include "HMC5883L.h"
#include "vl53lxx.h"
#include "MS5611.h"
#include "LC306.h"
#include "Utils.h"
#include "RemoteControl.h"
#include "tim.h"
#include "SensorFusion.h"
#include "Control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint8_t content[UART_CONTENT_BUFFER_SIZE];
extern uint8_t Uart_Flag,LC306_Uart_Flag;
extern float throttle,pitch_expect,roll_expect,yaw_expect;
extern float roll_offset,pitch_offset;
extern float control_max_angle;
extern Fifo_TypeDef remote_control_fifo;
extern int verbose;
extern uint8_t Light_Toggle;
extern int num_of_commands,num_of_success_commands;
extern float quad_control_period;
extern uint8_t Control_Mode;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NIGHT_LED_Pin GPIO_PIN_14
#define NIGHT_LED_GPIO_Port GPIOB
#define WORK_INDICATE_Pin GPIO_PIN_15
#define WORK_INDICATE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

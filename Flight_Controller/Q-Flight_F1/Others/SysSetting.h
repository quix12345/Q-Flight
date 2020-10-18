#ifndef __SYSSETTING_H
#define __SYSSETTING_H

#include "main.h"
//Index
#define FLASH_VERIFY_INDEX 0
#define MAX_CONTROL_ANGLE_INDEX 49
#define LIGHT_TOGGLE_INDEX 50
#define VERBOSE_INDEX 51
#define GYRO_X_OFFSET 52
#define GYRO_Y_OFFSET 53
#define GYRO_Z_OFFSET 54
#define PITCH_OFFSET_INDEX 55
#define ROLL_OFFSET_INDEX 56

//Values
#define FLASH_VERIFY 25
#define PITCH_OFFSET 0
#define ROLL_OFFSET 0
#define QUAD_PARAMETER_LENGTH 57


extern uint32_t Quad_Parameter[QUAD_PARAMETER_LENGTH];

void Flash_Setting_Init(void);
#endif

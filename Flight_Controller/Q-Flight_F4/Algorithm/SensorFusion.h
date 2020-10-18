#include "main.h"

#ifndef __SENSORFUSION_H
#define __SENSORFUSION_H

#define IS_MS5611_PRESENT 1
#define IS_VL53LXX_PRESENT 1
#define IS_LC306_PRESENT 1
#define IS_HMC5883L_PRESENT 0
#define IS_US_PRESENT 0
#define USE_OPT_FLOW_ONLY 1
extern uint8_t is_take_off;
extern float baro_offset;
extern float calc_height,calc_velocity,calc_acc;
void SensorInit(void);
void Altitude_Update(void);
void Altitude_Estimate(void);
void Velocity_Estimate(void);
void Aceleration_Estimate(void);
void Altitude_Calc(void);
void Position_Calc(float delta_time);
#endif



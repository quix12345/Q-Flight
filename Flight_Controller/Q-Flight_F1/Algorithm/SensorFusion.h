#include "main.h"

#ifndef __SENSORFUSION_H
#define __SENSORFUSION_H

extern uint8_t is_take_off;
extern float calc_height;

void SensorInit(void);
void Altitude_Update(void);
void Altitude_Estimate(void);

#endif



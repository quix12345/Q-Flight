#ifndef __Utils_H
#define __Utils_H

#include "main.h"
typedef struct{
	float roll;
	float pitch;
	float yaw;
}attitude_t;

typedef struct{
	Vector3 velocity;
	Vector3 position;
	attitude_t attitude;
	Vector3 acc;
	Vector3 raw_acc;
	Vector3 mag;
	Vector3 gyro;
}quadstate_t;

//float constrainf(float value,const float min,const float max);
uint8_t BCC_Check(uint8_t *aubData_p, uint8_t auwDataLength);
void Vector2f_Init(Vector2f * obj);
void Vector3_Init(Vector3 * obj);
void Attitude_Init(attitude_t * obj);
void Quadstate_Init(quadstate_t * obj);
int math_sign(float number);
float Convert2PercentagePWN(uint16_t Raw_PWN);

#endif

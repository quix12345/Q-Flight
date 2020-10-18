#include "Utils.h"

//float constrainf(float value,const float min,const float max){
//	if(value>=max){
//		return max;
//	}
//	else if(value<=min){
//		return min;
//	}
//	else{
//		return value;
//	}
//}

uint8_t BCC_Check(uint8_t *aubData_p, uint8_t auwDataLength)
{
	uint8_t aubChecksum = 0;
	uint8_t auwCnt = 0;

	while(auwCnt < auwDataLength)
	{
			aubChecksum ^= aubData_p[auwCnt];
			auwCnt++;
	}
	return aubChecksum;
}

void Vector2f_Init(Vector2f * obj){
	obj->x=0.0f;
	obj->y=0.0f;
}

void Vector3_Init(Vector3 * obj){
	obj->x=0.0f;
	obj->y=0.0f;
	obj->z=0.0f;
}

void Attitude_Init(attitude_t * obj){
	obj->roll=0.0f;
	obj->pitch=0.0f;
	obj->yaw=0.0f;
}

void Quadstate_Init(quadstate_t * obj){
	Vector3_Init(&(obj->position));
	Vector3_Init(&(obj->velocity));
	Attitude_Init(&(obj->attitude));
	Vector3_Init(&(obj->acc));
	Vector3_Init(&(obj->mag));
	Vector3_Init(&(obj->gyro));
	Vector3_Init(&(obj->raw_acc));
}

int math_sign(float number){
	if(number>=0){
		return 1;
	}
	else{
		return -1;
	}
}

float Convert2PercentagePWN(uint16_t Raw_PWN){
	if(Raw_PWN<1000){
		Raw_PWN=1000;
	}
	else if(Raw_PWN>2000){
			Raw_PWN=2000;
	}
	return (float)(Raw_PWN-1000)/1000.0;
}

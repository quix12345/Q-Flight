#include "main.h"

#ifndef __AHRS_H
#define __AHRS_H

#define GRAVITY_CONSTANT 1
#define ANGLE_TO_RADIAN 0.017453f
#define ACC_XYZ_MAX  8192.0f
#define AcceGravity 9.81f

#define SAMPLE_HALF_T 0.005f     //采样周期的一半，单位：s

#define GYRO_G 	0.0610351f				//角速度变成度/S   此参数对应陀螺2000度每秒  Gyro_G=1/16.375=0.0610687
#define GYRO_GR	0.0010653f				//角速度变成弧度/S	此参数对应陀螺2000度每秒

#define IMU_KP 3.0f    					//比例
#define IMU_KI 0.0005f 						//积分

#define IMPACT_ACC_LIMIT 3500.0f

typedef struct
{
	float x;
	float y;
	float z;
}Vector3;

typedef struct
{
	float X;
	float Y;
	float Z;
}S_FLOAT_XYZ;

extern volatile S_FLOAT_XYZ Angle;
extern float euler_angles[3];
extern void Prepare_Data(void);
extern void IMU_Update(void);
extern void AHRS_Date_Init(void);
extern int16_t accData[3],gyroData[3],gyroRawData[3],magData[3];
extern float invSqrt(float x);
extern float Sin_Pitch,Cos_Pitch,Sin_Roll,Cos_Roll,Sin_Yaw,Cos_Yaw;
extern float acc_velocity[3],acc_position[3],real_accData[3];
extern float module_AccData;
void Attitude_Update(void);
void ImuComputeRotationMatrix(void);
void Vector_From_EarthFrame2BodyFrame(Vector3 *ef,Vector3 *bf);
void Vector_From_BodyFrame2EarthFrame(Vector3 *bf,Vector3 *ef);
void Calc_RealAccData(void);
void Impact_Detect(void);

#endif


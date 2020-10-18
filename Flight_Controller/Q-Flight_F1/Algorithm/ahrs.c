#include 	"ahrs.h"
#include 	"stdio.h"
#include  "math.h"    
#include 	"main.h"
#include "mpu9250.h"

float Sin_Pitch,Cos_Pitch,Sin_Roll,Cos_Roll,Sin_Yaw,Cos_Yaw=0;
int16_t accData[3],gyroData[3],magData[3],gyroRawData[3];
float euler_angles[3],gravity_offset[3],real_accData[3];
float velocity[3]={0,0,0};
float position[3]={0,0,0};
float ex_int = 0, ey_int = 0, ez_int = 0;   //X、Y、Z轴的比例误差
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    //定义四元素
float q0_yaw = 1, q1_yaw = 0, q2_yaw = 0, q3_yaw = 0;    //弥补Mahony算法在无地磁情况解算Yaw轴满足不了大扰动要求的现象

float rotationMatrix[3][3];
void Vector_From_BodyFrame2EarthFrame(Vector3 *bf,Vector3 *ef)
{
  ef->x=rotationMatrix[0][0]*bf->x+rotationMatrix[0][1]*bf->y+rotationMatrix[0][2]*bf->z;
  ef->y=rotationMatrix[1][0]*bf->x+rotationMatrix[1][1]*bf->y+rotationMatrix[1][2]*bf->z;
  ef->z=rotationMatrix[2][0]*bf->x+rotationMatrix[2][1]*bf->y+rotationMatrix[2][2]*bf->z;
}

void Vector_From_EarthFrame2BodyFrame(Vector3 *ef,Vector3 *bf)
{
  bf->x=rotationMatrix[0][0]*ef->x+rotationMatrix[1][0]*ef->y+rotationMatrix[2][0]*ef->z;
  bf->y=rotationMatrix[0][1]*ef->x+rotationMatrix[1][1]*ef->y+rotationMatrix[2][1]*ef->z;
  bf->z=rotationMatrix[0][2]*ef->x+rotationMatrix[1][2]*ef->y+rotationMatrix[2][2]*ef->z;
}

void ImuComputeRotationMatrix(void)
{
  Sin_Pitch=sin(euler_angles[1]* ANGLE_TO_RADIAN);
  Cos_Pitch=cos(euler_angles[1]* ANGLE_TO_RADIAN);
  Sin_Roll=sin(-euler_angles[0]* ANGLE_TO_RADIAN);
  Cos_Roll=cos(-euler_angles[0]* ANGLE_TO_RADIAN);
  Sin_Yaw=sin(-euler_angles[2]* ANGLE_TO_RADIAN);
	Cos_Yaw=cos(-euler_angles[2]* ANGLE_TO_RADIAN);
  
  rotationMatrix[0][0]=Cos_Yaw* Cos_Roll;
  rotationMatrix[0][1]=Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw;
  rotationMatrix[0][2]=Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw;
  
  rotationMatrix[1][0]=Sin_Yaw * Cos_Roll;
  rotationMatrix[1][1]=Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw;
  rotationMatrix[1][2]=Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw;
  
  rotationMatrix[2][0]=-Sin_Roll;
  rotationMatrix[2][1]= Sin_Pitch * Cos_Roll;
  rotationMatrix[2][2]= Cos_Pitch * Cos_Roll;
}


void Position_Update(){
	Vector3 Body_Frame,Earth_Frame;
	Body_Frame.x=accData[1];//X_Origion;//Acce_Control[0];
  Body_Frame.y=accData[0];//Y_Origion;//Acce_Control[1];
  Body_Frame.z=accData[2];//Z_Origion;//Acce_Control[2];
	if(sqrt(accData[0]*accData[0]+accData[1]*accData[1]+accData[2]*accData[2])>=100000){
		return;
	}
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  Earth_Frame.z*=AcceGravity/ACC_XYZ_MAX;
  Earth_Frame.z-=AcceGravity;//减去重力加速度
  Earth_Frame.z*=100;//加速度cm/s^2
  Earth_Frame.x*=AcceGravity/ACC_XYZ_MAX;
  Earth_Frame.x*=100;//加速度cm/s^2
  Earth_Frame.y*=AcceGravity/ACC_XYZ_MAX;
  Earth_Frame.y*=100;//加速度cm/s^2
	real_accData[0]=Earth_Frame.x;
	real_accData[1]=Earth_Frame.y;
	real_accData[2]=Earth_Frame.z;
	float dt=(float)quad_control_period/1000;
	static float velocity_old[3];
	for(int i=0;i<3;i++){
		if(fabs(real_accData[i]*dt)<=0.3){
			real_accData[i]=0;
		}
		velocity[i]+=real_accData[i]*dt;
		position[i]+=velocity_old[i]*dt+0.5f*real_accData[i]*pow(dt,2);
		velocity_old[i]=velocity[i];
	}
	
	// printf("%.2f %.2f %.2f\r\n",real_accData[0],real_accData[1],real_accData[2]);
	//	printf("%.2f %.2f %.2f  %.2f %.2f %.2f\r\n",position[0]/100,position[1]/100,position[2]/100,velocity[0],velocity[1],velocity[2]);
}


void Attitude_Update(){
	MPU9250_AccRead(accData);
	MPU9250_GyroRead(gyroData);
	IMU_Update();
	ImuComputeRotationMatrix();
	Position_Update();
}


void IMU_Update(void)
{
  //float norm;	
	float norm_inv;
	float gx = gyroData[0]	*	GYRO_GR,gy = gyroData[1]	*	GYRO_GR,gz = gyroData[2]	*	GYRO_GR;
	float ax = accData[0],ay =accData[1],az = accData[2];
	
	float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q1q1 = q1 * q1;
  float q1q3 = q1 * q3;
  float q2q2 = q2	* q2;
  float q2q3 = q2	*	q3;
  float q3q3 = q3	*	q3;	
	float vx, vy, vz;
  float ex, ey, ez;
	
	float	q0_yawq0_yaw = q0_yaw * q0_yaw;
	float	q1_yawq1_yaw = q1_yaw * q1_yaw;
	float	q2_yawq2_yaw = q2_yaw * q2_yaw;
	float	q3_yawq3_yaw = q3_yaw * q3_yaw;
	float	q1_yawq2_yaw = q1_yaw * q2_yaw;
	float	q0_yawq3_yaw = q0_yaw * q3_yaw;
	
//**************************Yaw轴计算******************************
	
	//Yaw轴四元素的微分方程
	float sample_half_t=(float)quad_control_period/2000;
  q0_yaw = q0_yaw + (-q1_yaw * gx - q2_yaw * gy - q3_yaw * gz) * sample_half_t;
  q1_yaw = q1_yaw + (q0_yaw * gx + q2_yaw * gz - q3_yaw * gy) * sample_half_t;
  q2_yaw = q2_yaw + (q0_yaw * gy - q1_yaw * gz + q3_yaw * gx) * sample_half_t;
  q3_yaw = q3_yaw + (q0_yaw * gz + q1_yaw * gy - q2_yaw * gx) * sample_half_t;
	
	//规范化Yaw轴四元数
  // norm = sqrt(q0_yawq0_yaw + q1_yawq1_yaw + q2_yawq2_yaw + q3_yawq3_yaw);
	norm_inv=invSqrt(q0_yawq0_yaw + q1_yawq1_yaw + q2_yawq2_yaw + q3_yawq3_yaw);
  q0_yaw = q0_yaw * norm_inv;
  q1_yaw = q1_yaw * norm_inv;
  q2_yaw = q2_yaw * norm_inv;
  q3_yaw = q3_yaw * norm_inv;
	
  
	if(ax * ay * az	== 0)
	return ;
	
	//规范化加速度计值
  //  norm = sqrt(ax * ax + ay * ay + az * az); 
	norm_inv=invSqrt(ax * ax + ay * ay + az * az);
  ax = ax * norm_inv;
  ay = ay * norm_inv;
  az = az * norm_inv;
	
	//估计重力方向和流量/变迁
  vx = 2 * (q1q3 - q0q2);											
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
  //向量外积再相减得到差分就是误差
  ex = (ay * vz - az * vy) ;      
  ey = (az * vx - ax * vz) ;
  ez = (ax * vy - ay * vx) ;

	//对误差进行PI计算
  ex_int = ex_int + ex * IMU_KI;			
  ey_int = ey_int + ey * IMU_KI;
  ez_int = ez_int + ez * IMU_KI;

  //校正陀螺仪
  gx = gx + IMU_KP * ex + ex_int;					
  gy = gy + IMU_KP * ey + ey_int;
  gz = gz + IMU_KP * ez + ez_int;			
			
	//四元素的微分方程
  q0 = q0 + (-q1 * gx - q2	*	gy - q3	*	gz)	*	sample_half_t;
  q1 = q1 + (q0	*	gx + q2	*	gz - q3	*	gy)	*	sample_half_t;
  q2 = q2 + (q0	*	gy - q1	*	gz + q3	*	gx)	*	sample_half_t;
  q3 = q3 + (q0	*	gz + q1	*	gy - q2	*	gx)	*	sample_half_t;

  //规范化Pitch、Roll轴四元数
  //norm = sqrt(q0q0 + q1q1 + q2q2 + q3q3);
	norm_inv=invSqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 * norm_inv;
  q1 = q1 * norm_inv;
  q2 = q2 * norm_inv;
  q3 = q3 * norm_inv;
	
	//求解欧拉角
	euler_angles[0] = atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1) * 57.3; //roll
	euler_angles[1] = asin(-2 * q1q3 + 2 * q0q2) * 57.3; //pitch
	euler_angles[2] = atan2(2 * q1_yawq2_yaw + 2 * q0_yawq3_yaw, -2 * q2_yawq2_yaw - 2 * q3_yawq3_yaw + 1)	* 57.3; //yaw
}

/*
 * 函数名：AHRS_Date_Init
 * 描述  ：航姿数据初始化
 * 输入  ：无
 * 输出  ：无
 */ 
void AHRS_Date_Init(void)
{	
	q0 = 1;
	q1 = 0;
	q2 = 0;
	q3 = 0;
	
	q0_yaw = 1;
	q1_yaw = 0;
	q2_yaw = 0;
	q3_yaw = 0;

//	Angle.X = 0;
//	Angle.Y = 0;
//	Angle.Z = 0;
//	
//	Exp_Angle.X = 0;
//	Exp_Angle.Y = 0;
//	Exp_Angle.Z = 0;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)	/*快速开平方求倒*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}







#include "main.h"

float Sin_Pitch,Cos_Pitch,Sin_Roll,Cos_Roll,Sin_Yaw,Cos_Yaw=0;
int16_t accData[3],gyroData[3],magData[3],gyroRawData[3];
float euler_angles[3],real_accData[3],real_accDataOffset[3];
float module_AccData=0;
float sum_real_accData[3]={0,0,0};
float acc_velocity[3]={0,0,0};
float acc_position[3]={0,0,0};
float ex_int = 0, ey_int = 0, ez_int = 0;   //X、Y、Z轴的比例误差
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    //定义四元素
float q0_yaw = 1, q1_yaw = 0, q2_yaw = 0, q3_yaw = 0;    //弥补Mahony算法在无地磁情况解算Yaw轴满足不了大扰动要求的现象
float rotationMatrix[3][3];
biquadFilter_t acc_velocity_Filter[2];
extern osMutexId RC_USART_MutexHandle;
extern quadstate_t current_state;
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
	Sin_Roll=sin(euler_angles[0]* ANGLE_TO_RADIAN);
  Cos_Roll=cos(euler_angles[0]* ANGLE_TO_RADIAN);
  Sin_Pitch=sin(euler_angles[1]* ANGLE_TO_RADIAN);
  Cos_Pitch=cos(euler_angles[1]* ANGLE_TO_RADIAN);
  Sin_Yaw=sin(euler_angles[2]* ANGLE_TO_RADIAN);
	Cos_Yaw=cos(euler_angles[2]* ANGLE_TO_RADIAN);
	rotationMatrix[0][0]=Cos_Yaw* Cos_Pitch;
	rotationMatrix[0][1]=Sin_Pitch*Sin_Roll*Cos_Yaw-Sin_Pitch * Sin_Yaw;
	rotationMatrix[0][2]=Sin_Roll * Sin_Yaw+Cos_Roll * Sin_Pitch * Cos_Yaw;
	rotationMatrix[1][0]=Sin_Yaw * Cos_Pitch;
	rotationMatrix[1][1]=Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Roll * Cos_Yaw;
	rotationMatrix[1][2]=Cos_Roll * Sin_Pitch * Sin_Yaw - Sin_Roll * Cos_Yaw;
	rotationMatrix[2][0]=-Sin_Pitch;
	rotationMatrix[2][1]= Sin_Roll * Cos_Pitch;
	rotationMatrix[2][2]= Cos_Pitch * Cos_Roll;
}

void Calc_RealAccData(){
	Vector3 Body_Frame,Earth_Frame;
	Body_Frame.x=accData[0];//X_Origion;//Acce_Control[0];
  Body_Frame.y=accData[1];//Y_Origion;//Acce_Control[1];
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
	real_accData[0]=-Earth_Frame.x; //转换为全局导航大地坐标系
	real_accData[1]=-Earth_Frame.y;
	real_accData[2]=Earth_Frame.z;
}

void Calc_RealAccData_Offset(){
	for(uint8_t cnt=0;cnt<100;cnt++){
		MPU9250_AccRead(accData);
		MPU9250_GyroRead(gyroData);
		IMU_Update();
	}
	for(uint8_t cnt=0;cnt<20;cnt++){
		MPU9250_AccRead(accData);
		MPU9250_GyroRead(gyroData);
		IMU_Update();
		ImuComputeRotationMatrix();
		Calc_RealAccData();
		sum_real_accData[0]+=real_accData[0];
		sum_real_accData[1]+=real_accData[1];
		sum_real_accData[2]+=real_accData[2];
	}
	real_accDataOffset[0]=sum_real_accData[0]/20;
	real_accDataOffset[1]=sum_real_accData[1]/20;
	real_accDataOffset[2]=sum_real_accData[2]/20;
}

void Position_Update(){
	Calc_RealAccData();
	real_accData[0]=real_accData[0]-real_accDataOffset[0];
	real_accData[1]=real_accData[1]-real_accDataOffset[1];
	real_accData[2]=real_accData[2]-real_accDataOffset[2];
	if(Control_Mode>=FIXED_ALTITUDE_MODE){ // 在没有开启传感器的情况下关闭惯性导航
		float dt=(float)quad_control_period/1000;
		static float velocity_old[3],acc_old[3]={0,0,0};
		if(!USE_OPT_FLOW_ONLY){
			acc_velocity[0]=current_state.velocity.x;
			acc_position[0]=current_state.position.x;
			
			acc_velocity[1]=current_state.velocity.y;
			acc_position[1]=current_state.position.y;
		}
		acc_velocity[2]=current_state.velocity.z;
		acc_position[2]=current_state.position.z;
		for(int i=0;i<3;i++){
			if(fabs(real_accData[i])<=5){
				real_accData[i]=0;
			}
			acc_velocity[i]+=0.5f*(acc_old[i]+real_accData[i])*dt;
			if(i<2){
				acc_velocity[i]=biquadFilterApply(&acc_velocity_Filter[i],acc_velocity[i]);
			}
			if(i==2){
				if(fabs(acc_velocity[i])<=0.3){
					acc_velocity[i]=0;
				}
			}
			else{
				if(fabs(acc_velocity[i])<=0.4){
					acc_velocity[i]=0;
				}
			}
			// acc_position[i]+=velocity_old[i]*dt+0.5f*real_accData[i]*pow(dt,2);
			acc_position[i]+=0.5f*(acc_velocity[i]+velocity_old[i])*dt;
			velocity_old[i]=acc_velocity[i];
			acc_old[i]=real_accData[i];
			
			//更新至当前状态
			current_state.position.z=acc_position[2];
			current_state.velocity.z=acc_velocity[2];
			if(!USE_OPT_FLOW_ONLY){
				current_state.position.y=acc_position[1];
				current_state.velocity.y=acc_velocity[1];
				current_state.position.x=acc_position[0];
				current_state.velocity.x=acc_velocity[0];
			}
		}
	}
}


void Attitude_Update(){
	MPU9250_AccRead(accData);
	MPU9250_GyroRead(gyroData);
	IMU_Update();
//	if(fabs(euler_angles[1])<=60){
//		euler_angles[1]
//	}
	ImuComputeRotationMatrix();
  Position_Update();
	Impact_Detect();
}

void Impact_Detect(){
	module_AccData=sqrt(real_accData[0]*real_accData[0]+real_accData[1]*real_accData[1]+real_accData[2]*real_accData[2]);
	if(module_AccData>=IMPACT_ACC_LIMIT){
		uint8_t Is_Enable_FreeRTOS_Again=0;
		if(Enable_FreeRTOS){
			osThreadSuspendAll();
			Enable_FreeRTOS=0;
			Is_Enable_FreeRTOS_Again=1;
		}
		PWN_Motor_Force_Stop(MOTO_TIM);
		printf("检测到撞击!!\r\n");
		Work_Indicate(1);
		HAL_Delay(1000);
		if(Is_Enable_FreeRTOS_Again){
			Enable_FreeRTOS=1;
			osThreadResumeAll();
		}
	}
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
	euler_angles[0] = atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1) * 57.3f; //roll
	euler_angles[1] = asin(-2 * q1q3 + 2 * q0q2) * 57.3f; //pitch
	euler_angles[2] = atan2(2 * q1_yawq2_yaw + 2 * q0_yawq3_yaw, -2 * q2_yawq2_yaw - 2 * q3_yawq3_yaw + 1)	* 57.3f; //yaw
	// printf("%.2f %.2f %.2f\r\n",euler_angles[0],euler_angles[1],euler_angles[2]);
}


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
	
	Calc_RealAccData_Offset();
	biquadFilterInitLPF(&acc_velocity_Filter[0],15,quad_control_period*1000);
	biquadFilterInitLPF(&acc_velocity_Filter[1],15,quad_control_period*1000);
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







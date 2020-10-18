#include "imu.h"
#include "maths.h"
#define Kp 2.0f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.00f                	// 0.001  integral gain governs rate of convergence of gyroscope biases
#define T 0.02

#define IMU_INTEGRAL_LIM  ( 2.0f *DEG_TO_RAD )
//用于校准的数据误差
float ax_cali=0,ay_cali=0,az_cali=0;
short gx_cali=0,gy_cali=0,gz_cali=0;
//测出加速度计误差并保存待用，50次取平均，校准过程中须保持水平
//返回值，0:校准未完成，1：校准完成
//自创的向量校正方法
uint8_t Acc_Calibrate(short ax,short ay,short az,uint8_t enable)
{
	static uint8_t i=0;
	float norm;
	static s32 axcount=0;
	static s32 aycount=0;
	static s32 azcount=0;
	float array_x,array_y,array_z;
	if(enable!=1)
	{
		i=0;
		axcount=0;aycount=0;azcount=0;
		return 0;
	}
	else if(i<50)
	{
		axcount+=ax;aycount+=ay;azcount+=az;
		i++;
		return 0;
	}
	else
	{
		array_x=axcount/50.0;array_y=aycount/50.0;array_z=azcount/50.0;//实际向量
		norm=Q_rsqrt(array_x*array_x+array_y*array_y+array_z*array_z);//均方倒数归一化
		array_x*=norm;array_y*=norm;array_z*=norm;//实际单位向量
		ax_cali-=array_x;//以下为误差校正向量，即实际单位向量和单位重力向量之间的向量差
		ay_cali-=array_y;
		az_cali+=1-array_z;
		i=0;
		axcount=0;aycount=0;azcount=0;
		return 1;
	}
}
//测出陀螺仪误差并保存待用，50次取平均，校准过程中须保持静止
//返回值 0:校准未完成，1：校准完成
uint8_t Gyro_Calibrate(short gx,short gy,short gz,uint8_t enable)
{
	static uint8_t i=0;
	static s32 gxcount=0;
	static s32 gycount=0;
	static s32 gzcount=0;
	if(enable!=1)
	{
		i=0;
		gxcount=0;gycount=0;gzcount=0;
		return 0;
	}
	else if(i<50)
	{
		gxcount+=gx;gycount+=gy;gzcount+=gz;
		i++;
		return 0;
	}
	else
	{
		gxcount/=50;gycount/=50;gzcount/=50;
		gx_cali+=gxcount;
		gy_cali+=gycount;
		gz_cali+=gzcount;
		i=0;
		gxcount=0;gycount=0;gzcount=0;
		return 1;
	}
}
//用保存的误差参数校正加速度计原始数据
//陀螺仪的误差参数另外作为卡尔曼滤波的状态变量在滤波中校正而不在此函数中
void IMU_Calibrate(short *ax,short *ay,short *az)
{
	short accx=*ax;
	short accy=*ay;
	short accz=*az;
	float norm;
	//用校准参数校正加速度计原始数据
	norm=my_sqrt(accx*accx+accy*accy+accz*accz);
	accx+=ax_cali*norm;
	accy+=ay_cali*norm;
	accz+=az_cali*norm;
	*ax = accx;
	*ay = accy;
	*az = accz;
}
float sigma_a=0.01,sigma_g=0.01;//陀螺仪驱动噪声方差和加速度计观测噪声方差
//卡尔曼增益
float K_roll[2];
float K_pitch[2];
//最小均方误差矩阵,M[n|n]或M[n-1|n-1]
float mmse_roll[2][2] = { { 1, 0 },{ 0, 1 } };
float mmse_pitch[2][2] = { { 1, 0 },{ 0, 1 } };
//最小预测均方误差矩阵,M[n|n-1]
float mPmse_roll[2][2];
float mPmse_pitch[2][2];
//六轴融合卡尔曼滤波算法
void IMUupdate(short *gx,short *gy,short *gz,short ax,short ay,short az,float *roll,float *pitch,float *yaw)
{
	float temp;//为减少计算量而暂时保存数据，无实际意义
	float roll_temp,pitch_temp;//状态变量预测值,s[n|n-1]
	//预测
	*gx-=gx_cali;
	*gy-=gz_cali;
	*gz-=gz_cali;
	*yaw+=GYRO_TO_DEG(*gz)*T;
	roll_temp=*roll+GYRO_TO_DEG(*gx)*T;
	pitch_temp=*pitch+GYRO_TO_DEG(*gy)*T;
	//最小预测MSE
	mPmse_roll[0][0]=mmse_roll[0][0]+(mmse_roll[1][1]+sigma_g)*T*T-(mmse_roll[0][1]+mmse_roll[1][0])*T;
	mPmse_roll[0][1]=mmse_roll[0][1]-mmse_roll[1][1]*T;
	mPmse_roll[1][0]=mmse_roll[1][0]-mmse_roll[1][1]*T;
	mPmse_roll[1][1]=mmse_roll[1][1];
	mPmse_pitch[0][0]=mmse_pitch[0][0]+(mmse_pitch[1][1]+sigma_g)*T*T-(mmse_pitch[0][1]+mmse_pitch[1][0])*T;
	mPmse_pitch[0][1]=mmse_pitch[0][1]-mmse_pitch[1][1]*T;
	mPmse_pitch[1][0]=mmse_pitch[1][0]-mmse_pitch[1][1]*T;
	mPmse_pitch[1][1]=mmse_pitch[1][1];
	//卡尔曼增益
	K_roll[0]=mPmse_roll[0][0]/(mPmse_roll[0][0]+sigma_a);
	K_roll[1]=mPmse_roll[1][0]/(mPmse_roll[0][0]+sigma_a);
	K_pitch[0]=mPmse_pitch[0][0]/(mPmse_pitch[0][0]+sigma_a);
	K_pitch[1]=mPmse_pitch[1][0]/(mPmse_pitch[0][0]+sigma_a);
	//修正
	temp=fast_atan2(ay,az)*180/PI-roll_temp;
	*roll=roll_temp+K_roll[0]*temp;
	gx_cali=gx_cali+K_roll[1]*temp;
	temp=fast_atan2(-ax,az)*180/PI-pitch_temp;
	*pitch=pitch_temp+K_pitch[0]*temp;
	gy_cali=gy_cali+K_pitch[1]*temp;
	//最小MSE
	mmse_roll[0][0]=mPmse_roll[0][0]-K_roll[0]*mPmse_roll[0][0];
	mmse_roll[0][1]=mPmse_roll[0][1]-K_roll[0]*mPmse_roll[0][1];
	mmse_roll[1][0]=mPmse_roll[1][0]-K_roll[1]*mPmse_roll[0][0];
	mmse_roll[1][1]=mPmse_roll[1][1]-K_roll[1]*mPmse_roll[0][1];
	mmse_pitch[0][0]=mPmse_pitch[0][0]-K_pitch[0]*mPmse_pitch[0][0];
	mmse_pitch[0][1]=mPmse_pitch[0][1]-K_pitch[0]*mPmse_pitch[0][1];
	mmse_pitch[1][0]=mPmse_pitch[1][0]-K_pitch[1]*mPmse_pitch[0][0];
	mmse_pitch[1][1]=mPmse_pitch[1][1]-K_pitch[1]*mPmse_pitch[0][1];
}

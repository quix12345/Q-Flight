#include "main.h"
#include "i2c.h"
#include "mpu9250.h"
#include "filter2.h"
#include <stdio.h>
#include "ahrs.h"

int16_t gyro_offset[3];
float acc_temp[3];
biquadFilter_t my_Filters[6];
 
uint8_t MPU9250_WriteByte(uint8_t reg,uint8_t data)
{
	if(HALIIC_WriteByteToSlave(MPU9250Addr,reg,data))
		return 1;
	else
		return 0;

}

uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(HALIIC_ReadByteFromSlave(MPU9250Addr,reg,buf))
		return 1;
	else
		return 0;
}


uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_WriteMultByteToSlave(MPU9250Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_ReadMultByteFromSlave(MPU9250Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

void MPU9250_AccRead(int16_t *accData)
{
    uint8_t buf[6];
   	MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H,6,buf);
//		acc_temp[0]=biquadFilterApply(&my_Filters[3],((float)(-((buf[0] << 8) | buf[1]))-ACC_X_BIAS)/ACC_X_K);
//		acc_temp[1]=biquadFilterApply(&my_Filters[4],((float)((buf[2] << 8) | buf[3])-ACC_Y_BIAS)/ACC_Y_K);
//		acc_temp[2]=biquadFilterApply(&my_Filters[5],((float)((buf[4] << 8) | buf[5])-ACC_Z_BIAS)/ACC_Z_K);
//		accData[0] = (int16_t)(acc_temp[0]*ACC_XYZ_MAX);
//    accData[1] = (int16_t)(acc_temp[1]*ACC_XYZ_MAX);
//    accData[2] = (int16_t)(acc_temp[2]*ACC_XYZ_MAX);
		accData[0]=biquadFilterApply(&my_Filters[3],-((buf[0] << 8) | buf[1]));
		accData[1]=biquadFilterApply(&my_Filters[4],(buf[2] << 8) | buf[3]);
		accData[2]=biquadFilterApply(&my_Filters[5],(buf[4] << 8) | buf[5]);
}

void MPU9250_GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
		static int err_times=0;
		while(MPU9250_ReadMultBytes(MPU9250_RA_GYRO_XOUT_H, 6, buf)){
			err_times++;
			printf("mpu9250 error!\r\n");
			if(err_times>500){
				HAL_NVIC_SystemReset();
			}
		}
		err_times=0;
		gyroRawData[0]=(int16_t)((buf[0] << 8) | buf[1])-gyro_offset[0];
		gyroRawData[1]=(-(int16_t)((buf[2] << 8) | buf[3]))-gyro_offset[1];
		gyroRawData[2]=(-(int16_t)((buf[4] << 8) | buf[5]))-gyro_offset[2];
		for(int i=0;i<3;i++){
			if(abs(gyroRawData[i])<10){
				gyroRawData[i]=0;
			}
		}
    gyroData[0] = biquadFilterApply(&my_Filters[0],gyroRawData[0]);
    gyroData[1] = biquadFilterApply(&my_Filters[1],gyroRawData[1]);
    gyroData[2] =  biquadFilterApply(&my_Filters[2],gyroRawData[2]);
}

void MPU9250_MagRead(int16_t *magData)
{
    uint8_t buf[6];
    HALIIC_WriteByteToSlave(MPU9250Addr,0x37,0x02);//turn on Bypass Mode
    HAL_Delay(10);
    HALIIC_WriteByteToSlave(AK8963_ADDR,0x0A,0x11);
    HAL_Delay(10);
    HALIIC_ReadMultByteFromSlave(AK8963_ADDR,MAG_XOUT_L, 6, buf);
    magData[0] = (int16_t)((buf[1] << 8) | buf[0]) ;
    magData[1] = (int16_t)((buf[3] << 8) | buf[2]) ;
    magData[2] = (int16_t)((buf[5] << 8) | buf[4]) ;
}

void MPU9250_TempRead(float *tempdata)
{
	uint8_t buf[2];
	short data;
	MPU9250_ReadMultBytes(MPU9250_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	*tempdata = 21.0 + ((float)data/333.87f);
}

uint8_t MPU9250_Check(){
	uint8_t buf;
	MPU9250_ReadByte(MPU9250_RA_WHO_AM_I, &buf);//????ID
	if(buf==0x68||buf==0x71){
		return 1;
	}
	printf("Mpu9250 init error!\r\n");
	HAL_Delay(10);
	HAL_NVIC_SystemReset();
	return 0;
}

void Filter_Init(){
	for(int i=0;i<sizeof(my_Filters)/sizeof(biquadFilter_t);i++){
		biquadFilterInitLPF(&my_Filters[i],10,quad_control_period*1000);
	}
}

void MPU9250_Init(void)
{
	Filter_Init();
	while(!MPU9250_Check()); //????ID,??MPU9250????
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x80); //??MPU9250
	HAL_Delay(100);
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x00); //??MPU9250,??????x?PLL???? (MPU9250_RA_PWR_MGMT_1, 0x01)
	MPU9250_WriteByte(MPU9250_RA_INT_ENABLE, 0x00); //????
	MPU9250_WriteByte(MPU9250_RA_GYRO_CONFIG, 0x18); //??????+-2000?/? (????? = 2^15/2000 = 16.4LSB/?/?
	MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG, 0x08); //??????+-4g   (????? = 2^15/4g = 8196LSB/g )
	MPU9250_WriteByte(MPU9250_RA_CONFIG, MPU9250_DLPF_BW_256);//????????1kHZ,DLPF=20Hz
	MPU9250_WriteByte(MPU9250_RA_SMPLRT_DIV, 0x00);  //???? (???? = ??????? / (1+DIV),????1000hz)
	MPU9250_WriteByte(MPU9250_RA_INT_PIN_CFG, 0x02); //MPU ?????MPU9250??I2C
}

float Calc_Var(float Temp[MEAN_ARRAY_SIZE]){
		double mean=0;
		long sum=0;
		for(int i=0;i<MEAN_ARRAY_SIZE;i++){
			sum+=Temp[i];
		}
		mean=(float)sum/MEAN_ARRAY_SIZE;
		double var_sum=0;
		float var=0;
		for(int i=0;i<MEAN_ARRAY_SIZE;i++){
			var_sum+=pow((float)(Temp[i]-mean),2);
		}
    var=(double)var_sum/MEAN_ARRAY_SIZE;
	return var;
}


float Calc_Mean(int16_t gyroDataTemp[][CALC_STD_ARRAY_SIZE]){
	int16_t total_sum=0;
	for(int j=0;j<3;j++){
		for(int i=0;i<CALC_STD_ARRAY_SIZE;i++){
			total_sum+=abs(gyroDataTemp[j][i]);
		}
	}
	float mean=(float)total_sum/(3*CALC_STD_ARRAY_SIZE);
	return mean;
}

void MPU9250_CalcGyroOffset(){
int16_t gyroDataTemp[3][CALC_STD_ARRAY_SIZE];
	float meanTemp[10];
	float Var=0;
	for(int i=0;i<=500;i++){
		MPU9250_AccRead(accData);
		MPU9250_GyroRead(gyroData);
//		printf("gyro: %d %d %d\r\n",gyroData[0],gyroData[1],gyroData[2]);
	}
	do{
		for(int z=0;z<10;z++){
			for(int i=0;i<CALC_STD_ARRAY_SIZE;i++){
				MPU9250_GyroRead(gyroData);
				for(int j=0;j<3;j++){
					gyroDataTemp[j][i]=gyroData[j];
				}
			}
			meanTemp[z]=Calc_Mean(gyroDataTemp);
		}
		Var = Calc_Var(meanTemp);
	}
	while(Var>=0.8);
	MPU9250_GyroRead(gyro_offset);
	printf("Offsets: %d %d %d\r\n",gyro_offset[0],gyro_offset[1],gyro_offset[2]);
}



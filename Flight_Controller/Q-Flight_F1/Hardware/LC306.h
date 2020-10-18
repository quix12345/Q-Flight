#include "main.h"
#include "math.h"
#ifndef _LC306_H
#define _LC306_H
#define SENSOR_IIC_ADDR 0xdc
#define LC306_USART USART3
extern int16_t pixel_flow_x_integral,pixel_flow_y_integral,integration_timespan,qual;   
extern float speed_x,speed_y,sum_flow_x,sum_flow_y;
uint8_t BCC_Check(uint8_t *aubData_p, uint8_t auwDataLength);
void LC306_Config_Init_Uart(void);

typedef struct{
unsigned short frame_count_since_last_readout;
signed short pixel_flow_x_integral;
signed short pixel_flow_y_integral;
signed short gyro_x_rate_integral;
signed short gyro_y_rate_integral;
signed short gyro_z_rate_integral;
unsigned int integration_timespan;
unsigned int sonar_timestamp;
unsigned short ground_distance;
signed short gyro_temperature;
unsigned char qual;
}flow_integral_frame;

typedef struct{
float x;
float y;
unsigned short dt;
unsigned char qual;
unsigned char update;
}flow_float;


typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;


typedef struct
{
  float x;
  float y;
}Vector2f;

extern Vector2f opt_filter_data; //滤波后的像素位移
extern Vector2f opt_gyro_data;   //光流角速度
extern Vector2f opt_speed_data,opt_position_data;

uint8_t LC306_Config_Init(void);
float Optflow_GyroFusion(float input_opt_gyro,float mpu_gyro);
void Optflow_Prase(void);
void OpticalFlow_Init(void);
void Vector2f_Init(Vector2f * obj);
void Optflow_Init(void);
void Optflow_precess(void);
#endif

#include "main.h"
#include "math.h"
#ifndef _MS5611_H
#define _MS5611_H
#include "stdio.h"

//MS5611 ADDR CSB接地 
#define MS5611_SLAVE_ADDR 0XEE

//COMMAND
#define MS5611_CMD_REST 0X1E 
#define MS5611_CMD_CONVERT_D1_256 0X40
#define MS5611_CMD_CONVERT_D1_512 0X42
#define MS5611_CMD_CONVERT_D1_1024 0X44
#define MS5611_CMD_CONVERT_D1_2048 0X46
#define MS5611_CMD_CONVERT_D1_4096 0X48
#define MS5611_CMD_CONVERT_D2_256 0X50
#define MS5611_CMD_CONVERT_D2_512 0X52
#define MS5611_CMD_CONVERT_D2_1024 0X54
#define MS5611_CMD_CONVERT_D2_2048 0X56
#define MS5611_CMD_CONVERT_D2_4096 0X58

#define MS6511_ADC_READ 0X00

#define MS5611_PROM_READ_0 0XA0
#define MS5611_PROM_READ_1 0XA2
#define MS5611_PROM_READ_2 0XA4
#define MS5611_PROM_READ_3 0XA6
#define MS5611_PROM_READ_4 0XA8
#define MS5611_PROM_READ_5 0XAA
#define MS5611_PROM_READ_6 0XAC
#define MS5611_PROM_READ_7 0XAE


#define PA_OFFSET_INIT_NUM 10
typedef struct 
{
	uint16_t C[6];	//用于补偿温度和气压数据
	uint16_t *reserve;
	uint16_t *crc;	
	unsigned int D[2];	//存放读取的气压，温度数据
	signed long long dT;	//Difference between actual and reference temperature
	signed long long OFF;	//Offset at actual temperature
	signed long long SENS;	//Sensitivity at actual temperature
	signed int TEMP;	//Actual temperature
	signed int P;	//温度补充气压
	float Altitude;
	float Pressure_offset;
}MS5611_t;

extern MS5611_t *ms5611_handle;
extern float baro_height;
extern uint8_t Baro_enabled;

void MS5611_Reset(void);
void MS5611_Baro_update(void);
void Read_MS5611_Offset(void);
float getEstimatedAltitude(float baroPress);
void MS5611_Init(void);
void MS5611_read_temp(void);
void MS5611_read_press(void);
void MS5611_calculate(void);
void MS5611_Update(void);
void MS5611_StateMachine(void);
#endif

#include "MS5611.h"

float baro_height=0;
uint8_t Baro_enabled=0;

//50hz  2hz
const static float b_baro_pressure[3]={0.01335920002786,  0.02671840005571,  0.01335920002786};
const static float a_baro_pressure[3]={1,   -1.647459981077,   0.7008967811884};
float LPButter_BaroAlt(float curr_input)
{
  static float input[3];
  static float output[3];
  /* 气压计高度Butterworth滤波 */
  /* 获取最新x(n) */
  input[2] = curr_input;
  /* Butterworth滤波 */
  output[2] = b_baro_pressure[0] * input[2] + b_baro_pressure[1] * input[1]
    + b_baro_pressure[2] * input[0] - a_baro_pressure[1] * output[1] - a_baro_pressure[2] * output[0];
  
  /* x(n) 序列保存 */
  input[0] = input[1];
  input[1] = input[2];
  /* y(n) 序列保存 */
  output[0] = output[1];
  output[1] = output[2];
  return output[2];
}
// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************
MS5611_t *ms5611_handle;
void MS5611_Send(uint8_t buf){
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_SLAVE_ADDR,&buf,1,0x10);
}

uint8_t MS5611_ReadMultBytes(uint8_t reg, void *buf, uint8_t size) {
  if(HALIIC_ReadMultByteFromSlave(MS5611_SLAVE_ADDR,reg,size,buf))
		return 1;
	else
		return 0;
}

uint8_t MS5611_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_WriteMultByteToSlave(MS5611_SLAVE_ADDR,reg,len,buf))
		return 1;
	else
		return 0;
}

uint8_t MS5611_WriteByte(uint8_t reg, uint8_t val) {
if(HALIIC_WriteByteToSlave(MS5611_SLAVE_ADDR,reg,val))
		return 1;
	else
		return 0;
}

uint8_t MS5611_ReadByte(uint8_t reg) {
  uint8_t val;
  if(HALIIC_ReadByteFromSlave(MS5611_SLAVE_ADDR,reg,&val))
		return val;
	else
		return 0;
}



void MS5611_StateMachine(void)
{
  static uint8_t MS5611_status=0;
	static uint16_t MS5611_ReadyCNT=0;
	if(MS5611_ReadyCNT<=400){
		// 400*10ms=4s
		MS5611_ReadyCNT++;
	}
	else{
		Baro_enabled=1;
	}
  MS5611_status++;
  if(MS5611_status == 1)
  {
		//仅执行单次
		MS5611_Send(MS5611_CMD_CONVERT_D2_4096);
  }
  else if(MS5611_status == 2)
  {
    uint8_t buf[3];
		MS5611_ReadMultBytes(MS6511_ADC_READ,&buf,3);
		ms5611_handle->D[1]=(buf[0] << 16) | ((buf[1] << 8) | buf[2]);
		
		MS5611_Send(MS5611_CMD_CONVERT_D1_4096);
  }
  else if(MS5611_status == 3)
  {
   	uint8_t buf[3];
		MS5611_ReadMultBytes(MS6511_ADC_READ,&buf,3);
		ms5611_handle->D[0]=(buf[0] << 16) | ((buf[1] << 8) | buf[2]);
		MS5611_calculate();
		ms5611_handle->Altitude=LPButter_BaroAlt(getEstimatedAltitude(ms5611_handle->P));
		baro_height=ms5611_handle->Altitude;
		MS5611_Send(MS5611_CMD_CONVERT_D2_4096);
		MS5611_status=1;
  }
}

void Read_MS5611_Offset(void)
{
  uint16_t i=0;
	uint16_t MS5611_Cnter=0;
	uint16_t paInitCnt=0;
	double paOffsetNum = 0;
  while(MS5611_Cnter<=10)
  {
    MS5611_Cnter++;
    MS5611_Baro_update();
  }
  
  for(i=0;i<PA_OFFSET_INIT_NUM;i++)
  {
    MS5611_Baro_update();
    paOffsetNum+=ms5611_handle->P;
    paInitCnt++;
  }
  ms5611_handle->Pressure_offset=paOffsetNum/PA_OFFSET_INIT_NUM;
}

/*
 * 参数MS5611结构体
*/
void ms5611_handle_init()
{
	//指针必须初始化
	ms5611_handle = (MS5611_t *)malloc(sizeof(MS5611_t));
	ms5611_handle->reserve = (	uint16_t *)malloc(sizeof(	uint16_t));
	ms5611_handle->crc = (	uint16_t *)malloc(sizeof(	uint16_t));
	
	ms5611_handle->C[0] = 0;
	ms5611_handle->C[1] = 0;
	ms5611_handle->C[2] = 0;
	ms5611_handle->C[3] = 0;
	ms5611_handle->C[4] = 0;
	ms5611_handle->C[5] = 0;
	
	ms5611_handle->D[0] = 0;
	ms5611_handle->D[1] = 0;
	
	ms5611_handle->TEMP = 0;
	ms5611_handle->P = 0;
	ms5611_handle->SENS = 0;
	ms5611_handle->OFF = 0;
	ms5611_handle->Altitude=0;
	ms5611_handle->Pressure_offset=0;
}

/*
 * 重启ms5611
 */
void MS5611_Reset()
{
	MS5611_Send(MS5611_CMD_REST);
	HAL_Delay(4);
}

/*
*  读取prom的内容
*/
void MS5611_Init()
{
	ms5611_handle_init();
	MS5611_Reset();
	uint8_t buf[5];
	MS5611_ReadMultBytes(MS5611_PROM_READ_0,&buf,2);
	*ms5611_handle->reserve=(buf[0] << 8) | buf[1];
	memset(buf,0x00,sizeof(buf));
	MS5611_ReadMultBytes(MS5611_PROM_READ_1,&buf,2);
	ms5611_handle->C[0]=(buf[0] << 8) | buf[1];
	memset(buf,0x00,sizeof(buf));
	MS5611_ReadMultBytes(MS5611_PROM_READ_2,&buf,2);
	ms5611_handle->C[1]=(buf[0] << 8) | buf[1];
	memset(buf,0x00,sizeof(buf));
	MS5611_ReadMultBytes(MS5611_PROM_READ_3,&buf,2);
	ms5611_handle->C[2]=(buf[0] << 8) | buf[1];
	memset(buf,0x00,sizeof(buf));
	MS5611_ReadMultBytes(MS5611_PROM_READ_4,&buf,2);
	ms5611_handle->C[3]=(buf[0] << 8) | buf[1];
	memset(buf,0x00,sizeof(buf));
	MS5611_ReadMultBytes(MS5611_PROM_READ_5,&buf,2);
	ms5611_handle->C[4]=(buf[0] << 8) | buf[1];
	memset(buf,0x00,sizeof(buf));
	MS5611_ReadMultBytes(MS5611_PROM_READ_6,&buf,2);
	ms5611_handle->C[5]=(buf[0] << 8) | buf[1];
	memset(buf,0x00,sizeof(buf));
	MS5611_ReadMultBytes(MS5611_PROM_READ_7,&buf,2);
	*ms5611_handle->crc=(buf[0] << 8) | buf[1];
	memset(buf,0x00,sizeof(buf));
	HAL_Delay(50);
	Read_MS5611_Offset();
}

/*
 * 读取温度 转换精度4096
*/
void MS5611_read_temp()
{
	MS5611_Send(MS5611_CMD_CONVERT_D2_4096);//发送转换命令
	HAL_Delay(9);//等待转换
	uint8_t buf[3];
	MS5611_ReadMultBytes(MS6511_ADC_READ,&buf,3);
	ms5611_handle->D[1]=(buf[0] << 16) | ((buf[1] << 8) | buf[2]);
}

/*
 * 读取气压 转换精度4096
*/
void MS5611_read_press()
{
	MS5611_Send(MS5611_CMD_CONVERT_D1_4096);//发送转换命令
	HAL_Delay(9);
	uint8_t buf[3];
	MS5611_ReadMultBytes(MS6511_ADC_READ,&buf,3);
	ms5611_handle->D[0]=(buf[0] << 16) | ((buf[1] << 8) | buf[2]);
}

/*
 * 修正气压和温度
*/
void MS5611_calculate()
{
	int64_t dT = 0,TEMP = 0,T2 = 0,OFF = 0,OFF2 = 0,SENS2 = 0,SENS = 0;
	
	dT = ms5611_handle->D[1] - ((int64_t) (ms5611_handle->C[4])<<8);
	TEMP = 2000 + ((int64_t) (dT*(ms5611_handle->C[5]))>>23);
	//低于20°时：
	if(TEMP < 2000 && TEMP > -1500)
	{
		T2 = ( dT*dT )>>31;
		OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 2;
		SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 4;
	}
	OFF = (((int64_t)(ms5611_handle->C[1])) << 16) + (((ms5611_handle->C[3]) * dT) >> 7);
	SENS = (((int64_t)(ms5611_handle->C[0])) << 15) + (((ms5611_handle->C[2]) * dT) >> 8);
	OFF -= OFF2;
	SENS-=SENS2;
	ms5611_handle->dT = dT;
	ms5611_handle->OFF = OFF;
	ms5611_handle->TEMP -= T2;
	ms5611_handle->SENS = SENS;
	ms5611_handle->P = ((((ms5611_handle->D[0]) * (ms5611_handle->SENS))>>21) - (ms5611_handle->OFF))>>15;
}

float getEstimatedAltitude(float baroPress)
{
	// 单位 cm
  double Tempbaro=(float)(baroPress/ms5611_handle->Pressure_offset)*1.0;
  float Altitude = 4433000.0f * (1 - powf((float)(Tempbaro),0.190295f));
  return Altitude;
}

void MS5611_Baro_update(){
	MS5611_read_temp();
	MS5611_read_press();
	MS5611_calculate();
}

void MS5611_Update(){
	MS5611_Baro_update();
	ms5611_handle->Altitude=LPButter_BaroAlt(getEstimatedAltitude(ms5611_handle->P));
	baro_height=ms5611_handle->Altitude;
}




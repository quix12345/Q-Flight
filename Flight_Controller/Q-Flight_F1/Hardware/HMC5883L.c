#include "HMC5883L.h"

biquadFilter_t HMC_Filters[3];

uint8_t HMC_WriteByte(uint8_t reg,uint8_t data)
{
	if(HALIIC_WriteByteToSlave(HMC5883L_Addr,reg,data))
		return 1;
	else
		return 0;

}

uint8_t HMC_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(HALIIC_ReadByteFromSlave(HMC5883L_Addr,reg,buf))
		return 1;
	else
		return 0;
}


uint8_t HMC_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_WriteMultByteToSlave(HMC5883L_Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

uint8_t HMC_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_ReadMultByteFromSlave(HMC5883L_Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

void HMC5883L_Init(void)
{
	uint8_t buf;
  HMC_ReadByte(0x0A,&buf);
  if(buf== HMC5883L_DEVICE_ID){
		HMC_WriteByte(0x00,0x78);//配置寄存器A：采样平均数1 输出速率75Hz 正常测量
		HMC_WriteByte(0x01,0xE0);//配置寄存器B：增益控制
		HMC_WriteByte(0x02,0x00);//模式寄存器：连续测量模式
		
		for(int i=0;i<3;i++){
			biquadFilterInitLPF(&HMC_Filters[i],80,quad_control_period*1000);
		}
	}
	else{
		printf("HMC5883L init error!\r\n");
		HAL_NVIC_SystemReset();
	}
}

void HMC5883L_Read(void)
{
  int x,y,z;
//  float x1,y1,z1;
  unsigned char buf[6];
//  float angle;
  uint8_t status;
  HMC_ReadByte(0x09,&status);
  if ((status & 0x01) == 0x01)
  {
		HMC_ReadMultBytes(0x03,6,buf);
    x=(buf[0] << 8) | buf[1]; //Combine MSB and LSB of X Data output register
    z=(buf[2] << 8) | buf[3]; //Combine MSB and LSB of Z Data output register
    y=(buf[4] << 8) | buf[5];
    if(x>0x7fff)  x-=0xffff;
    if(y>0x7fff)  y-=0xffff;
    if(z>0x7fff)  z-=0xffff;
//		x1=(float)(x-MAG_X_BIAS)/MAG_X_K;
//		y1=(float)(y-MAG_Y_BIAS)/MAG_Y_K;
//		z1=(float)(z-MAG_Z_BIAS)/MAG_Z_K;
//		x=x1*8192;
//		y=y1*8192;
//		z=z1*8192;
		magData[0]=biquadFilterApply(&HMC_Filters[0],x);
		magData[1]=biquadFilterApply(&HMC_Filters[1],y);
		magData[2]=biquadFilterApply(&HMC_Filters[0],z);
  }
}

void HMC5883L_StateMachine(void)
{
	static uint8_t HMC5883L_Sample_Cnt=0;
  HMC5883L_Sample_Cnt++;
  if(HMC5883L_Sample_Cnt==2)
  {
    HMC_WriteByte(0x00,0x78);//配置寄存器A：采样平均数1 输出速率75Hz 正常测量
  }
  else if(HMC5883L_Sample_Cnt>=4)
  {
    HMC5883L_Read();
    HMC5883L_Sample_Cnt=0;
  }
}

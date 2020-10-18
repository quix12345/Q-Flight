#include "main.h"
#include "LC306.h"

uint8_t OpticalFlow_enabled=0;
uint8_t lc306_content[20];
uint8_t content_temp[20];
int16_t pixel_flow_x_integral,pixel_flow_y_integral,integration_timespan,qual=0;    
float speed_x,speed_y,sum_flow_x,sum_flow_y,opt_height=0;
extern quadstate_t target_state,current_state;
static void SensorConfig_UartSend(uint8_t dat)
{
	while (!LL_USART_IsActiveFlag_TXE(LC306_USART));
	LL_USART_TransmitData8(LC306_USART,dat);
	while (!LL_USART_IsActiveFlag_TC(LC306_USART));
}
		
/**************************************************/		
void LC306_Config_Init_Uart()
{
	const uint8_t tab_focus[4] = {0x96,0x26,0xbc,0x50};		
	const uint8_t Sensor_cfg[]={
//地址, 数据
0x12, 0x80, 
0x11, 0x30, 
0x1b, 0x06, 
0x6b, 0x43, 
0x12, 0x20, 
0x3a, 0x00, 
0x15, 0x02, 
0x62, 0x81, 
0x08, 0xa0, 
0x06, 0x68, 
0x2b, 0x20, 
0x92, 0x25, 
0x27, 0x97, 
0x17, 0x01, 
0x18, 0x79, 
0x19, 0x00, 
0x1a, 0xa0, 
0x03, 0x00, 
0x13, 0x00, 
0x01, 0x13, 
0x02, 0x20, 
0x87, 0x16, 
0x8c, 0x01, 
0x8d, 0xcc, 
0x13, 0x07, 
0x33, 0x10, 
0x34, 0x1d, 
0x35, 0x46, 
0x36, 0x40, 
0x37, 0xa4, 
0x38, 0x7c, 
0x65, 0x46, 
0x66, 0x46, 
0x6e, 0x20, 
0x9b, 0xa4, 
0x9c, 0x7c, 
0xbc, 0x0c, 
0xbd, 0xa4, 
0xbe, 0x7c, 
0x20, 0x09, 
0x09, 0x03, 
0x72, 0x2f, 
0x73, 0x2f, 
0x74, 0xa7, 
0x75, 0x12, 
0x79, 0x8d, 
0x7a, 0x00, 
0x7e, 0xfa, 
0x70, 0x0f, 
0x7c, 0x84, 
0x7d, 0xba, 
0x5b, 0xc2, 
0x76, 0x90, 
0x7b, 0x55, 
0x71, 0x46, 
0x77, 0xdd, 
0x13, 0x0f, 
0x8a, 0x10, 
0x8b, 0x20, 
0x8e, 0x21, 
0x8f, 0x40, 
0x94, 0x41, 
0x95, 0x7e, 
0x96, 0x7f, 
0x97, 0xf3, 
0x13, 0x07, 
0x24, 0x58, 
0x97, 0x48, 
0x25, 0x08, 
0x94, 0xb5, 
0x95, 0xc0, 
0x80, 0xf4, 
0x81, 0xe0, 
0x82, 0x1b, 
0x83, 0x37, 
0x84, 0x39, 
0x85, 0x58, 
0x86, 0xff, 
0x89, 0x15, 
0x8a, 0xb8, 
0x8b, 0x99, 
0x39, 0x98, 
0x3f, 0x98, 
0x90, 0xa0, 
0x91, 0xe0, 
0x40, 0x20, 
0x41, 0x28, 
0x42, 0x26, 
0x43, 0x25, 
0x44, 0x1f, 
0x45, 0x1a, 
0x46, 0x16, 
0x47, 0x12, 
0x48, 0x0f, 
0x49, 0x0d, 
0x4b, 0x0b, 
0x4c, 0x0a, 
0x4e, 0x08, 
0x4f, 0x06, 
0x50, 0x06, 
0x5a, 0x56, 
0x51, 0x1b, 
0x52, 0x04, 
0x53, 0x4a, 
0x54, 0x26, 
0x57, 0x75, 
0x58, 0x2b, 
0x5a, 0xd6, 
0x51, 0x28, 
0x52, 0x1e, 
0x53, 0x9e, 
0x54, 0x70, 
0x57, 0x50, 
0x58, 0x07, 
0x5c, 0x28, 
0xb0, 0xe0, 
0xb1, 0xc0, 
0xb2, 0xb0, 
0xb3, 0x4f, 
0xb4, 0x63, 
0xb4, 0xe3, 
0xb1, 0xf0, 
0xb2, 0xa0, 
0x55, 0x00, 
0x56, 0x40, 
0x96, 0x50, 
0x9a, 0x30, 
0x6a, 0x81, 
0x23, 0x33, 
0xa0, 0xd0, 
0xa1, 0x31, 
0xa6, 0x04, 
0xa2, 0x0f, 
0xa3, 0x2b, 
0xa4, 0x0f, 
0xa5, 0x2b, 
0xa7, 0x9a, 
0xa8, 0x1c, 
0xa9, 0x11, 
0xaa, 0x16, 
0xab, 0x16, 
0xac, 0x3c, 
0xad, 0xf0, 
0xae, 0x57, 
0xc6, 0xaa, 
0xd2, 0x78, 
0xd0, 0xb4, 
0xd1, 0x00, 
0xc8, 0x10, 
0xc9, 0x12, 
0xd3, 0x09, 
0xd4, 0x2a, 
0xee, 0x4c, 
0x7e, 0xfa, 
0x74, 0xa7, 
0x78, 0x4e, 
0x60, 0xe7, 
0x61, 0xc8, 
0x6d, 0x70, 
0x1e, 0x39, 
0x98, 0x1a
};
	
	
	uint16_t i;
	uint16_t len ;
//	uint8_t recv[3];
//	int recv_cnt;
	HAL_Delay(150);	
	len = sizeof(Sensor_cfg);
	
//0xAA指令
	SensorConfig_UartSend(0xAA);

//0xAB指令			
	SensorConfig_UartSend(0xAB);		 
	SensorConfig_UartSend(tab_focus[0]);		
	SensorConfig_UartSend(tab_focus[1]);
	SensorConfig_UartSend(tab_focus[2]);
	SensorConfig_UartSend(tab_focus[3]);
	SensorConfig_UartSend(tab_focus[0]^tab_focus[1]^tab_focus[2]^tab_focus[3]);			 
		 
//	recv_cnt = 0;
//	while(recv_cnt<3)  //如接收不到模块返回的三个数，可延时10ms从0xAA指令开始重新配置
//	{
//		while(!LL_USART_IsActiveFlag_RXNE(LC306_USART)); 
//		recv[recv_cnt++]=LL_USART_ReceiveData8(LC306_USART);		
//	}	
			
//	if(((recv[0]^recv[1]) == recv[2])&&(recv[1] == 0x00)){
//		printf("AB Command configuration successconfig succefful\n");
//	}
			
//0xBB指令							
	for(i=0; i<len;i+=2 )
	{
		SensorConfig_UartSend(0xBB);		 
		SensorConfig_UartSend(SENSOR_IIC_ADDR);		
		SensorConfig_UartSend(Sensor_cfg[i]);
		SensorConfig_UartSend(Sensor_cfg[i+1]);
		SensorConfig_UartSend(SENSOR_IIC_ADDR^Sensor_cfg[i]^Sensor_cfg[i+1]);
		 
//		recv_cnt = 0;
//		while(recv_cnt<3)  //如接收不到模块返回的三个数，可延时1ms重新发送0xBB指令
//		{	
//			while(!LL_USART_IsActiveFlag_RXNE(LC306_USART)); 
//			recv[recv_cnt++]=LL_USART_ReceiveData8(LC306_USART);
//			if(((recv[0]^recv[1]) == recv[2]) && (recv[1] == 0x00)){
//				printf("BB Command configuration successconfig succefful\n");			
//			}			
//		}
	}
//0xDD			 
	SensorConfig_UartSend(0xDD);	
	HAL_Delay(100);
	Optflow_Init();
	OpticalFlow_enabled=1;
	LL_USART_EnableIT_RXNE(LC306_USART);
	LL_USART_EnableIT_PE(LC306_USART);
}			 

#define M_PI_F 3.141592653589793f
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}


//入口:	float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  static int LPF_Cnt=0;
  Buffer->Input_Butter[2]=curr_input;
  if(LPF_Cnt>=100)
  {
    /* Butterworth滤波 */
    Buffer->Output_Butter[2]=
      Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
          +Parameter->b[2] * Buffer->Input_Butter[0]
            -Parameter->a[1] * Buffer->Output_Butter[1]
              -Parameter->a[2] * Buffer->Output_Butter[0];
  }
  else
  {
    Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
    LPF_Cnt++;
  }
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}


float opticalflow_high=1000;//默认1m=100cm=1000mm
float filter_data_x,filter_data_y=0;
flow_float opt_data;
Vector2f opt_filter_data; //滤波后的像素位移
Vector2f opt_gyro_data;   //光流角速度
Vector2f opt_gyro_filter_data;//光流经过旋转互补滤波后的角速度
Vector2f gyro_filter_data;//低通同步相位的角速度
Vector2f opt_speed_data,opt_position_data;
Butter_Parameter OpticalFlow_Parameter,OpticalFlow_Gyro_Parameter;
Butter_BufferData Buffer_OpticalFlow[2],Buffer_OpticalFlow_Gyro[2];
void  OpticalFlow_CF(float flow_height,Vector2f accel,Vector2f flow);
void OpticalFlow_Init()
{
  Set_Cutoff_Frequency(50, 20,&OpticalFlow_Parameter);//20
  Set_Cutoff_Frequency(50, 10,&OpticalFlow_Gyro_Parameter);//同步相位//10
}

void Optflow_Init(){
	Vector2f_Init(&opt_speed_data);
	Vector2f_Init(&opt_position_data);
	Vector2f_Init(&opt_gyro_data);
	Vector2f_Init(&opt_gyro_filter_data);
	Vector2f_Init(&opt_filter_data);
	Vector2f_Init(&gyro_filter_data);
	OpticalFlow_Init();
}

void Optflow_Prase()//50hz
{   
	if(OpticalFlow_enabled){
	static uint8_t head_toggle=0;
	static uint8_t pointer=0;
	static uint8_t tmp_old;
	uint8_t tmp;
	if(LL_USART_IsActiveFlag_RXNE(USART3)) //????????
	{
		tmp=LL_USART_ReceiveData8(USART3);   //??????????
		if(pointer>=16){
				memset(lc306_content,0x00,sizeof(lc306_content));
				pointer=0;
		}
		if(tmp==0x0A&&tmp_old==0xFE){
			pointer=0;
			lc306_content[pointer++]=0xFE;
			head_toggle=1;
		}
		if(head_toggle){
			lc306_content[pointer++]=tmp;
			if(lc306_content[pointer-1]==0x55&&lc306_content[pointer-4]==0xF5){	
				for(int i=2;i<11;i++){
					content_temp[i-2]=lc306_content[i];
				}
				uint8_t sum=BCC_Check(content_temp,pointer-4);
				if(sum==lc306_content[pointer-2]){
					//10m以上则关闭光流模块
					if(calc_height<=1000){
						LC306_Uart_Flag=1;
					}
					pixel_flow_x_integral=(int16_t)(lc306_content[3]<<8)|lc306_content[2];
					pixel_flow_y_integral=(int16_t)(lc306_content[5]<<8)|lc306_content[4];
//			  uint16_t pixel_temp;
//			  pixel_temp=pixel_flow_x_integral;
//				pixel_flow_x_integral=pixel_flow_y_integral;
//				pixel_flow_y_integral=-pixel_temp;
					
					if(abs(pixel_flow_x_integral)<=2||abs(pixel_flow_x_integral)>=100000){
						pixel_flow_x_integral=0;
					}
					if(abs(pixel_flow_y_integral)<=2||abs(pixel_flow_x_integral)>=100000){
						pixel_flow_y_integral=0;
					}
					integration_timespan= ((int16_t)(lc306_content[7]<<8)|lc306_content[6]);
					qual= (int16_t)lc306_content[12]; 
					opt_height=current_state.position.z/100;
//					opt_height=0.65;
					speed_x = (opt_height*pixel_flow_x_integral/ (integration_timespan*0.000001))/100;
					speed_y = (opt_height*pixel_flow_y_integral/(integration_timespan*0.000001))/100;
				  sum_flow_x += speed_x*(integration_timespan*0.000001); 
				  sum_flow_y += speed_y*(integration_timespan*0.000001);
					opt_filter_data.x=LPButterworth(pixel_flow_x_integral,&Buffer_OpticalFlow[0],&OpticalFlow_Parameter);
					opt_filter_data.y=LPButterworth(pixel_flow_y_integral,&Buffer_OpticalFlow[1],&OpticalFlow_Parameter);   
					//printf("%.2f %.2f   %.2f %.2f   %.2f %d %d \r\n",sum_flow_x,sum_flow_y,speed_x,speed_y,laser_height,pixel_flow_x_integral,pixel_flow_y_integral);
					
					Optflow_precess();
				}
				head_toggle=0;
				pointer=0;
				memset(lc306_content,0x00,sizeof(lc306_content));
			}
		}
		tmp_old=tmp;
	}
}
}

float Optflow_GyroFusion(float input_opt_gyro,float mpu_gyro){
	return input_opt_gyro-constrainf(mpu_gyro,-3.14,3.14);
}

void Optflow_precess(){
//	static int cnt =0;
//	if(cnt++>=4){
	opt_gyro_data.x=(opt_filter_data.x)/200.0f;//光流角速度rad/s
	opt_gyro_data.y=(opt_filter_data.y)/200.0f;//光流角速度rad/s         
	gyro_filter_data.x=LPButterworth((float)gyroData[0]*GYRO_G,&Buffer_OpticalFlow_Gyro[0],&OpticalFlow_Gyro_Parameter)/57.3f;//陀螺仪相位同步角速度 roll
	gyro_filter_data.y=LPButterworth((float)gyroData[1]*GYRO_G,&Buffer_OpticalFlow_Gyro[1],&OpticalFlow_Gyro_Parameter)/57.3f;//陀螺仪相位同步角速度 pitch
	opt_gyro_filter_data.x=Optflow_GyroFusion(opt_gyro_data.x,gyro_filter_data.x);//光流角速度与陀螺仪角速度融合 
	opt_gyro_filter_data.y=Optflow_GyroFusion(opt_gyro_data.y,gyro_filter_data.y); //光流角速度与陀螺仪角速度融合 
	//printf("%.2f %.2f %.2f\r\n",opt_gyro_data.y,gyro_filter_data.y,opt_gyro_filter_data.y);
	opt_filter_data.x=opt_gyro_filter_data.x*200.0f;
	opt_filter_data.y=opt_gyro_filter_data.y*200.0f;
	opt_speed_data.x=(opt_height*opt_filter_data.x/ (integration_timespan*0.000001))/100;
	opt_speed_data.y=(opt_height*opt_filter_data.y/ (integration_timespan*0.000001))/100;
	opt_position_data.x += speed_x*(integration_timespan*0.000001); 
	opt_position_data.y += speed_y*(integration_timespan*0.000001);
	
	current_state.position.x=-opt_position_data.x;
	current_state.position.y=opt_position_data.y;
	//printf("%.2f %.2f  %.2f %.2f \r\n",opt_position_data.x,sum_flow_x,opt_position_data.y,sum_flow_y);
//	cnt=2;
//	}
}

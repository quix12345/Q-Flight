#include "SensorFusion.h"

extern quadstate_t current_state;
float baro_offset,calc_height=0;
uint8_t is_take_off=0;
biquadFilter_t Alt_Filter;


const static float b_pressure[3]={0.01335920002786,  0.02671840005571,  0.01335920002786};
const static float a_pressure[3]={1,   -1.647459981077,   0.7008967811884};
float LPButter_Alt(float curr_input)
{
  static float input[3];
  static float output[3];
  /* 获取最新x(n) */
  input[2] = curr_input;
  /* Butterworth滤波 */
  output[2] = b_pressure[0] * input[2] + b_pressure[1] * input[1]
    + b_pressure[2] * input[0] - a_pressure[1] * output[1] - a_pressure[2] * output[0];
  
  /* x(n) 序列保存 */
  input[0] = input[1];
  input[1] = input[2];
  /* y(n) 序列保存 */
  output[0] = output[1];
  output[1] = output[2];
  return output[2];
}

void SensorInit(){
	// HMC5883L_Init();
	if(Control_Mode!=REPORT_ONLY_MODE&&Control_Mode!=MANUAL_MODE){
		vl53l0x_Init();
		MS5611_Init();
		LC306_Config_Init_Uart();
		biquadFilterInitLPF(&Alt_Filter,15,quad_control_period*1000);
	}
}

void Altitude_Update(){
	vl53l0x_StateMachine();
	MS5611_StateMachine();
}

void FlightStatus_Update(){
	uint16_t pwm_moto_avg=(pwm_moto1+pwm_moto2+pwm_moto3+pwm_moto4)/4;
	if(LaserLidar_enabled&&pwm_moto_avg>=(MIN_THROTTLE+100)){
		if(laser_height<=5){
			is_take_off=0;
		}
		else{
			is_take_off=1;
		}
	}
}

void Baro_offset_Update(){
	static uint8_t cnt=0;
	if(LaserLidar_enabled&&cnt++>=10){
		if(laser_height>2){
			baro_offset=baro_height-laser_height;
		}
		cnt=0;
	}
}

void Altitude_Estimate(){
	Altitude_Update(); //更新高度传感器读数
	FlightStatus_Update(); //更新起飞状态
	
	if(LaserLidar_enabled&&Baro_enabled){
		if(fabs(euler_angles[0])>=15||fabs(euler_angles[1])>=15){
			//飞机倾斜时，激光置信度自动下降
			calc_height=0.75*laser_height+0.25*(baro_height-baro_offset);
		}
		else{
			calc_height=0.95*laser_height+0.05*(baro_height-baro_offset);
		}
	}
	else{
		if(LaserLidar_enabled){
			calc_height=laser_height;
		}
		if(Baro_enabled){
			calc_height=baro_height-baro_offset;
		}
	}
	//calc_height=LPButter_Alt(calc_height); //开启巴特沃斯滤波器
	calc_height=biquadFilterApply(&Alt_Filter,calc_height);
	// printf("%d %.2f %.2f %.2f\r\n",10*is_take_off,calc_height,laser_height,baro_height-baro_offset);
	Baro_offset_Update();
}

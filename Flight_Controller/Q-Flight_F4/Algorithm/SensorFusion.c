#include "SensorFusion.h"

float TIME_CONTANST_ZER=0.75f;//3.0
#define K_ACC_ZER 	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (5.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_POS_ZER               (5.0f / TIME_CONTANST_ZER)

float TIME_CONTANST_ZER_XY=0.4f;//3.0
#define K_ACC_ZER_XY 	        (3.0f / (TIME_CONTANST_ZER_XY * TIME_CONTANST_ZER_XY * TIME_CONTANST_ZER_XY))
#define K_VEL_ZER_XY	        (5.0f / (TIME_CONTANST_ZER_XY * TIME_CONTANST_ZER_XY))
#define K_POS_ZER_XY               (5.0f / TIME_CONTANST_ZER_XY)



extern quadstate_t current_state;
float baro_offset;
float calc_height,calc_velocity,calc_acc=0;
float altitude_estimate_period=0.01;
uint8_t is_take_off=0;
biquadFilter_t Alt_Filter,Vel_Filter[3],Acc_Filter;
float acc_correction_z,vel_correction_z,pos_correction_z=0;
float acc_correction_x,vel_correction_x,pos_correction_x=0;
float acc_correction_y,vel_correction_y,pos_correction_y=0;

void SensorInit(){
	 #if IS_HMC5883L_PRESENT
	 HMC5883L_Init();
	 #endif
//	 if(Control_Mode!=REPORT_ONLY_MODE&&Control_Mode!=MANUAL_MODE){
		#if IS_VL53LXX_PRESENT
		vl53l0x_Init();
		#endif
		#if IS_MS5611_PRESENT
		MS5611_Init();
		#endif
		#if IS_LC306_PRESENT
		LC306_Config_Init_Uart();
		#endif
		#if IS_US_PRESENT
		US_Init(); 
		#endif
		altitude_estimate_period=0.005f;
		biquadFilterInitLPF(&Alt_Filter,1200,altitude_estimate_period*10000);
		biquadFilterInitLPF(&Vel_Filter[2],300,altitude_estimate_period*10000);
		biquadFilterInitLPF(&Vel_Filter[0],10000,altitude_estimate_period*40000);
		biquadFilterInitLPF(&Vel_Filter[1],10000,altitude_estimate_period*40000);
		biquadFilterInitLPF(&Acc_Filter,200,altitude_estimate_period*10000);
		calc_velocity=0;
		calc_acc=0;
//	}
}

void Altitude_Update(){
	static uint8_t cnt=0;
	Enable_FreeRTOS=0;
	osThreadSuspendAll();
	#if IS_VL53LXX_PRESENT
	vl53l0x_StateMachine();
	#endif
	#if IS_MS5611_PRESENT
	if(++cnt>=2){
		MS5611_StateMachine();
		cnt=0;
	}
	#endif
	osThreadResumeAll();
	Enable_FreeRTOS=1;
	#if IS_US_PRESENT
	MT_US_get_distance();
	#endif
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
	//50ms 更新一次
	if(cnt++>=3){
		if(LaserLidar_enabled){
			if(laser_height>=4){
				baro_offset=baro_height-laser_height;
			}
		}
		else{
			if(baro_height<=1){
				baro_offset=baro_height-1;
			}
		}
		cnt=0;
	}
}

void Velocity_Estimate(){
	static double previous_height=0;
	calc_velocity+=(((calc_height-previous_height)/altitude_estimate_period)-calc_velocity)*0.3;
	calc_velocity=biquadFilterApply(&Vel_Filter[2],calc_velocity);
	previous_height=calc_height;
}

void Aceleration_Estimate(){
	static double previous_velocity=0;
	calc_acc+=(((calc_velocity-previous_velocity)/altitude_estimate_period)-calc_acc)*0.25;
	calc_acc=biquadFilterApply(&Acc_Filter,calc_acc);
	previous_velocity=calc_velocity;
}

void Position_Calc(float delta_time){
	#if USE_OPT_FLOW_ONLY
	current_state.velocity.x=opt_speed_data.x;
	current_state.velocity.y=opt_speed_data.y;
	current_state.position.x=opt_position_data.x;
	current_state.position.y=opt_position_data.y;
	#else
	delta_time=fabs(delta_time)/1000.0f;
	static float new_previous_acc_x,new_previous_velocity_x,new_previous_position_x=0;
	static float new_previous_acc_y,new_previous_velocity_y,new_previous_position_y=0;
	float Position_x_Dealt=opt_position_data.x-current_state.position.x;
  //三路积分反馈量修正惯导
  acc_correction_x +=Position_x_Dealt* K_ACC_ZER_XY*delta_time ;//加速度矫正量
  vel_correction_x +=Position_x_Dealt* K_VEL_ZER_XY*delta_time ;//速度矫正量
  pos_correction_x +=Position_x_Dealt* K_POS_ZER_XY*delta_time ;//位置矫正量
  //加速度计矫正后更新
  new_previous_acc_x=real_accData[0];//上一次加速度量
  float new_acc_x=real_accData[0]+acc_correction_x;
  float velocity_delta_x=(new_previous_acc_x+new_acc_x)*delta_time/2.0f;
  new_previous_position_x+=(current_state.velocity.x+0.5f*velocity_delta_x)*delta_time;	  //原始位置更新
  current_state.position.x=new_previous_position_x+pos_correction_x;	  //位置矫正后更新
  new_previous_velocity_x+=velocity_delta_x;	  //原始速度更新
  current_state.velocity.x=biquadFilterApply(&Vel_Filter[0],new_previous_velocity_x+vel_correction_x);	  //速度矫正后更新
	
	float Position_y_Dealt=opt_position_data.y-current_state.position.y;
	float Speed_y_Dealt=opt_speed_data.y-current_state.velocity.y;
  //三路积分反馈量修正惯导
  acc_correction_y +=Position_y_Dealt* K_ACC_ZER_XY*delta_time ;//加速度矫正量
  vel_correction_y +=Position_y_Dealt* K_VEL_ZER_XY*delta_time ;//速度矫正量
  pos_correction_y +=Position_y_Dealt* K_POS_ZER_XY*delta_time ;//位置矫正量
  //加速度计矫正后更新
  new_previous_acc_y=real_accData[1];//上一次加速度量
  float new_acc_y=real_accData[1]+acc_correction_y;
  float velocity_delta_y=(new_previous_acc_y+new_acc_y)*delta_time/2.0f;
  new_previous_position_y+=(current_state.velocity.y+0.5f*velocity_delta_y)*delta_time;	  //原始位置更新
  current_state.position.y=new_previous_position_y+pos_correction_y;	  //位置矫正后更新
  new_previous_velocity_y+=velocity_delta_y;	  //原始速度更新
  current_state.velocity.y=biquadFilterApply(&Vel_Filter[1],new_previous_velocity_y+vel_correction_y);	  //速度矫正后更新
	#endif
}

void Altitude_Calc(){
	Altitude_Estimate();
	Velocity_Estimate();
	static float new_previous_acc,new_previous_velocity,new_previous_height=0;
	#if 0
	Velocity_Estimate();
	Aceleration_Estimate();
	float new_acc=(1-K_ACC_CALC)*real_accData[2]+K_ACC_CALC*calc_acc;
	float new_delta_velocity=K_ACC_CALC2*(0.5f*(new_previous_acc+new_acc)*altitude_estimate_period)+(1-K_ACC_CALC2)*acc_velocity[2];
	float new_velocity=(1-K_VEL_CALC)*(new_previous_velocity+
		new_delta_velocity)+K_VEL_CALC*calc_velocity;
	new_previous_velocity=new_velocity;
	float new_delta_height=K_VEL_CALC2*(0.5f*(new_previous_velocity+new_velocity)*altitude_estimate_period)+(1-K_VEL_CALC2)*acc_position[2];
	float new_height=(1-K_POS_CALC)*(new_previous_height+
		 new_delta_height)+K_POS_CALC*calc_height;
	new_previous_height=new_height;
	sins_height=new_height;
	sins_velocity=new_velocity;
	acc_velocity[2]=0;
	acc_position[2]=0;
	#else
	float Altitude_Dealt=calc_height-current_state.position.z;//气压计(超声波)与SINS估计量的差，单位cm
  //三路积分反馈量修正惯导
  acc_correction_z +=Altitude_Dealt* K_ACC_ZER*altitude_estimate_period ;//加速度矫正量
  vel_correction_z +=Altitude_Dealt* K_VEL_ZER*altitude_estimate_period ;//速度矫正量
  pos_correction_z +=Altitude_Dealt* K_POS_ZER*altitude_estimate_period ;//位置矫正量
  //加速度计矫正后更新
  new_previous_acc=real_accData[2];//上一次加速度量
  float new_acc=real_accData[2]+acc_correction_z;
  //速度增量矫正后更新，用于更新位置,由于步长h=0.005,相对较长，
  //这里采用二阶龙格库塔法更新微分方程，不建议用更高阶段，因为加速度信号非平滑
  float velocity_delta=(new_previous_acc+new_acc)*altitude_estimate_period/2.0f;
  //原始位置更新
  new_previous_height+=(current_state.velocity.z+0.5f*velocity_delta)*altitude_estimate_period;
  //位置矫正后更新
  current_state.position.z=new_previous_height+pos_correction_z;
  //原始速度更新
  new_previous_velocity+=velocity_delta;
  //速度矫正后更新
  current_state.velocity.z=new_previous_velocity+vel_correction_z;
	#endif
}

void Altitude_Estimate(){
	Altitude_Update(); //更新高度传感器读数
	FlightStatus_Update(); //更新起飞状态
	
	if(LaserLidar_enabled&&Baro_enabled){
		if(fabs(euler_angles[0])>=15||fabs(euler_angles[1])>=15){
			//飞机倾斜时，激光置信度自动下降
			calc_height=0.75f*laser_height+0.25f*(baro_height-baro_offset);
		}
		else{
			calc_height=0.95f*laser_height+0.05f*(baro_height-baro_offset);
		}
	}
	else if(LaserLidar_enabled||Baro_enabled){
		if(LaserLidar_enabled){
			calc_height=laser_height;
		}
		if(Baro_enabled){
			calc_height=baro_height-baro_offset;
		}
	}
	if(US_Enabled){
		if(fabs(euler_angles[0])<=12.5||fabs(euler_angles[1])<=12.5){
			calc_height=0.6f*calc_height+0.4f*US_distance;
		}
		else{
			calc_height=0.95f*calc_height+0.05f*US_distance;
		}
	}
	if((!US_Enabled)&&(!Baro_enabled)&&(!LaserLidar_enabled)){
		calc_height+=calc_velocity*altitude_estimate_period;
	}

	//calc_height=LPButter_Alt(calc_height); //开启巴特沃斯滤波器
	if(calc_height<=1.0f){
		calc_height=1.0f;  //防止光流无合法高度信息
	}
	calc_height=biquadFilterApply(&Alt_Filter,calc_height);
	// printf("%d %.2f %.2f %.2f\r\n",10*is_take_off,calc_height,laser_height,baro_height-baro_offset);
	Baro_offset_Update();
}

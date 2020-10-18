#include "Control.h"

pidsuite pid_velocity_x,pid_velocity_y,pid_velocity_z; //速度环pid
pidsuite pid_position_x,pid_position_y,pid_position_z; //位置环pid
pidsuite pid_in_Pitch,pid_in_Roll,pid_in_Yaw;
pidsuite pid_out_Pitch,pid_out_Roll,pid_out_Yaw;
Vector3 previous_position;
Vector2f error_position;
quadstate_t target_state,current_state;
float dt;
float throttleLpf,throttleBase,throttle;
float pid_out_pitch=0;
float pid_out_roll=0;
float pid_out_yaw=0;
float pid_in_pitch=0;
float pid_in_roll=0;
float pid_in_yaw=0;
uint16_t pwm_moto1,pwm_moto2,pwm_moto3,pwm_moto4;   

void PID_ControllerInit(){
	//姿态控制器pid初始化
	pidInit(&pid_out_Pitch, 0, Flash2Float(Quad_Parameter[1]), Flash2Float(Quad_Parameter[2]), Flash2Float(Quad_Parameter[3]));
	pidInit(&pid_out_Roll, 0, Flash2Float(Quad_Parameter[4]), Flash2Float(Quad_Parameter[5]), Flash2Float(Quad_Parameter[6]));
	pidInit(&pid_out_Yaw, 0, Flash2Float(Quad_Parameter[7]), Flash2Float(Quad_Parameter[8]), Flash2Float(Quad_Parameter[9]));
	
	pidInit(&pid_in_Pitch, 0, Flash2Float(Quad_Parameter[10]), Flash2Float(Quad_Parameter[11]), Flash2Float(Quad_Parameter[12]));
  pidInit(&pid_in_Roll, 0, Flash2Float(Quad_Parameter[13]), Flash2Float(Quad_Parameter[14]), Flash2Float(Quad_Parameter[15]));
	pidInit(&pid_in_Yaw,0, Flash2Float(Quad_Parameter[16]),Flash2Float(Quad_Parameter[17]),Flash2Float(Quad_Parameter[18]));

	pidSetIntegralLimit(&pid_out_Pitch, Quad_Parameter[19]);
	pidSetIntegralLimit(&pid_out_Roll, Quad_Parameter[20]);
	pidSetIntegralLimit(&pid_out_Yaw, Quad_Parameter[21]);
  pidSetIntegralLimit(&pid_in_Pitch, Quad_Parameter[22]);
	pidSetIntegralLimit(&pid_in_Roll, Quad_Parameter[23]);
	pidSetIntegralLimit(&pid_in_Yaw, Quad_Parameter[24]);
	
	
//	pidInit(&pid_out_Pitch, 0, PID_OUT_PITCH_KP, PID_OUT_PITCH_KI, PID_OUT_PITCH_KD);
//	pidInit(&pid_out_Roll, 0, PID_OUT_ROLL_KP, PID_OUT_ROLL_KI, PID_OUT_ROLL_KD);
//	
//	pidInit(&pid_in_Pitch, 0, PID_IN_PITCH_KP, PID_IN_PITCH_KI, PID_IN_PITCH_KD);
//  pidInit(&pid_in_Roll, 0, PID_IN_ROLL_KP, PID_IN_ROLL_KI, PID_IN_ROLL_KD);
//	pidInit(&pid_in_Yaw,0,PID_IN_YAW_KP,PID_IN_YAW_KI,PID_IN_YAW_KD);

//	pidSetIntegralLimit(&pid_out_Pitch, PID_OUT_PITCH_INTEGRATION_LIMIT);
//	pidSetIntegralLimit(&pid_out_Roll, PID_OUT_ROLL_INTEGRATION_LIMIT);
//  pidSetIntegralLimit(&pid_in_Pitch, PID_IN_PITCH_INTEGRATION_LIMIT);
//	pidSetIntegralLimit(&pid_in_Roll, PID_IN_ROLL_INTEGRATION_LIMIT);
//	pidSetIntegralLimit(&pid_in_Yaw, PID_IN_YAW_INTEGRATION_LIMIT);
//	uint8_t index_offset=24;
//	pidInit(&pid_velocity_x, 0, Flash2Float(Quad_Parameter[1+index_offset+index_offset]), Flash2Float(Quad_Parameter[2+index_offset]), Flash2Float(Quad_Parameter[3+index_offset]));
//	pidInit(&pid_velocity_y, 0, Flash2Float(Quad_Parameter[4+index_offset+index_offset]), Flash2Float(Quad_Parameter[5+index_offset]), Flash2Float(Quad_Parameter[6+index_offset]));
//	pidInit(&pid_velocity_z, 0, Flash2Float(Quad_Parameter[7+index_offset+index_offset]), Flash2Float(Quad_Parameter[8+index_offset]), Flash2Float(Quad_Parameter[9+index_offset]));
//	
//	pidInit(&pid_position_x, 0, Flash2Float(Quad_Parameter[10+index_offset]), Flash2Float(Quad_Parameter[11+index_offset]), Flash2Float(Quad_Parameter[12+index_offset]));
//  pidInit(&pid_position_y, 0, Flash2Float(Quad_Parameter[13+index_offset]), Flash2Float(Quad_Parameter[14+index_offset]), Flash2Float(Quad_Parameter[15+index_offset]));
//	pidInit(&pid_position_z,0, Flash2Float(Quad_Parameter[16+index_offset]),Flash2Float(Quad_Parameter[17+index_offset]),Flash2Float(Quad_Parameter[18+index_offset]));

//	pidSetIntegralLimit(&pid_velocity_x, Quad_Parameter[19+index_offset]);
//	pidSetIntegralLimit(&pid_velocity_y, Quad_Parameter[20+index_offset]);
//	pidSetIntegralLimit(&pid_velocity_z, Quad_Parameter[21+index_offset]);
//  pidSetIntegralLimit(&pid_position_x, Quad_Parameter[22+index_offset]);
//	pidSetIntegralLimit(&pid_position_y, Quad_Parameter[23+index_offset]);
//	pidSetIntegralLimit(&pid_position_z, Quad_Parameter[24+index_offset]);
	
	//位置&速度控制器pid初始化
	pidInit(&pid_velocity_x, 0, PID_VELOCITY_X_KP, PID_VELOCITY_X_KI, PID_VELOCITY_X_KD);
	pidInit(&pid_velocity_y, 0, PID_VELOCITY_Y_KP, PID_VELOCITY_Y_KI, PID_VELOCITY_Y_KD);
	pidInit(&pid_velocity_z, 0, PID_VELOCITY_Z_KP, PID_VELOCITY_Z_KI, PID_VELOCITY_Z_KD);
	
	pidInit(&pid_position_x, 0, PID_POSITION_X_KP, PID_POSITION_X_KI, PID_POSITION_X_KD);
  pidInit(&pid_position_y, 0, PID_POSITION_Y_KP, PID_POSITION_Y_KI, PID_POSITION_Y_KD);
	pidInit(&pid_position_z, 0, PID_POSITION_Z_KP, PID_POSITION_Z_KI, PID_POSITION_Z_KD);

	pidSetIntegralLimit(&pid_velocity_x, PID_VELOCITY_X_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_velocity_y, PID_VELOCITY_Y_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_velocity_z, PID_VELOCITY_Z_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pid_position_x, PID_POSITION_X_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_position_y, PID_POSITION_Y_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&pid_position_z, PID_POSITION_Z_INTEGRATION_LIMIT);
	
	Quadstate_Init(&target_state);
	Quadstate_Init(&current_state);
	Vector3_Init(&previous_position);
	Vector2f_Init(&error_position);
	dt=quad_control_period/1000;
}

void Get_current_state(){
	previous_position=current_state.position;
	
	current_state.attitude.roll=euler_angles[0];
	current_state.attitude.pitch=euler_angles[1];
	current_state.attitude.yaw=euler_angles[2];
	if(Control_Mode!=REPORT_ONLY_MODE&&Control_Mode!=MANUAL_MODE){
		// 注意光流位置信息的对应关系！
		current_state.position.y=opt_position_data.x;
		current_state.position.x=opt_position_data.y;
		current_state.position.z=calc_height;
		
		// 计算运动速度
		current_state.velocity.x=(current_state.position.x-previous_position.x)/dt;
		current_state.velocity.y=(current_state.position.y-previous_position.y)/dt;
		current_state.velocity.z=(current_state.position.z-previous_position.z)/dt;
	}
//	current_state.acc.x=accData[0];
//	current_state.acc.y=accData[1];
//	current_state.acc.z=accData[2];
	current_state.gyro.x=gyroData[0];
	current_state.gyro.y=gyroData[1];
	current_state.gyro.z=gyroData[2];
//	current_state.mag.x=gyroData[0];
//	current_state.mag.y=gyroData[1];
//	current_state.mag.z=gyroData[2];
}

void RCposition_Update(){
	static uint8_t isChangingPositionXY,position_changing_cnt=0;
	if(target_state.position.z==0&&!is_take_off){
		is_take_off=1;
	}
	else{
		if(fabs(RCvelocity.x)>=2||fabs(RCvelocity.y)>=2){
			isChangingPositionXY = 1;
			position_changing_cnt = 0;
		}
		if(isChangingPositionXY){
			if(position_changing_cnt++ > 100)
			{
				position_changing_cnt = 0;
				isChangingPositionXY = 0;
			}		
			target_state.position.x+=RCvelocity.x*dt+error_position.x;
			target_state.position.y+=RCvelocity.y*dt+error_position.y;
		}
		else if(!isChangingPositionXY){
			error_position.x = target_state.position.x - current_state.position.x;
			error_position.y = target_state.position.y - current_state.position.y;
			error_position.x = constrainf(error_position.x, -30.0f, 30.0f);	/*误差限幅 单位cm*/
			error_position.y = constrainf(error_position.y, -30.0f, 30.0f);	/*误差限幅 单位cm*/
		}
	}
	target_state.position.z+=RCvelocity.z*dt;
}

void ControlMode_AutoShift(){
	if(Control_Mode==FIXED_ALTITUDE_MODE||Control_Mode==FIXED_POINT_MODE){
		//光流数据可用则自动开启定点模式
		if(LC306_Uart_Flag==1&&Control_Mode==FIXED_ALTITUDE_MODE){
			Control_Mode=FIXED_POINT_MODE;
		}
		//光流数据不可用则自动开启定高模式
		else if(LC306_Uart_Flag==0&&Control_Mode==FIXED_POINT_MODE){
			Control_Mode=FIXED_ALTITUDE_MODE;
		}
	}
}

void Quad_Control(){
	static uint8_t position_cnt,velocity_cnt,report_cnt=0;
	Get_current_state(); //获取当前状态

	if(Control_Mode!=REPORT_ONLY_MODE){ //非报告模式则开启控制
		if(Control_Mode!=MANUAL_MODE){ //非手动模式则开启位置/速度控制器
			RCposition_Update(); //10ms遥控位置更新
			ControlMode_AutoShift(); //自动切换控制模式
			if(velocity_cnt++>=1){ //20ms
				velocity_cnt=0;
				Velocity_Control();
			}
			if(position_cnt++>=3){ //40ms
				position_cnt=0;
				Position_Control();
			}
		}
		Attitude_Control(); //10ms
		Motor_Control(); //10ms
	}
	if(report_cnt++>2){
		Report_Control();
	}
}

void Report_Control(){
	switch(verbose){
		case 0:
			break;
		case 1:
			ANO_DT_Send_Status(euler_angles[0],euler_angles[1],euler_angles[2],10,0,1);
			ANO_DT_Send_Senser(accData[0],accData[1],accData[2],gyroData[0],gyroData[1],gyroData[2],0,0,0,0);
			ANO_DT_Send_RCData((throttle-1000)/10,target_state.attitude.yaw,target_state.attitude.roll,target_state.attitude.pitch,0,0,0,0,0,0);
			ANO_DT_Send_MotoPWM(pwm_moto1,pwm_moto2,pwm_moto3,pwm_moto4,0,0,0,0);
			break;
		case 2:
			QF_Send_Status(euler_angles[0],euler_angles[1],euler_angles[2],10,pwm_moto1,pwm_moto2,pwm_moto3,pwm_moto4,
		accData[0],accData[1],accData[2],gyroData[0],gyroData[1],gyroData[2],0,0,0,
		(int)((throttle-1000)/10),(int)(target_state.attitude.yaw*100),(int)(target_state.attitude.roll*100),(int)(target_state.attitude.pitch*100));
		case 3:
			QF_Send_PID_Output();
		default:
			break;
	}
}

void Motor_Control(){
	PWN_Control_Motors(htim1, pwm_moto1, pwm_moto2, pwm_moto3,pwm_moto4);
}

void Attitude_Control(){
	if(throttle>MIN_THROTTLE)        //油门大于死区才进行控制
	{
		pid_out_pitch = pidUpdate(&pid_out_Pitch,current_state.attitude.pitch,target_state.attitude.pitch);
		pid_in_pitch=pidUpdate(&pid_in_Pitch,gyroData[1],pid_out_pitch);
		
		pid_out_roll = pidUpdate(&pid_out_Roll,current_state.attitude.roll,target_state.attitude.roll);
		pid_in_roll=pidUpdate(&pid_in_Roll,gyroData[0],pid_out_roll);
		
		// 使用单角速度环控制yaw
		// pid_out_yaw=pidUpdate(&pid_out_Yaw,euler_angles[2],target_state.attitude.yaw);
		// pid_in_yaw=pidUpdate(&pid_in_Yaw,pid_out_yaw,yaw_expect);
		pid_in_yaw=pidUpdate(&pid_in_Yaw,gyroData[2]/16.4,target_state.attitude.yaw);
	}

	else if(throttle<=MIN_THROTTLE)   //否则积分清零
	{
		pid_out_Pitch.integ=0;
		pid_out_Roll.integ=0;
		pid_out_Yaw.integ=0;
		pid_in_Pitch.integ=0;
		pid_in_Roll.integ=0;
		pid_in_Yaw.integ=0;
	}

	//内环pid输出限幅
	pid_in_pitch=constrainf(pid_in_pitch,-MAX_GYRORATE_OUTPUT,MAX_GYRORATE_OUTPUT);
	pid_in_roll=constrainf(pid_in_roll,-MAX_GYRORATE_OUTPUT,MAX_GYRORATE_OUTPUT);
	pid_in_yaw=constrainf(pid_in_yaw,-MAX_GYRORATE_OUTPUT,MAX_GYRORATE_OUTPUT);
	
	//动力分配
	pwm_moto1=throttle-pid_in_pitch-pid_in_roll+pid_in_yaw;
	pwm_moto2=throttle-pid_in_pitch+pid_in_roll-pid_in_yaw;
	pwm_moto3=throttle+pid_in_pitch+pid_in_roll+pid_in_yaw;
	pwm_moto4=throttle+pid_in_pitch-pid_in_roll-pid_in_yaw;
	
	if(throttle>MIN_THROTTLE)
	{
		//限幅
		if(pwm_moto1<MIN_THROTTLE)pwm_moto1=YM_Dead;else if(pwm_moto1>2000)pwm_moto1=2000;
		if(pwm_moto2<MIN_THROTTLE)pwm_moto2=YM_Dead;else if(pwm_moto2>2000)pwm_moto2=2000;
		if(pwm_moto3<MIN_THROTTLE)pwm_moto3=YM_Dead;else if(pwm_moto3>2000)pwm_moto3=2000;
		if(pwm_moto4<MIN_THROTTLE)pwm_moto4=YM_Dead;else if(pwm_moto4>2000)pwm_moto4=2000;
	}
	else 
	{
	  pwm_moto1=1000,pwm_moto2=1000,pwm_moto3=1000,pwm_moto4=1000;
	}
}

void Velocity_Control(){
	static uint16_t altholdCount = 0;
	
	// Roll & Pitch ,定高模式则直接以遥控器为准
	if(Control_Mode!=FIXED_ALTITUDE_MODE){
		target_state.attitude.pitch = 0.15f * pidUpdate(&pid_velocity_x, current_state.velocity.x, target_state.velocity.x);
		target_state.attitude.roll = 0.15f * pidUpdate(&pid_velocity_y, current_state.velocity.y, target_state.velocity.y);
	}
	target_state.attitude.pitch = constrainf(target_state.attitude.pitch,-MAX_ANGLE_XY,MAX_ANGLE_XY);
	target_state.attitude.roll = constrainf(target_state.attitude.roll,-MAX_ANGLE_XY,MAX_ANGLE_XY);
	// throttle
	float throttleRaw = pidUpdate(&pid_velocity_z, current_state.velocity.z, target_state.velocity.z);
	throttle = constrainf(throttleRaw + THROTTLE_BASE, 1000, 1700);	/*油门限幅*/
	throttleLpf += (throttle - throttleLpf) * 0.003f;
	
	if(fabs(current_state.acc.z) < 35.f)
	{
		altholdCount++;
		if(altholdCount > 500)
		{
			altholdCount = 0;
			if(fabs(throttleBase - throttleLpf) > 1000.f)	/*更新基础油门值*/
				throttleBase = throttleLpf;
		}
	}
	else
	{
		altholdCount = 0;
	}
}

void Position_Control(){
		if(Control_Mode!=FIXED_ALTITUDE_MODE){
			target_state.velocity.x = 0.1f * pidUpdate(&pid_position_x, current_state.position.x,target_state.position.x);
			target_state.velocity.x = constrainf(target_state.velocity.x,-MAX_VELOCITY_XY,MAX_VELOCITY_XY);
		}
		target_state.velocity.y = 0.1f * pidUpdate(&pid_position_y, current_state.position.y,target_state.position.y);
		target_state.velocity.y = constrainf(target_state.velocity.y,-MAX_VELOCITY_XY,MAX_VELOCITY_XY);
		target_state.velocity.z = pidUpdate(&pid_position_z, current_state.position.z,target_state.position.z);
		target_state.velocity.z = constrainf(target_state.velocity.y,-MAX_VELOCITY_Z,MAX_VELOCITY_Z);
}

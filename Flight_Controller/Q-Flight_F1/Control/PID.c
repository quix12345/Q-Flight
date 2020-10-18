#include "PID.h"
#include "tim.h"
#include "PWN.h"
#include "main.h"
#include "stdio.h"
#include "ahrs.h"
#include "Flash.h"
#include "Verbose.h"
#include "SysSetting.h"

// x:pitch    y:roll    z:yaw

//              y
//              |
//           1  |  2
//            \ | /
//             \ /
//              * ------x
//             / \
//            /   \
//           4     3

//uint8_t armed=1;
//uint16_t MOTO1_PWM,MOTO2_PWM,MOTO3_PWM,MOTO4_PWM;    //四电机pwm

//pidsuite pid_out_Pitch;			//俯仰角外环pid
//pidsuite pid_out_Roll;			//横滚角外环pid
//pidsuite pid_out_Yaw;				//偏航角外环pid
//pidsuite pid_in_Pitch;			//俯仰角内环pid
//pidsuite pid_in_Roll;				//横滚角内环pid
//pidsuite pid_in_Yaw;				//偏航角内环pid


//float pid_out_pitch=0;
//float pid_out_roll=0;
//float pid_out_yaw=0;
//float pid_in_pitch=0;
//float pid_in_roll=0;
//float pid_in_yaw=0;

/*------------------------------------------pid结构初始化-----------------------------------------*/
//输入参数：结构体指针，期望值，kp,ki,kd
void pidInit(pidsuite* pid, const float desired, const float kp,
             const float ki, const float kd)
{

  pid->error = 0;
  pid->prevError = 0;
  pid->integ = 0;
  pid->deriv = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
}

/*----------------------------------------------pid积分部分限制值-------------------------------------------*/
void pidSetIntegralLimit(pidsuite* pid, const float limit)
{
  pid->iLimit = limit;
}

/*--------------------------------------pid输出更新------------------------------------------*/
//输入参数：pid结构体指针，测量值 ,期望值
//输出：pid输出
float pidUpdate(pidsuite* pid, const float measured,float expect)
{
  float output;
	static int8_t integ_sign=0;
	uint8_t is_shrink=1;

  pid->desired=expect;			 				//获取期望角度

  pid->error = pid->desired - measured;	 	  //偏差：期望-测量值
	if(is_shrink){
		if(integ_sign==0){
			pid->integ += pid->error;
			integ_sign=1;
		}
		else{
			if(math_sign(pid->error)!=math_sign(pid->integ)){
				pid->integ += pid->error;	  //偏差积分
			}
		}
	}
	else{
		pid->integ += pid->error;	
	}
  if (pid->integ > pid->iLimit)				  //积分限幅
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)		//积分限幅
  {
    pid->integ = -pid->iLimit;
  }
  pid->deriv = pid->error - pid->prevError;		//微分

	pid->outP = pid->kp * pid->error;					  //方便独立观察
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;
	
	output = pid->outP +
					 pid->outI +
					 pid->outD;
  
  pid->prevError = pid->error;							 		//更新前一次偏差

  return output;
}

//void Control(float Throttle,float pitch_expect,float roll_expect,float yaw_expect)
//{
//	if(Throttle<0){
//		Throttle=0;
//	}
//	else if(Throttle>1){
//			Throttle=1;
//	}
//	Throttle=0.65*Throttle+0.1;
//	uint16_t RC_THROTTLE=1000+Throttle*1000;
//	//RC_THROTTLE*=0.8;
//	if(RC_THROTTLE>YM_Dead)        //油门大于死区才进行控制
//	{
//		pid_out_pitch = pidUpdate(&pid_out_Pitch,euler_angles[1],pitch_expect);
//		//pid_out_pitch=pitch_expect;
//		pid_in_pitch=pidUpdate(&pid_in_Pitch,gyroData[1],pid_out_pitch);
//		pid_out_roll = pidUpdate(&pid_out_Roll,euler_angles[0],roll_expect);
//		//pid_out_roll=roll_expect;
//		pid_in_roll=pidUpdate(&pid_in_Roll,gyroData[0],pid_out_roll);
//		pid_out_yaw=pidUpdate(&pid_out_Yaw,euler_angles[2],yaw_expect);
//		// pid_in_yaw=pidUpdate(&pid_in_Yaw,pid_out_yaw,yaw_expect);
//		pid_in_yaw=pidUpdate(&pid_in_Yaw,gyroData[2]/16.4,yaw_expect);
//	}

//	else if(RC_THROTTLE<YM_Dead)   //否则积分清零
//	{
//		pid_in_Pitch.integ=0;
//		pid_in_Roll.integ=0;
//		pid_in_Yaw.integ=0;
//	}

////	pid_in_yaw=0;
////	pid_in_pitch=0;
////	int roll_in_limit=200;
////	if(pid_in_roll>=roll_in_limit){
////		pid_in_roll=roll_in_limit;
////	}
////	else if(pid_in_roll<=-roll_in_limit){
////		pid_in_roll=-roll_in_limit;
////	}
//	
//	//动力分配
//	MOTO1_PWM=RC_THROTTLE+pid_in_pitch-pid_in_roll+pid_in_yaw;
//	MOTO2_PWM=RC_THROTTLE+pid_in_pitch+pid_in_roll-pid_in_yaw;
//	MOTO3_PWM=RC_THROTTLE-pid_in_pitch+pid_in_roll+pid_in_yaw;
//	MOTO4_PWM=RC_THROTTLE-pid_in_pitch-pid_in_roll-pid_in_yaw;
//	
//	// printf("pitch:%d roll:%d yaw:%d 1:%d 2:%d 3:%d\r\n",gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]);
//	if(RC_THROTTLE>YM_Dead)
//	{
//		//限幅
//		if(MOTO1_PWM<YM_Dead)MOTO1_PWM=YM_Dead;else if(MOTO1_PWM>2000)MOTO1_PWM=2000;
//		if(MOTO2_PWM<YM_Dead)MOTO2_PWM=YM_Dead;else if(MOTO2_PWM>2000)MOTO2_PWM=2000;
//		if(MOTO3_PWM<YM_Dead)MOTO3_PWM=YM_Dead;else if(MOTO3_PWM>2000)MOTO3_PWM=2000;
//		if(MOTO4_PWM<YM_Dead)MOTO4_PWM=YM_Dead;else if(MOTO4_PWM>2000)MOTO4_PWM=2000;
//	}
//	else 
//	{
//	  MOTO1_PWM=1000,MOTO2_PWM=1000,MOTO3_PWM=1000,MOTO4_PWM=1000;
//	}
//	static int counter=0;
//	if(counter>=2){
//	switch(verbose){
//		case 0:
//			break;
//		case 1:
//			ANO_DT_Send_Status(euler_angles[0],euler_angles[1],euler_angles[2],10,0,1);
//			ANO_DT_Send_Senser(accData[0],accData[1],accData[2],gyroData[0],gyroData[1],gyroData[2],0,0,0,0);
//			ANO_DT_Send_RCData(throttle,yaw_expect,roll_expect,pitch_expect,0,0,0,0,0,0);
//			ANO_DT_Send_MotoPWM(MOTO1_PWM,MOTO2_PWM,MOTO3_PWM,MOTO4_PWM,0,0,0,0);
//			break;
//		case 2:
//			QF_Send_Status(euler_angles[0],euler_angles[1],euler_angles[2],10,MOTO1_PWM,MOTO2_PWM,MOTO3_PWM,MOTO4_PWM,
//		accData[0],accData[1],accData[2],gyroData[0],gyroData[1],gyroData[2],0,0,0,
//		(int)(throttle*100),(int)(yaw_expect*100),(int)(roll_expect*100),(int)(pitch_expect*100));
//		case 3:
//			QF_Send_PID_Output();
//		default:
//			break;
//	}
////	if(verbose!=0&&verbose<9){
////		printf("\r\n");
////	}
//	counter=0;
//}
//	else{
//		counter++;
//	}


//	PWN_Control_Motors(htim1, Convert2PercentagePWN(MOTO1_PWM), 
//	Convert2PercentagePWN(MOTO2_PWM), 
//	Convert2PercentagePWN(MOTO3_PWM), 
//	Convert2PercentagePWN(MOTO4_PWM));
//	
//	
//}

//void PID_controllerInit(void)     //pid参数初始化
//{
//	
//	pidInit(&pid_out_Pitch, 0, Flash2Float(Quad_Parameter[1]), Flash2Float(Quad_Parameter[2]), Flash2Float(Quad_Parameter[3]));
//	pidInit(&pid_out_Roll, 0, Flash2Float(Quad_Parameter[4]), Flash2Float(Quad_Parameter[5]), Flash2Float(Quad_Parameter[6]));
//	pidInit(&pid_out_Yaw, 0, Flash2Float(Quad_Parameter[7]), Flash2Float(Quad_Parameter[8]), Flash2Float(Quad_Parameter[9]));
//	
//	pidInit(&pid_in_Pitch, 0, Flash2Float(Quad_Parameter[10]), Flash2Float(Quad_Parameter[11]), Flash2Float(Quad_Parameter[12]));
//  pidInit(&pid_in_Roll, 0, Flash2Float(Quad_Parameter[13]), Flash2Float(Quad_Parameter[14]), Flash2Float(Quad_Parameter[15]));
//	pidInit(&pid_in_Yaw,0, Flash2Float(Quad_Parameter[16]),Flash2Float(Quad_Parameter[17]),Flash2Float(Quad_Parameter[18]));

//	pidSetIntegralLimit(&pid_out_Pitch, Quad_Parameter[19]);
//	pidSetIntegralLimit(&pid_out_Roll, Quad_Parameter[20]);
//	pidSetIntegralLimit(&pid_out_Yaw, Quad_Parameter[21]);
//  pidSetIntegralLimit(&pid_in_Pitch, Quad_Parameter[22]);
//	pidSetIntegralLimit(&pid_in_Roll, Quad_Parameter[23]);
//	pidSetIntegralLimit(&pid_in_Yaw, Quad_Parameter[24]);
//	
//	
////	pidInit(&pid_out_Pitch, 0, PID_OUT_PITCH_KP, PID_OUT_PITCH_KI, PID_OUT_PITCH_KD);
////	pidInit(&pid_out_Roll, 0, PID_OUT_ROLL_KP, PID_OUT_ROLL_KI, PID_OUT_ROLL_KD);
////	
////	pidInit(&pid_in_Pitch, 0, PID_IN_PITCH_KP, PID_IN_PITCH_KI, PID_IN_PITCH_KD);
////  pidInit(&pid_in_Roll, 0, PID_IN_ROLL_KP, PID_IN_ROLL_KI, PID_IN_ROLL_KD);
////	pidInit(&pid_in_Yaw,0,PID_IN_YAW_KP,PID_IN_YAW_KI,PID_IN_YAW_KD);

////	pidSetIntegralLimit(&pid_out_Pitch, PID_OUT_PITCH_INTEGRATION_LIMIT);
////	pidSetIntegralLimit(&pid_out_Roll, PID_OUT_ROLL_INTEGRATION_LIMIT);
////  pidSetIntegralLimit(&pid_in_Pitch, PID_IN_PITCH_INTEGRATION_LIMIT);
////	pidSetIntegralLimit(&pid_in_Roll, PID_IN_ROLL_INTEGRATION_LIMIT);
////	pidSetIntegralLimit(&pid_in_Yaw, PID_IN_YAW_INTEGRATION_LIMIT);
//}



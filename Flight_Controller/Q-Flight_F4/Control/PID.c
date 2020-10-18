#include "PID.h"


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

float pidErrorUpdate(pidsuite* pid, const float expect,float error) // target - current
{
  float output;
	static uint8_t integ_sign=0;
	uint8_t is_shrink=1;

  pid->desired=expect;			 				//获取期望角度

  pid->error = error;	 	  //偏差：期望-测量值
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

/*--------------------------------------pid输出更新------------------------------------------*/
//输入参数：pid结构体指针，测量值 ,期望值
//输出：pid输出
float pidUpdate(pidsuite* pid, const float measured,float expect)
{
  float output;
	static uint8_t integ_sign=0;
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


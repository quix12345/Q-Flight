#include "PID.h"


/*------------------------------------------pid�ṹ��ʼ��-----------------------------------------*/
//����������ṹ��ָ�룬����ֵ��kp,ki,kd
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

/*----------------------------------------------pid���ֲ�������ֵ-------------------------------------------*/
void pidSetIntegralLimit(pidsuite* pid, const float limit)
{
  pid->iLimit = limit;
}

float pidErrorUpdate(pidsuite* pid, const float expect,float error) // target - current
{
  float output;
	static uint8_t integ_sign=0;
	uint8_t is_shrink=1;

  pid->desired=expect;			 				//��ȡ�����Ƕ�

  pid->error = error;	 	  //ƫ�����-����ֵ
	if(is_shrink){
		if(integ_sign==0){
			pid->integ += pid->error;
			integ_sign=1;
		}
		else{
			if(math_sign(pid->error)!=math_sign(pid->integ)){
				pid->integ += pid->error;	  //ƫ�����
			}
		}
	}
	else{
		pid->integ += pid->error;	
	}
  if (pid->integ > pid->iLimit)				  //�����޷�
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)		//�����޷�
  {
    pid->integ = -pid->iLimit;
  }
  pid->deriv = pid->error - pid->prevError;		//΢��

	pid->outP = pid->kp * pid->error;					  //��������۲�
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;
	
	output = pid->outP +
					 pid->outI +
					 pid->outD;
  
  pid->prevError = pid->error;							 		//����ǰһ��ƫ��

  return output;
}

/*--------------------------------------pid�������------------------------------------------*/
//���������pid�ṹ��ָ�룬����ֵ ,����ֵ
//�����pid���
float pidUpdate(pidsuite* pid, const float measured,float expect)
{
  float output;
	static uint8_t integ_sign=0;
	uint8_t is_shrink=1;

  pid->desired=expect;			 				//��ȡ�����Ƕ�

  pid->error = pid->desired - measured;	 	  //ƫ�����-����ֵ
	if(is_shrink){
		if(integ_sign==0){
			pid->integ += pid->error;
			integ_sign=1;
		}
		else{
			if(math_sign(pid->error)!=math_sign(pid->integ)){
				pid->integ += pid->error;	  //ƫ�����
			}
		}
	}
	else{
		pid->integ += pid->error;	
	}
  if (pid->integ > pid->iLimit)				  //�����޷�
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)		//�����޷�
  {
    pid->integ = -pid->iLimit;
  }
  pid->deriv = pid->error - pid->prevError;		//΢��

	pid->outP = pid->kp * pid->error;					  //��������۲�
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;
	
	output = pid->outP +
					 pid->outI +
					 pid->outD;
  
  pid->prevError = pid->error;							 		//����ǰһ��ƫ��

  return output;
}


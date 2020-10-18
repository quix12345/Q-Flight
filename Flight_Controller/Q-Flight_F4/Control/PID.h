#include "main.h"

#ifndef _PID
#define _PID

#define DEFAULT_PID_INTEGRATION_LIMIT  100.0
#define YM_Dead 1150

#define PID_FLASH_VERIFY 25


typedef struct
{
  float desired;      //< ����������ֵ
  float error;        //< ����ֵ-ʵ��ֵ
  float prevError;    //< ǰһ��ƫ��
  float integ;        //< ���ֲ���
  float deriv;        //< ΢�ֲ���
  float kp;           //< ��������
  float ki;           //< ���ֲ���
  float kd;           //< ΢�ֲ���
  float outP;         //< pid�������֣�������
  float outI;         //< pid���ֲ��֣�������
  float outD;         //< pid΢�ֲ��֣�������
  float iLimit;       //< ��������
} pidsuite;

extern uint8_t armed;
extern pidsuite pid_out_Pitch;			//�������⻷pid
extern pidsuite pid_out_Roll;			//������⻷pid
extern pidsuite pid_out_Yaw;			//������⻷pid
extern pidsuite pid_in_Pitch;			//�������ڻ�pid
extern pidsuite pid_in_Roll;				//������ڻ�pid
extern pidsuite pid_in_Yaw;				//ƫ�����ڻ�pid
void PID_controllerInit(void);     //pid������ʼ��
void Control(float Throttle,float pitch_expect,float roll_expect,float yaw_expect);
void pidInit(pidsuite* pid, const float desired, const float kp, const float ki, const float kd);
void pidSetIntegralLimit(pidsuite* pid, const float limit);
float pidUpdate(pidsuite* pid, const float measured,float expect);
float pidErrorUpdate(pidsuite* pid, const float expect,float error);
#endif

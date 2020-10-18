#include "main.h"

#ifndef _PID
#define _PID

#define DEFAULT_PID_INTEGRATION_LIMIT  100.0
#define YM_Dead 1150

#define PID_FLASH_VERIFY 25

//************�⻷pid����**************************//
#define PID_OUT_PITCH_KP  50  //2.5
#define PID_OUT_PITCH_KI  0   //0
#define PID_OUT_PITCH_KD  0   //0
#define PID_OUT_PITCH_INTEGRATION_LIMIT   100.0 // 500.0

#define PID_OUT_ROLL_KP  50    //3
#define PID_OUT_ROLL_KI  0    //0
#define PID_OUT_ROLL_KD  0    //0
#define PID_OUT_ROLL_INTEGRATION_LIMIT    100.0 // 500.0

#define PID_OUT_YAW_KP  50    //3
#define PID_OUT_YAW_KI  0    //0
#define PID_OUT_YAW_KD  0    //0
#define PID_OUT_YAW_INTEGRATION_LIMIT 100.0

//************�ڻ�pid����**************************//
#define PID_IN_PITCH_KP  0.08    //0.5 
#define PID_IN_PITCH_KI  0.00   //0.003
#define PID_IN_PITCH_KD  0.1   //0.25
#define PID_IN_PITCH_INTEGRATION_LIMIT   200.0 // 500.0

#define PID_IN_ROLL_KP  0.08   //0.3
#define PID_IN_ROLL_KI  0.00   //0.003
#define PID_IN_ROLL_KD  0.1     //0.20
#define PID_IN_ROLL_INTEGRATION_LIMIT    200.0 // 500.0

#define PID_IN_YAW_KP   2.8       //3.0
#define PID_IN_YAW_KI   0        //0
#define PID_IN_YAW_KD   0        //0
#define PID_IN_YAW_INTEGRATION_LIMIT   100.0 // 200.0

#define CONTROL_MAX_ANGLE 25

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
#endif

#include "main.h"

#ifndef _PID
#define _PID

#define DEFAULT_PID_INTEGRATION_LIMIT  100.0
#define YM_Dead 1150

#define PID_FLASH_VERIFY 25


typedef struct
{
  float desired;      //< 被调量期望值
  float error;        //< 期望值-实际值
  float prevError;    //< 前一次偏差
  float integ;        //< 积分部分
  float deriv;        //< 微分部分
  float kp;           //< 比例参数
  float ki;           //< 积分参数
  float kd;           //< 微分参数
  float outP;         //< pid比例部分，调试用
  float outI;         //< pid积分部分，调试用
  float outD;         //< pid微分部分，调试用
  float iLimit;       //< 积分限制
} pidsuite;

extern uint8_t armed;
extern pidsuite pid_out_Pitch;			//俯仰角外环pid
extern pidsuite pid_out_Roll;			//横滚角外环pid
extern pidsuite pid_out_Yaw;			//横滚角外环pid
extern pidsuite pid_in_Pitch;			//俯仰角内环pid
extern pidsuite pid_in_Roll;				//横滚角内环pid
extern pidsuite pid_in_Yaw;				//偏航角内环pid
void PID_controllerInit(void);     //pid参数初始化
void Control(float Throttle,float pitch_expect,float roll_expect,float yaw_expect);
void pidInit(pidsuite* pid, const float desired, const float kp, const float ki, const float kd);
void pidSetIntegralLimit(pidsuite* pid, const float limit);
float pidUpdate(pidsuite* pid, const float measured,float expect);
float pidErrorUpdate(pidsuite* pid, const float expect,float error);
#endif

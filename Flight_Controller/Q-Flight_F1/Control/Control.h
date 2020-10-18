#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"
#include "tim.h"
#include "Utils.h"

// 速度控制器PID
#define PID_VELOCITY_X_KP 1.0f
#define PID_VELOCITY_X_KI 0.0f
#define PID_VELOCITY_X_KD 0.0f
#define PID_VELOCITY_X_INTEGRATION_LIMIT 100.0f

#define PID_VELOCITY_Y_KP 1.0f
#define PID_VELOCITY_Y_KI 0.0f
#define PID_VELOCITY_Y_KD 0.0f
#define PID_VELOCITY_Y_INTEGRATION_LIMIT 100.0f

#define PID_VELOCITY_Z_KP 1.0f
#define PID_VELOCITY_Z_KI 0.0f
#define PID_VELOCITY_Z_KD 0.0f
#define PID_VELOCITY_Z_INTEGRATION_LIMIT 100.0f

// 位置控制器PID
#define PID_POSITION_X_KP 1.0f
#define PID_POSITION_X_KI 0.0f
#define PID_POSITION_X_KD 0.0f
#define PID_POSITION_X_INTEGRATION_LIMIT 100.0f

#define PID_POSITION_Y_KP 1.0f
#define PID_POSITION_Y_KI 0.0f
#define PID_POSITION_Y_KD 0.0f
#define PID_POSITION_Y_INTEGRATION_LIMIT 100.0f

#define PID_POSITION_Z_KP 1.0f
#define PID_POSITION_Z_KI 0.0f
#define PID_POSITION_Z_KD 0.0f
#define PID_POSITION_Z_INTEGRATION_LIMIT 100.0f

// 基础油门
#define THROTTLE_BASE 1300
#define MIN_THROTTLE 1150

// 控制器限幅
#define MAX_VELOCITY_XY 1000 //10ms-1
#define MAX_VELOCITY_Z 500 //5ms-1
#define MAX_GYRORATE_OUTPUT 400
#define MAX_ANGLE_XY 30

//模式定义
#define REPORT_ONLY_MODE 0
#define MANUAL_MODE 1
#define FIXED_ALTITUDE_MODE 2
#define FIXED_POINT_MODE 3

//遥控最大速度
#define MAX_RC_VELOCITY_XY 200 //2ms-1
#define MAX_RC_VELOCITY_Z 100 //1ms-1

extern uint16_t pwm_moto1,pwm_moto2,pwm_moto3,pwm_moto4; 

void RCposition_Update(void);
void PID_ControllerInit(void);
void Quad_Control(void);
void Get_current_state(void);
void Velocity_Control(void);
void Position_Control(void);
void Motor_Control(void);
void Attitude_Control(void);
void Report_Control(void);

#endif

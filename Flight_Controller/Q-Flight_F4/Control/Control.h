#ifndef CONTROL_H
#define CONTROL_H

#include "main.h"
#include "tim.h"
#include "Utils.h"

// �ٶȿ�����PID
#define PID_VELOCITY_X_KP 1.1f
#define PID_VELOCITY_X_KI 0.0f
#define PID_VELOCITY_X_KD 1.28f
#define PID_VELOCITY_X_INTEGRATION_LIMIT 100.0f

#define PID_VELOCITY_Y_KP 1.1f
#define PID_VELOCITY_Y_KI 0.0f
#define PID_VELOCITY_Y_KD 1.28f
#define PID_VELOCITY_Y_INTEGRATION_LIMIT 100.0f

#define PID_VELOCITY_Z_KP 3.0f
#define PID_VELOCITY_Z_KI 4.0f
#define PID_VELOCITY_Z_KD 6.0f
#define PID_VELOCITY_Z_INTEGRATION_LIMIT 100.0f

// λ�ÿ�����PID
#define PID_POSITION_X_KP 1.0f
#define PID_POSITION_X_KI 0.0f
#define PID_POSITION_X_KD 0.0f
#define PID_POSITION_X_INTEGRATION_LIMIT 100.0f

#define PID_POSITION_Y_KP 1.0f
#define PID_POSITION_Y_KI 0.0f
#define PID_POSITION_Y_KD 0.0f
#define PID_POSITION_Y_INTEGRATION_LIMIT 100.0f

#define PID_POSITION_Z_KP 1.2f
#define PID_POSITION_Z_KI 0.0f
#define PID_POSITION_Z_KD 0.0f
#define PID_POSITION_Z_INTEGRATION_LIMIT 100.0f


//************�⻷pid����**************************//
#define PID_OUT_PITCH_KP  50  //2.5
#define PID_OUT_PITCH_KI  0   //0
#define PID_OUT_PITCH_KD  0   //0
#define PID_OUT_PITCH_INTEGRATION_LIMIT   100.0 // 500.0

#define PID_OUT_ROLL_KP  50    //3
#define PID_OUT_ROLL_KI  0    //0
#define PID_OUT_ROLL_KD  0    //0
#define PID_OUT_ROLL_INTEGRATION_LIMIT    100.0 // 500.0

#define PID_OUT_YAW_KP  15    //3
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

#define PID_IN_YAW_KP   0.2f       //3.0
#define PID_IN_YAW_KI   0        //0
#define PID_IN_YAW_KD   0.0f        //0
#define PID_IN_YAW_INTEGRATION_LIMIT   100.0 // 200.0

#define CONTROL_MAX_ANGLE 15

// ��������
#define THROTTLE_BASE 1420
#define MIN_THROTTLE 1150

// �������޷�
#define MAX_VELOCITY_XY 80 //10ms-1
#define MAX_VELOCITY_Z 100 //1ms-1
#define MAX_GYRORATE_OUTPUT 400
#define MAX_ANGLE_XY 15
// Yaw������ٶ�
#define MAX_VELOCITY_YAW 1500 

//ģʽ����
#define REPORT_ONLY_MODE 0
#define MANUAL_MODE 1
#define FIXED_ALTITUDE_MODE 2
#define FIXED_POINT_MODE 3

//ң������ٶ�
#define MAX_RC_VELOCITY_XY 50.0f //2ms-1
#define MAX_RC_VELOCITY_Z 50.0f //1ms-1

// Yawͣת�ж�
#define RC_YAW_THRESHOLD 1

//��ͣ�ж�RCvelocity
#define RC_VELOCITY_THRESHOLD 15

//�����ж�RCvelocity
#define RC_VELOCITY_XY_THRESHOLD 3

//�����������Χ
#define ERR_POS_XY_LIMIT 30.0f //30cm
#define ERR_POS_Z_LIMIT 10.0f //10cm

//�Ƿ���������ƣ������ã�
#define MOTOR_ENABLED 1

//������������
#define QUAD_CONTROL_BASE_PERIOD 2 //2ms

//��Ϳ�������߶�
#define MOTOR_ENABLE_MIN_HEIGHT 10

//��ͣ��������ֵ
#define UPDATE_THROTTLE_VELOCITY_Z 20.0f

#define MOTO_TIM htim1

extern uint16_t pwm_moto1,pwm_moto2,pwm_moto3,pwm_moto4; 
extern pidsuite pid_velocity_x,pid_velocity_y,pid_velocity_z; //�ٶȻ�pid
extern pidsuite pid_position_x,pid_position_y,pid_position_z; //λ�û�pid
extern uint8_t PID_Output_Mode,Enable_Motor;
void RCposition_Update(void);
void RCyaw_Update(void);
void PID_ControllerInit(void);
void Quad_Control(void);
void Get_current_state(void);
void Velocity_Control(void);
void Position_Control(void);
void Motor_Control(void);
void Attitude_Control(void);
void Report_Control(void);
void SetTargetHeight(float height);
float Calc_Yaw_Control_Coefficient(void);
#endif

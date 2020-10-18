#ifndef __VERBOSE_H
#define __VERBOSE_H

#include "main.h"
#include "stdio.h"
#include "Flash.h"

#define NUM_VERBOSE_MODES 4

void Work_Indicate(uint8_t status);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, uint32_t alt, uint8_t fly_model, uint8_t armed);
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int16_t bar);
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6);
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);
void ChangeVerbose(void);
void QF_Send_Status(float angle_rol, float angle_pit, float angle_yaw, uint32_t alt,
uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,
int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,
uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit);
void QF_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4);
void QF_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z);
void QF_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit);
void QF_Send_PID_Parameter(uint8_t mode);
void QF_Send_Response(uint8_t info);
void QF_Send_PID_Output(void);
#endif

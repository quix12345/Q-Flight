#ifndef __REMOTECONTROL_H
#define __REMOTECONTROL_H

#include "main.h"
#include "Control.h"

#define REMOTER_USART USART2
#define UART_CONTENT_BUFFER_SIZE 150

extern uint8_t RCthrottle,RCyaw,RCroll,RCpitch;
extern Vector3 RCvelocity;

void NightLed_Setting(void);
void TargetState_Setting(void);
void Flash_Setting(uint8_t pointer);
void Other_Setting(void);
void Verbose_Setting(void);
void Remote_Prase(void);
void Uart_Flag_Update(void);

#endif

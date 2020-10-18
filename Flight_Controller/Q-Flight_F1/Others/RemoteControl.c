#include "RemoteControl.h"
uint8_t sum;
uint8_t cnt_abc;
uint8_t RCthrottle,RCyaw,RCroll,RCpitch;
Vector3 RCvelocity;
extern quadstate_t target_state,current_state;

void NightLed_Setting(){
	if(content[2]==0xC1){
		HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port,NIGHT_LED_Pin,GPIO_PIN_RESET);
	}
	else if(content[2]==0xC2){
		HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port,NIGHT_LED_Pin,GPIO_PIN_SET);
	}
	else if(content[2]==0xC3){
		HAL_GPIO_TogglePin(NIGHT_LED_GPIO_Port,NIGHT_LED_Pin);
	}
	else if(content[2]==0xC4){
		HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port,NIGHT_LED_Pin,GPIO_PIN_SET);
		Flash_Change_once(LIGHT_TOGGLE_INDEX,1);
	}
	else if(content[2]==0xC5){
		HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port,NIGHT_LED_Pin,GPIO_PIN_RESET);
		Flash_Change_once(LIGHT_TOGGLE_INDEX,0);
	}
}

void TargetState_Setting(){
	RCthrottle=content[2]-1;
	RCyaw=content[3]-1;
	RCpitch=content[4]-1;
	RCroll=content[5]-1;
	float angle_const=control_max_angle/50.0;
	if(Control_Mode==REPORT_ONLY_MODE){
		return;
	}
	else if(Control_Mode==MANUAL_MODE){
		target_state.attitude.roll=((float)RCroll-50)*angle_const+roll_offset;
		target_state.attitude.pitch=((float)RCpitch-50)*angle_const+pitch_offset;
		target_state.attitude.yaw=((float)RCyaw-50)*angle_const;
		throttle=(uint16_t)((float)RCthrottle*6.50)+1100; // 油门限幅 0.65(throttle%)+0.1
	}
	else if(Control_Mode==FIXED_ALTITUDE_MODE){
		float velocity_z_const=MAX_RC_VELOCITY_Z/50;
		target_state.attitude.roll=((float)RCroll-50)*angle_const+roll_offset;
		target_state.attitude.pitch=((float)RCpitch-50)*angle_const+pitch_offset;
		target_state.attitude.yaw=((float)RCyaw-50)*angle_const;
		RCvelocity.z=((float)RCthrottle-50)*velocity_z_const;
	}
	else if(Control_Mode==FIXED_POINT_MODE){
		float velocity_xy_const=MAX_RC_VELOCITY_XY/50;
		float velocity_z_const=MAX_RC_VELOCITY_Z/50;
		RCvelocity.x=((float)RCpitch-50)*velocity_xy_const; // pitch对应x
		RCvelocity.y=((float)RCroll-50)*velocity_xy_const;  // roll对应y
		RCvelocity.z=((float)RCthrottle-50)*velocity_z_const;  // roll对应y
		target_state.attitude.yaw=((float)RCyaw-50)*angle_const;
	}
}

void Flash_Setting(uint8_t pointer){
	uint8_t verbose_old=verbose;
	verbose=0;
	if(content[2]==0xC1){
		uint8_t index=content[3];
		uint16_t number=(content[4]<<8)+content[5];
		float result = (float)number/(pow(10,(int)content[6]));
		Flash_Change_once(index,Flash2Int(result));
		HAL_NVIC_SystemReset();
	}
	else if(content[2]==0xC2){
		uint8_t mode = content[3];
		QF_Send_PID_Parameter(mode);
	}
	else if(content[2]==0xC3){
		Flash_Dataset_read();
		uint8_t cnt=3;
		while(content[cnt]!=0xC9){
			uint8_t index=content[cnt++];
			uint16_t number=(content[cnt]<<8)+content[cnt+1];
			cnt+=2;
			float result = (float)number/(pow(10,(int)content[cnt++]));
			Dataset[index]=Flash2Int(result);
		}
		Flash_Write_all();
		HAL_NVIC_SystemReset();
	}
	else if(content[2]==0xC4){
		uint8_t index=content[3];
		uint32_t number=(content[4]<<24)|((content[5]<<16)|((content[6]<<8)|content[7]));
		Flash_Change_once(index,number);
		QF_Send_Response(0xC1);
		HAL_NVIC_SystemReset();
	}
	else if(content[2]==0xC5){
		Flash_Dataset_read();
		cnt_abc=3;
		while(cnt_abc<pointer-1){
			uint8_t index=content[cnt_abc++];
			uint32_t number=(content[cnt_abc]<<24)|((content[cnt_abc+1]<<16)|((content[cnt_abc+2]<<8)|content[cnt_abc+3]));
			cnt_abc+=4;
			Dataset[index]=number;
		}
		Flash_Write_all();
		QF_Send_Response(0xC1);
		HAL_NVIC_SystemReset();
	}
	verbose=verbose_old;
}

void Other_Setting(){
	if(content[2]==0xC1){
//	PWN_Motor_Force_Stop(htim1);
//	HAL_Delay(200);
		Flash_Clear();
		HAL_NVIC_SystemReset();
	}
	else if(content[2]==0xC2){
		HAL_NVIC_SystemReset();
	}
	else if(content[2]==0xC3){
		PWN_Motor_Force_Stop(htim1);
		HAL_Delay(1000);
	}
	else if(content[2]==0xC4){
		PWN_Motor_Force_Stop(htim1);
		while(1);
	}
	else if(content[2]==0xC5){
		Flash_Dataset_read();
		Dataset[GYRO_X_OFFSET]=0;
		Dataset[GYRO_Y_OFFSET]=0;
		Dataset[GYRO_Z_OFFSET]=0;
		Flash_Write_all();
	}
	else if(content[2]==0xC6){
		Flash_Dataset_read();
		Dataset[PITCH_OFFSET_INDEX]=0;
		Dataset[ROLL_OFFSET_INDEX]=0;
		Flash_Write_all();
	}
}

void Verbose_Setting(){
		if(content[2]==0xC1){
			ChangeVerbose();
		}
		else if(content[2]==0xC2){
			while(verbose!=0){
				verbose=0;
			}
		}
		else if(content[2]==0xC3){
			while(verbose!=2){
				verbose=2;
			}
		}
		else if(content[2]==0xC4){
				QF_Send_Response(0xC0);
		}
}

void Remote_Prase(){
	static uint8_t head_toggle=0;
	static uint8_t pointer=0;
	uint8_t tmp;
	if(LL_USART_IsActiveFlag_RXNE(USART2))
	{
		tmp=LL_USART_ReceiveData8(USART2);
		if(pointer>=UART_CONTENT_BUFFER_SIZE){
				memset(content,0x00,sizeof(content));
				pointer=0;
		}
		if(tmp==0xC8){
			head_toggle=1;
			pointer=0;
		}
		if(head_toggle){
			content[pointer++]=tmp;
			if(content[pointer-1]==0xC9){	
				num_of_commands++;
				head_toggle=0;
				uint8_t sum_pointer=pointer-2;			
				sum=0;
				for(int i=0;i<sum_pointer;i++){
						sum+=content[i];
				}
				if(content[0]==0xC8&&content[sum_pointer+1]==0xC9&&sum==content[sum_pointer]){
					Uart_Flag=1;
					num_of_success_commands++;
					if(content[1]==0xC1){
						TargetState_Setting();
					}
					else if(content[1]==0xC2){
						Flash_Setting(pointer);
					}
					else if(content[1]==0xC3){
						NightLed_Setting();
					}
					else if(content[1]==0xC4){
						Other_Setting();
					}
					else if(content[1]==0xC5){
					  Verbose_Setting();
					}
				}
				memset(content,0x00,sizeof(content));
			}
		}
	}
}

void Uart_Flag_Update(){
	static uint8_t RC_count,Opt_count=0;
	if(RC_count++>=50){
		if(Uart_Flag==1){
			Uart_Flag=0;
		}
		else{
			PWN_Motor_Force_Stop(htim1);
		}
		RC_count=0;
	}
	if(Opt_count++>=3){
		if(LC306_Uart_Flag==1){
			LC306_Uart_Flag=0;
		}
		Opt_count=0;
	}
}

#include "SysSetting.h"
#include "PID.h"
#include "Flash.h"

uint32_t Quad_Parameter[QUAD_PARAMETER_LENGTH];

void Flash_Setting_Init(){
	//保证默认参数
	if(Flash_Read(0)!=FLASH_VERIFY){
//		uint32_t Initial_Parameter[QUAD_PARAMETER_LENGTH]={FLASH_VERIFY,Flash2Int(PID_OUT_PITCH_KP), Flash2Int(PID_OUT_PITCH_KI), Flash2Int(PID_OUT_PITCH_KD),
//		Flash2Int(PID_OUT_ROLL_KP), Flash2Int(PID_OUT_ROLL_KI), Flash2Int(PID_OUT_ROLL_KD),
//		Flash2Int(PID_OUT_YAW_KP), Flash2Int(PID_OUT_YAW_KI), Flash2Int(PID_OUT_YAW_KD),
//		Flash2Int(PID_IN_PITCH_KP), Flash2Int(PID_IN_PITCH_KI), Flash2Int(PID_IN_PITCH_KD),
//		Flash2Int(PID_IN_ROLL_KP), Flash2Int(PID_IN_ROLL_KI), Flash2Int(PID_IN_ROLL_KD),
//		Flash2Int(PID_IN_YAW_KP),Flash2Int(PID_IN_YAW_KI),Flash2Int(PID_IN_YAW_KD),
//		PID_OUT_PITCH_INTEGRATION_LIMIT,PID_OUT_ROLL_INTEGRATION_LIMIT,PID_OUT_YAW_INTEGRATION_LIMIT,
//		PID_IN_PITCH_INTEGRATION_LIMIT,PID_IN_ROLL_INTEGRATION_LIMIT,PID_IN_YAW_INTEGRATION_LIMIT,
//		CONTROL_MAX_ANGLE,2,0,0,0,0,(int32_t)PITCH_OFFSET,(int32_t)ROLL_OFFSET};
		
		uint32_t Initial_Parameter[QUAD_PARAMETER_LENGTH]={FLASH_VERIFY,
		//姿态控制器
		Flash2Int(PID_OUT_PITCH_KP), Flash2Int(PID_OUT_PITCH_KI), Flash2Int(PID_OUT_PITCH_KD),
		Flash2Int(PID_OUT_ROLL_KP), Flash2Int(PID_OUT_ROLL_KI), Flash2Int(PID_OUT_ROLL_KD),
		Flash2Int(PID_OUT_YAW_KP), Flash2Int(PID_OUT_YAW_KI), Flash2Int(PID_OUT_YAW_KD),
		Flash2Int(PID_IN_PITCH_KP), Flash2Int(PID_IN_PITCH_KI), Flash2Int(PID_IN_PITCH_KD),
		Flash2Int(PID_IN_ROLL_KP), Flash2Int(PID_IN_ROLL_KI), Flash2Int(PID_IN_ROLL_KD),
		Flash2Int(PID_IN_YAW_KP),Flash2Int(PID_IN_YAW_KI),Flash2Int(PID_IN_YAW_KD),
			
		PID_OUT_PITCH_INTEGRATION_LIMIT,PID_OUT_ROLL_INTEGRATION_LIMIT,PID_OUT_YAW_INTEGRATION_LIMIT,
		PID_IN_PITCH_INTEGRATION_LIMIT,PID_IN_ROLL_INTEGRATION_LIMIT,PID_IN_YAW_INTEGRATION_LIMIT,
			
		//位置&速度控制器
		Flash2Int(PID_VELOCITY_X_KP),Flash2Int(PID_VELOCITY_X_KI), Flash2Int(PID_VELOCITY_X_KD),
		Flash2Int(PID_VELOCITY_Y_KP),Flash2Int(PID_VELOCITY_Y_KI), Flash2Int(PID_VELOCITY_Y_KD),
		Flash2Int(PID_VELOCITY_Z_KP),Flash2Int(PID_VELOCITY_Z_KI), Flash2Int(PID_VELOCITY_Z_KD),		
		Flash2Int(PID_POSITION_X_KP),Flash2Int(PID_POSITION_X_KI), Flash2Int(PID_POSITION_X_KD),
		Flash2Int(PID_POSITION_Y_KP),Flash2Int(PID_POSITION_Y_KI), Flash2Int(PID_POSITION_Y_KD),
		Flash2Int(PID_POSITION_Z_KP),Flash2Int(PID_POSITION_Z_KI), Flash2Int(PID_POSITION_Z_KD),				
			
		PID_VELOCITY_X_INTEGRATION_LIMIT,PID_VELOCITY_Y_INTEGRATION_LIMIT,PID_VELOCITY_Z_INTEGRATION_LIMIT,
		PID_POSITION_X_INTEGRATION_LIMIT,PID_POSITION_Y_INTEGRATION_LIMIT,PID_POSITION_Z_INTEGRATION_LIMIT,
			
		//其他参数
		CONTROL_MAX_ANGLE,2,0,0,0,0,(uint32_t)PITCH_OFFSET,(uint32_t)ROLL_OFFSET};
		
		Flash_Dataset_write(Initial_Parameter,QUAD_PARAMETER_LENGTH);
	}
	
	Flash_Dataset_read_out(Quad_Parameter);
	pitch_offset=(float)((int32_t)Quad_Parameter[PITCH_OFFSET_INDEX])/100;
	roll_offset=(float)((int32_t)Quad_Parameter[ROLL_OFFSET_INDEX])/100;
	control_max_angle=Quad_Parameter[MAX_CONTROL_ANGLE_INDEX];
	verbose=Quad_Parameter[VERBOSE_INDEX];
	Light_Toggle=Quad_Parameter[LIGHT_TOGGLE_INDEX];
	if(Light_Toggle){
		HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port,NIGHT_LED_Pin,GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port,NIGHT_LED_Pin,GPIO_PIN_RESET);
	}
	
	gyro_offset[0]=Quad_Parameter[GYRO_X_OFFSET];
	gyro_offset[1]=Quad_Parameter[GYRO_Y_OFFSET];
	gyro_offset[2]=Quad_Parameter[GYRO_Z_OFFSET];

	if(gyro_offset[0]==0&&gyro_offset[1]==0&&gyro_offset[2]==0){
		MPU9250_CalcGyroOffset();
		Flash_Dataset_read();
		Dataset[GYRO_X_OFFSET]=gyro_offset[0];
		Dataset[GYRO_Y_OFFSET]=gyro_offset[1];
		Dataset[GYRO_Z_OFFSET]=gyro_offset[2];
		Flash_Write_all();
	}
	
	for(int i=0;i<QUAD_PARAMETER_LENGTH;i++){
		if(i>=1&&i<=18){
			printf("%.2f ",(float)Flash2Float(Quad_Parameter[i]));
		}
		else if(i>=25&&i<=42){
			printf("%.2f ",(float)Flash2Float(Quad_Parameter[i]));
		}
		else{
			printf("%d ",Quad_Parameter[i]);
		}
	}
	printf("\r\n");
}

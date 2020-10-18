#include "Flash.h"
 
FLASH_EraseInitTypeDef EraseInitStruct; 
FLASH_DATA_TYPE Dataset[DATASET_LENGTH];



void Flash_Dataset_read(){
	for(int i=0;i<DATASET_LENGTH;i++){
		Dataset[i]=Flash_Read(i);
	}
}

void Flash_Dataset_read_out(FLASH_DATA_TYPE *Dataset_new){
	Flash_Dataset_read();
	for(int i=0;i<DATASET_LENGTH;i++){
		Dataset_new[i]=Dataset[i];
	}
}

void Flash_Print(void){
	Flash_Dataset_read();
	printf("Flash content: ");
	for(int i=0;i<DATASET_LENGTH;i++){
		printf("%d ",Dataset[i]);
	}
	printf("\r\n");
}


void Flash_Change_once(FLASH_DATA_TYPE addr,FLASH_DATA_TYPE data){
	Flash_Dataset_read();
	Dataset[addr]=data;
	Flash_Write_all();
}

void Flash_Write_all(void){
	Flash_Clear();
	for(int i=0;i<DATASET_LENGTH;i++){
		Flash_Write_only(i,Dataset[i]);
	}
}

void Flash_Dataset_copy(FLASH_DATA_TYPE *dataset_new,uint8_t length){
	if(length>=DATASET_LENGTH){
		for(int i=0;i<DATASET_LENGTH;i++){
			Dataset[i] =dataset_new[i];
		}
	}
	else{
		for(int i=0;i<length;i++){
			Dataset[i] =dataset_new[i];
		}
	}
}

void Flash_Dataset_write(FLASH_DATA_TYPE *dataset_new,uint8_t length){
	Flash_Dataset_copy(dataset_new,length);
	Flash_Write_all();
}

void Flash_Clear(){
		HAL_FLASH_Unlock();   
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;  //??Flash??????????
    EraseInitStruct.PageAddress = START_ADDR;  //????????
		FLASH_DATA_TYPE PageError = 0;
    EraseInitStruct.NbPages = 1;   
    if (HAL_FLASHEx_Erase(&EraseInitStruct,&PageError) != HAL_OK) //???????????0x0801FC00,??????????,??OK
    {
        printf("Erase error\r\n");
    }
		 HAL_FLASH_Lock();
}

void Flash_Write_only(FLASH_DATA_TYPE addr,FLASH_DATA_TYPE data){
	  HAL_FLASH_Unlock();     //??
		addr*=4;
		addr+=START_ADDR;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,addr, data); //?FLASH???
    HAL_FLASH_Lock();
}

/**
  * @brief  ?FLASH??????,?????????????,??????,?????127??
  * @param  ?STM32?FLASH??????EEPROM???
  * @retval ??:???????
  */
void Flash_Write(FLASH_DATA_TYPE addr,FLASH_DATA_TYPE data)
{
	  Flash_Clear();
    HAL_FLASH_Unlock();     //??
		addr*=4;
		addr+=START_ADDR;
    FLASH_DATA_TYPE writeFlashData = data;       
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,addr, writeFlashData); //?FLASH???
    HAL_FLASH_Lock();
}
/**
  * @brief  ??????????
  * @param  ?STM32?FLASH??????EEPROM???
  * @retval ???:?FLASH?????
  */
FLASH_DATA_TYPE Flash_Read(FLASH_DATA_TYPE addr)
{
    HAL_FLASH_Unlock();
    FLASH_DATA_TYPE Page = 0;
		addr*=4;
		addr+=START_ADDR;
    Page=*(__IO FLASH_DATA_TYPE*)addr;
    return Page;
}

FLASH_DATA_TYPE Flash2Int(double input){
	return (FLASH_DATA_TYPE)(input*10000);
}

double Flash2Float(FLASH_DATA_TYPE input){
	double output;
	output=(double)input/10000;
	return output;
}

#include "main.h"
#include "stdio.h"

#ifndef _FLASH_H
#define _FLASH_H

#define START_ADDR 0x0801FC00
#define DATASET_LENGTH 57
#define FLASH_DATA_TYPE uint32_t
extern FLASH_DATA_TYPE Dataset[DATASET_LENGTH];
void Flash_Dataset_read(void);
void Flash_Clear(void);
void Flash_Write(FLASH_DATA_TYPE addr,FLASH_DATA_TYPE data);
FLASH_DATA_TYPE Flash_Read(FLASH_DATA_TYPE addr);
void Flash_Write_only(FLASH_DATA_TYPE addr,FLASH_DATA_TYPE data);
void Flash_Change_once(FLASH_DATA_TYPE addr,FLASH_DATA_TYPE data);
void Flash_Write_all(void);
void Flash_Dataset_copy(FLASH_DATA_TYPE *dataset_new,uint8_t length);
void Flash_Print(void);
void Flash_Dataset_write(FLASH_DATA_TYPE *dataset_new,uint8_t length);
FLASH_DATA_TYPE Flash2Int(double input);
void Flash_Dataset_read_out(FLASH_DATA_TYPE *dataset_new);
double Flash2Float(FLASH_DATA_TYPE input);
#endif

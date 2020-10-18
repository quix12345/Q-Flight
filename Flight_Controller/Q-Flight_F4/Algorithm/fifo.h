#include "main.h"

#ifndef __FIFO_H
#define __FIFO_H

#define FIFO_FULL 1;
#define FIFO_OK 0;
typedef struct
{
	int length;
	uint8_t isempty;
	uint8_t write_pointer;
	uint8_t read_pointer;
	uint8_t buffer[25];
}Fifo_TypeDef;

void fifo_init(Fifo_TypeDef * fifo);
void fifo_clear(Fifo_TypeDef * fifo);
uint8_t fifo_read(Fifo_TypeDef *fifo);
uint8_t fifo_write(Fifo_TypeDef *fifo,uint8_t info);
void fifo_read_message(Fifo_TypeDef *fifo,uint8_t *buffer);
uint8_t is_fifo_message_complete(Fifo_TypeDef *fifo);
uint8_t wait_fifo_message(Fifo_TypeDef *fifo);
uint8_t custom2_fifo_message_complete(Fifo_TypeDef *fifo,uint8_t custom_sign1,uint8_t custom_sign2);
uint8_t custom_fifo_message_complete(Fifo_TypeDef *fifo,uint8_t custom_sign);
#endif

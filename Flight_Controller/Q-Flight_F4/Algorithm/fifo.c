#include "fifo.h"
#include <string.h>

void fifo_init(Fifo_TypeDef * fifo){
	fifo->write_pointer=0;
	fifo->read_pointer=0;
	fifo->length=(int)(sizeof(fifo->buffer)/sizeof(uint8_t));
	fifo->isempty=1;
	memset(fifo->buffer,0x00,sizeof(fifo->buffer));
}

void fifo_clear(Fifo_TypeDef * fifo){
	fifo->write_pointer=0;
	fifo->read_pointer=0;
	fifo->isempty=1;
	memset(fifo->buffer,0x00,sizeof(fifo->buffer));
}


uint8_t fifo_read(Fifo_TypeDef *fifo){
	uint8_t current_pointer=fifo->read_pointer;
	uint8_t info=fifo->buffer[current_pointer];
	fifo->buffer[current_pointer]=0x00;
	if(info==0x00){
		fifo->isempty=1;
	}
	else{
		fifo->isempty=0;
		fifo->read_pointer++;
		if(fifo->read_pointer>fifo->length-1){
			fifo->read_pointer=0;
		}
	}
	return info;
}

uint8_t fifo_write(Fifo_TypeDef *fifo,uint8_t info){
	fifo->isempty=0;
	if(fifo->buffer[fifo->write_pointer]==0x00){
		fifo->buffer[fifo->write_pointer]=info;
	}
	else{
		fifo_clear(fifo);
		return FIFO_FULL;
	}
	if(++fifo->write_pointer>fifo->length-1){
		fifo->write_pointer=0;
	}
	return FIFO_OK;
}

void fifo_read_message(Fifo_TypeDef *fifo,uint8_t *buffer){
	int i=0;
//	memset(buffer,0x00,sizeof(buffer)/sizeof(uint8_t));
	while(!fifo->isempty){
		buffer[i++]=fifo_read(fifo);
	}
}

uint8_t is_fifo_message_complete(Fifo_TypeDef *fifo){
	int previous_pointer=fifo->write_pointer-1;
	if(previous_pointer<0){
		previous_pointer+=fifo->length-1;
	}
	if(fifo->buffer[previous_pointer]!=0x0A){
		return 0;
	}
	previous_pointer-=1;
		if(previous_pointer<0){
		previous_pointer+=fifo->length-1;
	}
			if(fifo->buffer[previous_pointer]!=0x0D){
		return 0;
	}
	return 1;
}

uint8_t custom2_fifo_message_complete(Fifo_TypeDef *fifo,uint8_t custom_sign1,uint8_t custom_sign2){
	int previous_pointer=fifo->write_pointer-1;
	if(previous_pointer<0){
		previous_pointer+=fifo->length-1;
	}
	if(fifo->buffer[previous_pointer]!=custom_sign1){
		return 1;
	}
	previous_pointer-=1;
	if(previous_pointer<0){
		previous_pointer+=fifo->length-1;
	}
	if(fifo->buffer[previous_pointer]!=custom_sign2){
		return 0;
	}
	return 0;
}

uint8_t custom_fifo_message_complete(Fifo_TypeDef *fifo,uint8_t custom_sign){
	int previous_pointer=fifo->write_pointer-1;
	if(previous_pointer<0){
		previous_pointer+=fifo->length-1;
	}
	if(fifo->buffer[previous_pointer]!=custom_sign){
		return 1;
	}
	return 0;
}


uint8_t wait_fifo_message(Fifo_TypeDef *fifo){
//	if(fifo->buffer[fifo->write_pointer]!=0x00){
//		fifo_clear(fifo);
//	}
	return !is_fifo_message_complete(fifo);
}

/*
 * circ_buffer.h
 *
 *  Created on: Mar 22, 2025
 *      Author: selenevrenkaya
 */

#ifndef GPS_CIRC_BUFFER_H_
#define GPS_CIRC_BUFFER_H_

#include"stdint.h"
#include "stdio.h"
#include"string.h"
#include"stdbool.h"


#define circ_buffer_size	(1 << 10)



typedef struct circ_buffer{
	unsigned char buffer[circ_buffer_size];
	//volatile uint32_t size;		/* will change in interrupt routines */
	//volatile uint32_t read_pos;	// volatile?
	//volatile uint32_t write_pos;
	uint32_t size;
	uint32_t read_pos;
	uint32_t write_pos;
	unsigned char sentence;

}t_circ_buffer;







bool circ_buffer_init(t_circ_buffer *cbuffer, uint8_t *cb, uint32_t buffer_size);

uint32_t circ_buffer_length(t_circ_buffer *cbuffer);

bool circ_buffer_write(t_circ_buffer *cbuffer, unsigned char char_data);
bool circ_buffer_write_block(t_circ_buffer *cbuffer, unsigned char *writeBlock, uint32_t buffer_size);

bool circ_buffer_read(t_circ_buffer *cbuffer, unsigned char *read_char);
bool circ_buffer_read_block(t_circ_buffer *cbuffer, unsigned char *readBlock, uint32_t buffer_size);

bool circ_buffer_peek(t_circ_buffer *cbuffer, uint8_t *destination, uint32_t index);



#endif /* GPS_CIRC_BUFFER_H_ */

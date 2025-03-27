/*
 * circ_buffer.h
 *
 *  Created on: Mar 22, 2025
 *      Author: selenevrenkaya
 */

#ifndef COMM_CIRC_BUFFER_H_
#define COMM_CIRC_BUFFER_H_

#include"stdint.h"
#include"string.h"
#include"stdbool.h"


#define circ_buffer_size	1 << 8



typedef struct circ_buffer{
	unsigned char *buffer;
	uint32_t size;
	uint32_t read_pos;
	uint32_t write_pos;
	uint8_t sentence;

}t_circ_buffer;







bool circ_buffer_init(tCircBuffer *cbuffer, uint8_t *cb, uint32_t bufferSize);
uint32_t circ_buffer_length(tCircBuffer *cbuffer);
bool circ_buffer_write(tCircBuffer *cbuffer, uint8_t *writeBlock, uint32_t blockSize);
bool circ_buffer_read(tCircBuffer *cbuffer, uint8_t *readBlock, uint32_t blockSize);
bool circ_buffer_peek(tCircBuffer *cbuffer, uint8_t *destination, uint32_t index);



#endif /* COMM_CIRC_BUFFER_H_ */

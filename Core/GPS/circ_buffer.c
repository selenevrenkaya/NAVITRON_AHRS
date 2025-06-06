/*
 * circ_buffer.c
 *
 *  Created on: Mar 22, 2025
 *      Author: selenevrenkaya
 */


#include "circ_buffer.h"


bool circ_buffer_init(t_circ_buffer *cbuffer, uint8_t *cb, uint32_t buffer_size){
/*
	cbuffer->buffer = cb;
	if (cbuffer->buffer == NULL){
		return false;
	}
*/
	cbuffer->read_pos = 0;
	cbuffer->write_pos = 0;
	cbuffer->size = buffer_size;
	return true;
}

/* buffer buyuklugune gore okunabilir alani hesaplar */
uint32_t circ_buffer_length(t_circ_buffer *cbuffer){
	return ((cbuffer->write_pos - cbuffer->read_pos) & (cbuffer->size - 1));

}

/* sends the string to the buffer
 */
/*
void circ_buffer_write_string(const char *s)
{
	while(*s) circ_buffer_write(*s++);
}
*/

/* Write data to circular buffer */
bool circ_buffer_write(t_circ_buffer *cbuffer, unsigned char char_data){

	if(circ_buffer_length(cbuffer) == (cbuffer->size)){
		return false;
	}

	cbuffer->buffer[cbuffer->write_pos] = char_data;
	cbuffer->write_pos = (cbuffer->write_pos + 1) & (cbuffer->size - 1);

	return true ;
}


bool circ_buffer_write_block(t_circ_buffer *cbuffer, unsigned char *writeBlock, uint32_t block_size){
	/* yazilabilecek alan yeterli mi kontrolu */
    if (block_size <= (cbuffer->size - 1) - circ_buffer_length(cbuffer)){
    	uint32_t availableSpace = (cbuffer->size-1) - cbuffer->write_pos;

    	if (block_size < availableSpace){
    		memcpy(cbuffer->buffer + cbuffer->write_pos, writeBlock, block_size);
    		cbuffer->write_pos += block_size;
    	}
    	else {
    		memcpy(cbuffer->buffer + cbuffer->write_pos, writeBlock, availableSpace);
    		memcpy(cbuffer->buffer, writeBlock + availableSpace, block_size - availableSpace);
    		cbuffer->write_pos = block_size - availableSpace;
    	}

    	return true;
    }
    return false;

}


/* Read data from circular buffer */
bool circ_buffer_read(t_circ_buffer *cbuffer, unsigned char *read_char){

	if((circ_buffer_length(cbuffer)) == 0){
		return false;
	}

	*read_char = cbuffer->buffer[cbuffer->read_pos];
	cbuffer->read_pos = (cbuffer->read_pos + 1) & (cbuffer->size - 1);

	return true ;
}


bool circ_buffer_read_block(t_circ_buffer *cbuffer, unsigned char *readBlock, uint32_t block_size){
	/* bufferdan okunmasi istenen kadar yer var mi kontrolu */
	if(circ_buffer_length(cbuffer) >=  block_size){
		uint32_t readableSpace = cbuffer->size - cbuffer->read_pos;

		if(block_size < readableSpace) {
			memcpy(readBlock, cbuffer->buffer + cbuffer->read_pos, block_size);
			cbuffer->read_pos += block_size;
		}
	    else{
	    	memcpy(readBlock, cbuffer->buffer + cbuffer->read_pos, readableSpace);
        	memcpy(readBlock + readableSpace, cbuffer->buffer, block_size - readableSpace);
        	cbuffer->read_pos = block_size - readableSpace;
	    }

		return true;
	}
	return false;

}


/* bufferda kontrol edilmesi istenen data var mi kontrolu */
bool circ_buffer_peek(t_circ_buffer *cbuffer, unsigned char *destination, uint32_t index){

	if(circ_buffer_length(cbuffer) >= index) {
		*destination = cbuffer->buffer[(cbuffer->read_pos + index) & (cbuffer->size-1)];

		return true;
	}
    return false;
}




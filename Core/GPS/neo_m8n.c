/*
 * no_m8n.c
 *
 *  Created on: Mar 27, 2025
 *      Author: Selen Evrenkaya
 */


#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#include "neo_m8n.h"
#include "ahrs.h"

#define knot_2_mps 		0.51444f


extern UART_HandleTypeDef huart4;

extern t_circ_buffer circ_buffer;
extern t_gps gps_data;
extern tAHRS ahrs;

int hour = 0, min = 0, day = 0, mon = 0, year = 0;


/* Check the sentence to find gga or rmc packet */
uint8_t check_sentence(t_gps *gps, char *string){
	//t_gps *temp_gps;

	uint32_t index = 0;
	uint8_t charc = 0;
	int len = strlen(string);

	while(circ_buffer_length(gps->cbuffer) != 0){

		for(index = 0; index < circ_buffer_size; index++){

			if(circ_buffer_peek(gps->cbuffer, &(gps->cbuffer->sentence), index) == true){

				if(gps->cbuffer->sentence == string[charc]){
					charc++;
					if(charc == len){
						gps->cbuffer->read_pos = (gps->cbuffer->read_pos + index + 1) & (circ_buffer_size - 1);
						return 1;
					}


				}
				else {
					charc = 0;

				}

			}
			else{
				return 0;
			}

		}
	}
	return 0;
}


uint8_t parse_gps_data(t_gps *gps, char string, unsigned char *destination){
	uint32_t index = 0;
	uint8_t charc = 0;
	//int len = strlen(string);
	memset(destination, '\0', 100);

	while(circ_buffer_length(gps->cbuffer) != 0){
		for(index = 0; index < circ_buffer_size; index++){

			if(circ_buffer_peek(gps->cbuffer, &(gps->cbuffer->sentence), index) == true){

				if(gps->cbuffer->sentence != string){

					destination[charc] = gps->cbuffer->buffer[gps->cbuffer->read_pos + index];
					charc++;

					//if(circ_buffer_length(gps->cbuffer) == 0)	return 0;
				}

				else if(gps->cbuffer->sentence == string){
					destination[charc] = gps->cbuffer->buffer[gps->cbuffer->read_pos + index];

					gps->cbuffer->read_pos = (gps->cbuffer->read_pos + index) & (circ_buffer_size - 1);
					return 1;
				}
			}
			else	return 0;
		}
	}

	return 0;
}



uint8_t decode_gga(unsigned char *gga_buffer, t_gps_gga *gga){
	int i, index = 0;
	char temp_buffer[12];	// max 12 index

	/*
	 * GGA Packet Structure
	 * $GPGGA, time, latitude, NS, longitude, EW, data fix, number of satellite, hdop value, altitude1, unit1, altitude2, unit2, *CRC
	 *
	 * */
	for(i = 0; i < 6; i++){
		while(gga_buffer[index] != ','){
			index++;

			if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.
		}
		index = index + 1;
	}

	/* reached the character to identify the fix */
	if ((gga_buffer[index] == '1') || (gga_buffer[index] == '2') || (gga_buffer[index] == '6')){	/* 0 indicates no fix yet */
		gga->is_fix_valid = 1;
		index = 0;
	}
	else{
		gga->is_fix_valid = 0;
		return 0;
	}



	/***** time *****/
	memset(temp_buffer, '\0', sizeof(temp_buffer));
	//if(gga_buffer[index] == ',')		index++;

	while(gga_buffer[index] != ','){
		index++;

		if(index > 100)		return 0;
	}
	index++;

	i = 0;
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;

		if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.
	}

	/* hh mm ss */
	gga->time.hour = (atoi((char*)temp_buffer) / 10000) + (GMT / 100);;
	gga->time.min = ((atoi((char*)temp_buffer) / 100) % 100) + (GMT % 100);
	gga->time.sec = atoi((char*)temp_buffer) % 100;



	/***** lattitude *****/
	index++;
	i = 0;

	memset(temp_buffer, '\0', sizeof(temp_buffer));
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;

		if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.
	}

	int16_t num = (atoi(temp_buffer));   // char buffer --> integer num

	// ondaliksiz kisimdaki basamak sayisi
	int dec_digit = 0;
	while (temp_buffer[dec_digit] != '.'){
		dec_digit++;

		if(dec_digit > 10)		return 0;
	}
	dec_digit++;	// noktayi da saymak icin

	// ondalik kismi
	int digit = (strlen(temp_buffer)) - dec_digit;

	// tam kismi tam sayiya cevirme
	int dec = atoi((char*)temp_buffer + dec_digit);

	double latitude = (num / 100.0) + (dec / pow(10, (digit + 2)));  // 1234.56789 = 12.3456789 (12 derece 3456789 dakika)

	gga->location.latitude = latitude;
	index++;
	gga->location.NS = gga_buffer[index++];


	/***** longitude *****/
	index++;
	i = 0;

	memset(temp_buffer, '\0', sizeof(temp_buffer));
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;

		if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.
	}

	num = (atoi(temp_buffer));   // char buffer --> integer num

	// ondaliksiz kisimdaki basamak sayisi
	dec_digit = 0;
	while (temp_buffer[dec_digit] != '.'){
		dec_digit++;

		if(dec_digit > 10)		return 0;
	}
	dec_digit++;	// noktayi da saymak icin

	// ondalik kismi
	digit = (strlen(temp_buffer)) - dec_digit;

	// tam kismi tam sayiya cevirme
	dec = atoi((char*)temp_buffer + dec_digit);

	double longitude = (num / 100.0) + (dec / pow(10, (digit + 2)));  // 1234.56789 = 12.3456789 (12 derece 3456789 dakika)

	gga->location.longitude = longitude;
	index++;
	gga->location.EW = gga_buffer[index++];


	// skip data fix
	index ++;   // ',' after EW
	while(gga_buffer[index] != ','){
		index++;

		if(index > 100)		return 0;
	}


	/***** number of satellistes *****/
	index++;
	i = 0;

	memset(temp_buffer, '\0', sizeof(temp_buffer));
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;

		if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.
	}

	gga->numofsat = atoi(temp_buffer);



	/***** hdop value *****/
	// skip
	index ++;   // ',' after EW
	while(gga_buffer[index] != ','){
		index++;

		if(index > 100)		return 0;
	}



	/***** altitude1 *****/
	/* denizden yukseklik */
	index++;
	i = 0;

	memset(temp_buffer, '\0', sizeof(temp_buffer));
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;

		if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.
	}

	num = (atoi(temp_buffer));

	dec_digit = 0;
	while (temp_buffer[dec_digit] != '.'){
		dec_digit++;

		if(dec_digit > 10)	return 0;
	}

	dec_digit++;

	digit = (strlen(temp_buffer)) - dec_digit;
	dec = atoi ((char *)temp_buffer + dec_digit);

	float altitude = (num) + (dec / pow(10, (digit)));
	gga->alt.altitude = altitude;

	index++;
	gga->alt.unit = gga_buffer[index];

	return 1;	// success :)
}



uint8_t decode_rmc (unsigned char *rmc_buffer, t_gps_rmc *rmc){
	int i, index = 0;
	char temp_buffer[12];	// max 12 index

	/*
	 * RMC Packet Structure
	 * $GPRMC, UTC time, state, latitude, NS, longitude, EW, speed over ground (knot), course over ground (heading), date, magnetic declination, unit, mode, *CRC
	 *
	 * */

	while(rmc_buffer[index] != ','){
		index++;	// skip first

		if(index > 100)		return 0;
	}
	index++;
	while(rmc_buffer[index] != ','){
		index++;	// skip utc time

		if(index > 100)		return 0;
	}
	index++;

	if(rmc_buffer[index] == 'A')
		rmc->is_valid = 1;
	else
		rmc->is_valid = 0;

	if(rmc->is_valid == 1){
		index = index + 2;
		for(i = 0; i < 4; i++){
			while(rmc_buffer[index] != ','){
				index++;

				if(index > 100)		return 0;
			}
			index++;
		}


		/***** speed *****/
		//index++;
		i = 0;

		memset(temp_buffer, '\0', sizeof(temp_buffer));
		while (rmc_buffer[index] != ','){
			temp_buffer[i] = rmc_buffer[index];
			i++;
			index++;
			if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.

		}

		if(strlen(temp_buffer) > 0){

			int16_t num = (atoi(temp_buffer));
			int dec_digit = 0;
			while (temp_buffer[dec_digit] != '.'){
				dec_digit++;

				if(dec_digit > 10)	return 0;
			}
			dec_digit++;

			int digit = (strlen(temp_buffer)) - dec_digit;
			int dec = atoi((char*)temp_buffer + dec_digit);

			/* TODO: check this part */
			float speed_knot = num + (dec / pow(10, digit));
			float speed = speed_knot * knot_2_mps;	// converting knot to m/s

			rmc->speed = speed;		// m/s

		}
		//else	rmc->speed = 0.0;


		/***** course *****/
		index++;
		i = 0;

		memset(temp_buffer, '\0', sizeof(temp_buffer));
		while (rmc_buffer[index] != ','){
			temp_buffer[i] = rmc_buffer[index];
			i++;
			index++;

			if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.
		}

		if(strlen(temp_buffer) > 0){

			int16_t num = (atoi(temp_buffer));
			int dec_digit = 0;
			while (temp_buffer[dec_digit] != '.'){
				dec_digit++;

				if(dec_digit > 10)	return 0;
			}

			int digit = (strlen(temp_buffer)) - dec_digit;
			int dec = atoi((char*)temp_buffer + dec_digit);

			/* TODO: check this calculation */
			float course = (num / 100.0) + (dec / pow(10, (digit + 2)));	// kuzeye gore derece cinsinden
			rmc->heading = course;

			float deg2rad = M_PI / 180.0;
			float heading_rad = course * deg2rad;	// degree to radian

		    rmc->vN = rmc->speed * cosf(heading_rad);
		    rmc->vE = rmc->speed * sinf(heading_rad);
		    rmc->vD = 0.0f;		// generally 0

		}

		/* if there is no data, do not update */

		//else	rmc->heading = 0.0;


		/***** date *****/
		index++;
		i = 0;

		memset(temp_buffer, '\0', sizeof(temp_buffer));
		while (rmc_buffer[index] != ','){
			temp_buffer[i] = rmc_buffer[index];
			i++;
			index++;

			if(index > 100)		return 0;		// bufferin boyutundan buyuk degere giderse bitsin.
		}

		// package: day month year
		day = atoi(temp_buffer) / 10000;
		mon = (atoi(temp_buffer) / 100) % 100;
		year = atoi(temp_buffer) % 100;

		rmc->date.day = day;
		rmc->date.month = mon;
		rmc->date.year = year;

		return 1;

	}

	else	return 0;
}


void set_ahrs_data(t_gps *gps, tAHRS* ahrs){

	/*
	 * 	latitude;
	 * 	longitude;
	 * 	altitude;
	 * 	velocity
	 *
	 * */


	/* latitude, longitude, altitude in degree */
	ahrs->position.latitude = gps->gga.location.latitude;
	ahrs->position.longitude = gps->gga.location.longitude;
	ahrs->position.altitude = gps->gga.alt.altitude;


	ahrs->velocity.ned.north = gps->rmc.vN;
	ahrs->velocity.ned.east = gps->rmc.vE;
	ahrs->velocity.ned.down = gps->rmc.vD;

	/* time info */
	ahrs->time.hour = gps->gga.time.hour;
	ahrs->time.min = gps->gga.time.min;
	ahrs->time.day = gps->rmc.date.day;
	ahrs->time.month = gps->rmc.date.month;
	ahrs->time.year = 2000 + gps->rmc.date.year;

}



void uart_isr(UART_HandleTypeDef *huart){

	uint32_t isrflags   = READ_REG(huart->Instance->SR);
	uint32_t cr1its     = READ_REG(huart->Instance->CR1);

	/* if DR is not empty and the Rx Int is enabled */
	if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
	{
  	 /******************
  	    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
  	    	      *          error) and IDLE (Idle line detected) flags are cleared by software
  	    	      *          sequence: a read operation to USART_SR register followed by a read
  	    	      *          operation to USART_DR register.
  	    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
  	    	      * @note   TC flag can be also cleared by software sequence: a read operation to
  	    	      *          USART_SR register followed by a write operation to USART_DR register.
  	    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

  	 *********************/
		huart->Instance->SR;                       /* Read status register */
		unsigned char char_data = huart->Instance->DR;     /* Read data register */

		circ_buffer_write(gps_data.cbuffer, char_data);		/* write received data in buffer */
		gps_data.receive_check = true;

		return;
	}

}


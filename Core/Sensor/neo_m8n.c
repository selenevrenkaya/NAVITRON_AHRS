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


int hour = 0, min = 0, day = 0, mon = 0, year = 0;


/* Check the sentence to find gga or rmc packet */
uint8_t check_sentence(t_gps *gps, char *string){
	uint32_t index = 0;
	uint8_t charc = 0;
	int len = strlen(string);

	while(!circ_buffer_length(gps->cbuffer)){
		for(index = 0; index < circ_buffer_size; index++){

			if(circ_buffer_peek(gps->cbuffer, &(gps->cbuffer->sentence), index) == true){

				while(gps->cbuffer->sentence == string[charc]){
					charc++;
					if(charc == len)	return 1;
					else	return 0;
				}
			}
			else	return 0;

		}
	}
	return 0;
}


uint8_t parse_gps_data(t_gps *gps, char *string, char *destination){
	uint32_t index = 0;
	uint8_t charc = 0;
	int len = strlen(string);

	while(!circ_buffer_length(gps->cbuffer)){
		for(index = 0; index < circ_buffer_size; index++){

			if(circ_buffer_peek(gps->cbuffer, &(gps->cbuffer->sentence), index) == true){

				while(gps->cbuffer->sentence != string[charc]){
					destination[index] = gps->cbuffer->buffer[gps->cbuffer->read_pos];

					gps->cbuffer->read_pos = (gps->cbuffer->read_pos + 1) % circ_buffer_size;
					charc++;
					while(!circ_buffer_length(gps->cbuffer));
				}

				while(gps->cbuffer->sentence == string[charc]){
					destination[index] = gps->cbuffer->buffer[gps->cbuffer->read_pos];
					memcpy((uint8_t*)&(gps->cbuffer->buffer[gps->cbuffer->read_pos]), (uint8_t*)&(destination[index]), len);

					return 1;
				}
			}
			else	return 0;
		}
	}

	return 0;
}



uint8_t decode_gga(char *gga_buffer, t_gps_gga *gga){
	int i, index = 0;
	char temp_buffer[12];	// max 12 index
	/*
	 * GGA Packet Structure
	 * $GPGGA, time, latitude, NS, longitude, EW, data fix, number of satellite, hdop value, altitude1, unit1, altitude2, unit2, *CRC
	 *
	 * */
	for(i = 0; i < 6; i++){
		while(gga_buffer[index] != ',')		index++;
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
	while(gga_buffer[index] != ',')		index++;

	i = 0;
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;
	}

	/* hh mm ss */
	gga->time.hour = (atoi(gga_buffer) / 10000) + (GMT / 100);;
	gga->time.min = ((atoi(gga_buffer) / 100) % 100) + (GMT % 100);
	gga->time.sec = atoi(gga_buffer) % 100;



	/***** lattitude *****/
	index++;
	i = 0;

	memset(temp_buffer, '\0', sizeof(temp_buffer));
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;
	}

	int16_t num = (atoi(temp_buffer));   // char buffer --> integer num

	// ondaliksiz kisimdaki basamak sayisi
	int dec_digit = 0;
	while (temp_buffer[dec_digit] != '.')	dec_digit++;

	// ondalik kismi
	int digit = (strlen(temp_buffer)) - dec_digit;

	// tam kismi tam sayiya cevirme
	int dec = atoi((char*)temp_buffer + dec_digit);

	float latitude = (num / 100.0) + (dec / pow(10, (digit + 2)));  // 1234.56789 = 12.3456789 (12 derece 3456789 dakika)

	gga->location.latitude = latitude;
	gga->location.NS = gga_buffer[index++];



	/***** longitude *****/
	index++;
	i = 0;

	memset(temp_buffer, '\0', sizeof(temp_buffer));
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;
	}

	num = (atoi(temp_buffer));   // char buffer --> integer num

	// ondaliksiz kisimdaki basamak sayisi
	dec_digit = 0;
	while (temp_buffer[dec_digit] != '.')	dec_digit++;

	// ondalik kismi
	digit = (strlen(temp_buffer)) - dec_digit;

	// tam kismi tam sayiya cevirme
	dec = atoi((char*)temp_buffer + dec_digit);

	float longitude = (num / 100.0) + (dec / pow(10, (digit + 2)));  // 1234.56789 = 12.3456789 (12 derece 3456789 dakika)

	gga->location.longitude = longitude;
	gga->location.EW = gga_buffer[index++];


	// skip data fix
	index ++;   // ',' after EW
	while(gga_buffer[index] != ',')		index++;


	/***** number of satellistes *****/
	index++;
	i = 0;

	memset(temp_buffer, '\0', sizeof(temp_buffer));
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;
	}

	gga->numofsat = atoi(temp_buffer);



	/***** hdop value *****/
	// skip
	index ++;   // ',' after EW
	while(gga_buffer[index] != ',')		index++;



	/***** altitude1 *****/
	/* denizden yukseklik */
	index++;
	i = 0;

	memset(temp_buffer, '\0', sizeof(temp_buffer));
	while (gga_buffer[index] != ','){
		temp_buffer[i] = gga_buffer[index];
		i++;
		index++;
	}

	num = (atoi(temp_buffer));
	dec_digit = 0;
	while (temp_buffer[dec_digit] != '.') 	dec_digit++;

	digit = (strlen(temp_buffer)) - dec_digit;
	dec = atoi ((char *)temp_buffer + dec_digit);

	float altitude = (num) + (dec / pow(10, (digit)));
	gga->alt.altitude = altitude;

	index++;
	gga->alt.unit = gga_buffer[index];

	return 1;	// success :)
}



uint8_t decode_rmc (char *rmc_buffer, t_gps_rmc *rmc){
	int i, index = 0;
	char temp_buffer[12];	// max 12 index

	/*
	 * RMC Packet Structure
	 * $GPRMC, UTC time, state, latitude, NS, longitude, EW, speed (knot), course, date, magnetic declination, unit, mode, *CRC
	 *
	 * */

	while(rmc_buffer[index] != ',')		index++;	// skip first
	index++;
	while(rmc_buffer[index] != ',')		index++;	// skip utc time
	index++;

	if(rmc_buffer[index] == 'A')	rmc->is_valid = 1;
	else	rmc->is_valid = 0;

	if(rmc->is_valid == 1){
		index++;
		for(i = 0; i < 4; i++){
			while(rmc_buffer[index] != ',')		index++;
			index++;
		}


		/***** speed *****/
		index++;
		i = 0;

		memset(temp_buffer, '\0', sizeof(temp_buffer));
		while (rmc_buffer[index] != ','){
			temp_buffer[i] = rmc_buffer[index];
			i++;
			index++;
		}

		if(strlen(temp_buffer) > 0){

			int16_t num = (atoi(temp_buffer));
			int dec_digit = 0;
			while (temp_buffer[dec_digit] != '.')	dec_digit++;

			int digit = (strlen(temp_buffer)) - dec_digit;
			int dec = atoi((char*)temp_buffer + dec_digit);

			float speed_knot = (num / 100.0) + (dec / pow(10, (digit + 2)));
			float speed = speed_knot * 1.852;	// converting knot to km/h

			rmc->speed = speed;		// km/h

		}
		else	rmc->speed = 0.0;


		/***** course *****/
		index++;
		i = 0;

		memset(temp_buffer, '\0', sizeof(temp_buffer));
		while (rmc_buffer[index] != ','){
			temp_buffer[i] = rmc_buffer[index];
			i++;
			index++;
		}

		if(strlen(temp_buffer) > 0){

			int16_t num = (atoi(temp_buffer));
			int dec_digit = 0;
			while (temp_buffer[dec_digit] != '.')	dec_digit++;

			int digit = (strlen(temp_buffer)) - dec_digit;
			int dec = atoi((char*)temp_buffer + dec_digit);

			float course = (num / 100.0) + (dec / pow(10, (digit + 2)));
			rmc->course = course;	// kuzeye gore derece cinsinden

		}
		else	rmc->course = 0.0;


		/***** date *****/
		index++;
		i = 0;

		memset(temp_buffer, '\0', sizeof(temp_buffer));
		while (rmc_buffer[index] != ','){
			temp_buffer[i] = rmc_buffer[index];
			i++;
			index++;
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




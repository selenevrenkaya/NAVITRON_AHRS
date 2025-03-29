/*
 * neo_m8n.h
 *
 *  Created on: Mar 27, 2025
 *      Author: Selen Evrenkaya
 */

#ifndef SENSOR_NEO_M8N_H_
#define SENSOR_NEO_M8N_H_

#include "circ_buffer.h"


#define GMT 	+300	/* Turkiye */



typedef struct date{
	int day;
	int month;
	int year;

}t_date;


typedef struct location{
	float latitude;
	char NS;			// North, South
	float longitude;
	char EW;			// East, Weast

}t_location;


typedef struct time{
	int hour;
	int min;
	int sec;

}t_time;


typedef struct altitude{	//yukseklik
	float altitude;
	char unit;			// birim (metre)

}t_altitude;


typedef struct gps_gga{
	t_location location;
	t_time time;
	int is_fix_valid;
	t_altitude alt;
	int numofsat;
	char *received_data;

}t_gps_gga;


typedef struct gps_rmc{
	t_date date;
	float speed;
	float course;
	int is_valid;

}t_gps_rmc;



typedef struct gps{
	t_circ_buffer *cbuffer;
	uint8_t header_l;
	uint8_t header_h;
	uint32_t package_size;
	uint8_t command;
	uint32_t CRC_value;
	uint8_t package_count;
	bool transmit_check;	/* TODO: can be different returns */
	bool receive_check;

	t_gps_gga *gga;
	t_gps_rmc *rmc;

}t_gps;





/* function definitions */

uint8_t check_sentence(t_gps *gps, char *string);
uint8_t parse_gps_data(t_gps *gps, char *string, char *destination);

uint8_t decode_gga(char *gga_buffer, t_gps_gga *gga);
uint8_t decode_rmc (char *rmc_buffer, t_gps_rmc *rmc);



#endif /* SENSOR_NEO_M8N_H_ */

/*
 * neo_m8n.h
 *
 *  Created on: Mar 27, 2025
 *      Author: selenevrenkaya
 */

#ifndef GPS_NEO_M8N_H_
#define GPS_NEO_M8N_H_

#include "circ_buffer.h"
#include "stm32f4xx_hal.h"

#include "ahrs.h"

#define GMT 			(+300)	/* Turkiye */
#define packet_size		(100)


typedef struct date{
	int day;
	int month;
	int year;

}t_date;


typedef struct location{
	double latitude;							// degree
	char NS;			/* North, South */
	double longitude;						// degree
	char EW;			/* East, Weast */

}t_location;


typedef struct time{
	int hour;
	int min;
	int sec;

}t_time;


typedef struct altitude{	/* yukseklik */
	float altitude;								// degree
	char unit;				/* birim (metre) */

}t_altitude;


typedef struct gps_gga{
	t_location location;
	t_time time;
	int is_fix_valid;
	t_altitude alt;
	int numofsat;
	char received_data[packet_size];

}t_gps_gga;


typedef struct gps_rmc{
	t_date date;
	float speed;		/* speed over ground (m/s) */
	float heading;		/* degree */
	float vN;			/* kuzey yonu hiz bileseni (m/s) */
	float vE;			/* dogu yonu hiz bileseni (m/s) */
	float vD;			/* asagi yonu hiz bileseni (m/s) --> genelde sifir */
	int is_valid;
	char received_data[packet_size];

}t_gps_rmc;


typedef struct gps_vgt{
	float speed;

}t_gps_vgt;



typedef struct gps{
	t_circ_buffer *cbuffer;
	//uint8_t header_l;
	//uint8_t header_h;
	//uint32_t package_size;
	//uint8_t command;
	//uint32_t CRC_value;
	//uint8_t package_count;
	bool transmit_check;	/* TODO: can be different returns */
	bool receive_check;

	t_gps_gga gga;
	t_gps_rmc rmc;

}t_gps;





/* function definitions */

uint8_t check_sentence(t_gps *gps, char *string);
uint8_t parse_gps_data(t_gps *gps, char string, unsigned char *destination);

uint8_t decode_gga(unsigned char *gga_buffer, t_gps_gga *gga);
uint8_t decode_rmc (unsigned char *rmc_buffer, t_gps_rmc *rmc);
uint8_t decode_vgt (unsigned char *vgt_buffer, t_gps_vgt *vgt);

void set_ahrs_data(t_gps *gps, tAHRS* ahrs);


void uart_isr(UART_HandleTypeDef *huart);



#endif /* GPS_NEO_M8N_H_ */

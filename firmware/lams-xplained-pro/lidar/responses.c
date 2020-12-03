#include "lidar.h"
#include "common.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

/* See lidar.h for declarations */
extern resp_desc_s      resp_desc;
extern write_data_s		sd_scan_data[MAX_SCANS];
extern conf_data_t		conf_data;
extern uint32_t			scan_count;
extern uint32_t			invalid_exp_scans;
extern uint8_t			lidar_request;
extern uint32_t			byte_count;
extern uint16_t			buffer_length;
extern volatile char	DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

legacy_cabin_data_s  legacy_cabins[LEGACY_CABIN_COUNT] = {0};
ultra_cabin_data_s   ultra_cabins[EXT_CABIN_COUNT] = {0};
dense_cabin_data_s   dense_cabins[DENSE_CABIN_COUNT] = {0};

static char info[512];

/** 
  * Process "EXPRESS_SCAN" Legacy Version request's response
  * Returns 1 if valid scan, 0 otherwise
  *
  * Response descriptor: A5 5A 54 00 00 40 82
  *
  *		Byte Offset:	+0		sync1 (0xA), checksum[3:0]
  *		Order 8..0		+1		sync2 (0x5), checksum[7:4]
  *						+2		start_angle_q6[7:0]
  *						+3		start, start_angle_q6[14:8]
  *
  *			   Legacy:	+4		cabin[0]
  *						+9		cabin[1]
  *								...
  *						+79		cabin[15]
  *
  *			 Extended:	+4		ultra_cabin[0]
  *						+8		ultra_cabin[1]
  *								...
  *						+128	ultra_cabin[31]
  *
  *				Dense:	+4		cabin[0]
  *						+9		cabin[1]
  *								...
  *						+79		cabin[40]
  *
  *		Bytes Offset:	
  *			   Legacy:	+0		distance1[6:0], angle_val1[4] (sign)
  *						+1		distance1[14:7]
  *						+2		distance2[6:0], angle_val2[4] (sign)
  *						+3		distance2[14:7]
  *						+4		angle_val2[3:0], angle_val1[3:0]
  *
  *			 Extended:	+0		predict2[9:2]		
  *						+1		predict2[1:0], predict1[9:4]
  *						+2		predict1[3:0], major[11:8]
  *						+3		major[7:0]
  *	
  *				Dense:	+0		distance[15:0]
  */
uint8_t LIDAR_RES_express_scan(void) 
{ 
	uint8_t calc_checksum;
	uint8_t PAYLOAD_SIZE=(resp_desc.response_info & 0x3FFFFFFF), CABIN_START=4;
	uint16_t i, pos;
	
	uint8_t checksum = ((uint8_t)DATA_RESPONSE[1] << 4) | ((uint8_t)DATA_RESPONSE[0] & 0x0F);
	uint16_t start_angle = (uint8_t)DATA_RESPONSE[2] | (((uint8_t)DATA_RESPONSE[3] & 0x7F) << 8);
	uint8_t S_flag = (uint8_t)DATA_RESPONSE[3] >> 7;

    /* Decrement byte count so next scan rewrites same DATA_RESPONSE bytes */
	byte_count -= PAYLOAD_SIZE;
	
	/* Check if data valid */
	calc_checksum = 0;
	for (i=2; i<PAYLOAD_SIZE; i++)
		calc_checksum ^= DATA_RESPONSE[i];
		
	if (checksum != calc_checksum) {
		invalid_exp_scans++;
		return 0;
	}

    switch (resp_desc.data_type) {
        
        /* Legacy Version */
        case EXPRESS_SCAN_LEGACY_VER:
            for (i = 0; i < LEGACY_CABIN_COUNT; i++) {
                pos = CABIN_START + (LEGACY_CABIN_BYTE_COUNT * i);
				legacy_cabins[i].start_angle = start_angle;
                legacy_cabins[i].distance1 =    ( (uint8_t)DATA_RESPONSE[pos+0] >> 1 ) |
												( (uint8_t)DATA_RESPONSE[pos+1] << 7 );
				legacy_cabins[i].distance2 =	( (uint8_t)DATA_RESPONSE[pos+2] >> 1 ) |
												( (uint8_t)DATA_RESPONSE[pos+3] << 7 );
                legacy_cabins[i].angle_value1 = (((uint8_t)DATA_RESPONSE[pos+0] & 0x1 ) << 4) |
												( (uint8_t)DATA_RESPONSE[pos+4] & 0x0F );
				legacy_cabins[i].angle_value2 = (((uint8_t)DATA_RESPONSE[pos+2] & 0x1 ) << 4) |
												( (uint8_t)DATA_RESPONSE[pos+4] >> 4 );
            }
            break;
        
        /* Extended Version */
		case EXPRESS_SCAN_EXTENDED_VER:
			for (i = 0; i < EXT_CABIN_COUNT; i++) {
				pos = CABIN_START + (EXT_CABIN_BYTE_COUNT * i);
				ultra_cabins[i].S = S_flag;
				ultra_cabins[i].start_angle = start_angle;
				ultra_cabins[i].predict2 =  ( (uint8_t)DATA_RESPONSE[pos+0] << 2 ) |
										    ( (uint8_t)DATA_RESPONSE[pos+1] >> 6 );
				ultra_cabins[i].predict1 =  (((uint8_t)DATA_RESPONSE[pos+1] & 0x3F ) << 4 ) |
										    ( (uint8_t)DATA_RESPONSE[pos+2] >> 4 );
				ultra_cabins[i].major =     ( (uint8_t)DATA_RESPONSE[pos+2] << 8 ) |
											( (uint8_t)DATA_RESPONSE[pos+3] );
			}
			break;
		
		/* Dense Version */
		case EXPRESS_SCAN_DENSE_VER:
			for (i = 0; i < DENSE_CABIN_COUNT; i++) {
				pos = CABIN_START + (DENSE_CABIN_BYTE_COUNT * i);
				dense_cabins[i].S = S_flag;
				dense_cabins[i].distance =  ( (uint8_t)DATA_RESPONSE[pos+0] << 8) |
											( (uint8_t)DATA_RESPONSE[pos+1]);
				dense_cabins[i].start_angle = start_angle;
			}
			break;
    }
	return 1;
}

/**	
  * Process "GET_INFO" request's response
  *		Byte Offset:	+0		model
  *		Order 8..0		+1		firmware_minor
  *						+2		firware_major
  *						+3		hardware
  *						+4		serial_number[0]
  *								 ...
  *						+19 	serial_number[15]		
  * When converting serial_number to text from hex, the least significant byte 
  *	prints first.
  */
char* LIDAR_RES_get_info(void) 
{	
	uint8_t model_id 		 = DATA_RESPONSE[0];
	uint8_t firmware_minor	 = DATA_RESPONSE[1];
	uint8_t firmware_major	 = DATA_RESPONSE[2];
	uint8_t hardware_version = DATA_RESPONSE[3];
	char serial_number[16]	 = {0};
	char tmp_info[512] = "";
	
	/** Get hexadecimal string output */
	int i;
	for (i=0; i<15; i++) {
		sprintf(&serial_number[i], "%02X", DATA_RESPONSE[i+4]);
	}
	
	/* Format string to print as header in .lam file */
	sprintf(tmp_info, "# RPLiDAR Model ID: %u\r\n", model_id);
	strcpy(info, tmp_info);
	sprintf(tmp_info, "# RPLiDAR Firmware Version: %u.%u\r\n", firmware_major, firmware_minor);
	strcat(info, tmp_info);
	sprintf(tmp_info, "# Hardware Version: %u\r\n", hardware_version);
	strcat(info, tmp_info);
	sprintf(tmp_info, "# Serial Number: 0x%s\r\n", serial_number);
	strcat(info, tmp_info);
	return info;
}

/**	
  * Process "GET_HEALTH" request's response
  *		Byte Offset:	+0		status
  *		Order 8..0		+1		error_code[7:0]
  *						+2		error_code[15:8]	
  *	@return uint16_t : error_code
  */
uint16_t LIDAR_RES_get_health(void) 
{
	char* status;
	uint16_t error_code;  
	
	switch(DATA_RESPONSE[0]) {
		case 0: status = "GOOD"; break;
		case 1: status = "WARNING"; break;
		case 2: status = "ERROR"; break;
		default: status = "UNKNOWN"; break;
	}
	
	error_code = DATA_RESPONSE[0] + ((unsigned)DATA_RESPONSE[1] << 8);
	
    if (LAMS_DEBUG) {
		if (error_code == 0)
			printf(" : LiDAR Health is %s!\r\n", status);
		else 
			printf(" : LiDAR Health is %s!\r\n : Error code: %u\r\n", 
				    status, error_code);
    }
	
	return error_code;
}

/**	
  * Process "GET_SAMPLERATE" request's response
  *		Byte Offset:	+0		Tstandard[7:0]
  *		Order 8..0		+1		Tstandard[15:8]
  *						+2		Texpress[7:0]
  *						+3		Texpress[15:8]	
  */
void LIDAR_RES_get_samplerate(void)
{
    if (LAMS_DEBUG) {
	    printf(" : Standard Scan Samplerate: %u\r\n", 
			    DATA_RESPONSE[0] + ((unsigned)DATA_RESPONSE[1] << 8));
		printf(" : Express Scan Samplerate: %u\r\n", 
			    DATA_RESPONSE[2] + ((unsigned)DATA_RESPONSE[3] << 8));
    }
}

/** 
  * Process "GET_LIDAR_CONF" request's response
  *		Byte Offset:	+0		configuration type
  *						+1		payload[0]
  *						+2		payload[1]
  *								...
  *						+(n+4)	payload[n]	
  */
void LIDAR_RES_get_lidar_conf(void)
{
	int i;
	uint32_t conf_type = (uint8_t)DATA_RESPONSE[0] | ((uint8_t)DATA_RESPONSE[1] << 8) |
						 ((uint8_t)DATA_RESPONSE[2] << 16) | ((uint8_t)DATA_RESPONSE[3] << 24);
	
	switch (conf_type) {
		case CONF_SCAN_MODE_COUNT:
			conf_data.resp1 = ( ((uint8_t)DATA_RESPONSE[4]) | 
								((uint8_t)DATA_RESPONSE[5] << 8) );
			if (LAMS_DEBUG)
				printf(" : %u scan modes supported\r\n", conf_data.resp1);
			return;
				
		case CONF_SCAN_MODE_US_PER_SAMPLE:
			conf_data.resp2 = ( ((uint8_t)DATA_RESPONSE[4])       |
								((uint8_t)DATA_RESPONSE[5] << 8 ) |
								((uint8_t)DATA_RESPONSE[6] << 16) |
								((uint8_t)DATA_RESPONSE[7] << 24) );
			conf_data.resp2 = conf_data.resp2 / (1 << 8);
			if (LAMS_DEBUG) 
				printf(" : Specified scan mode costs %"PRIu32" us per sample\r\n", conf_data.resp2);
			return;
		
		case CONF_SCAN_MODE_MAX_DISTANCE:
			conf_data.resp2 = ( ((uint8_t)DATA_RESPONSE[4])       |
								((uint8_t)DATA_RESPONSE[5] << 8 ) |
								((uint8_t)DATA_RESPONSE[6] << 16) |
								((uint8_t)DATA_RESPONSE[7] << 24) );
			conf_data.resp2 = conf_data.resp2 / (1 << 8);
			if (LAMS_DEBUG)
				printf(" : Specified scan mode has a max measuring distance of %"PRIu32" m\r\n", conf_data.resp2);
			return;
		
		case CONF_SCAN_MODE_ANS_TYPE:
			conf_data.resp0 = (uint8_t)DATA_RESPONSE[4];
			if (LAMS_DEBUG) {
				switch (conf_data.resp0) {
					case ANS_TYPE_STANDARD:
						printf(" : Specified scan mode returns data in rplidar_resp_measurement_node_t\r\n");
						break;
					case ANS_TYPE_EXPRESS:
						printf(" : Specified scan mode returns data in capsuled format\r\n");
						break;
					case ANS_TYPE_CONF:
						printf(" : Specified scan mode returns data in ultra capsuled format\r\n");
						break;
					default:
						printf(" : Specified scan mode returns data in unspecified format (0x%02X)\r\n", conf_data.resp0);
				}
			}
			return;
		
		case CONF_SCAN_MODE_TYPICAL:
			conf_data.resp1 = ( ((uint8_t)DATA_RESPONSE[4]) |
								((uint8_t)DATA_RESPONSE[5] << 8) );
			if (LAMS_DEBUG)
				printf(" : Typical scan mode id of LiDAR is %"PRIu16"\r\n", conf_data.resp1);
			conf_data.resp1 = conf_data.resp1;
			return;
		
		case CONF_SCAN_MODE_NAME:
			for (i=0; i<(resp_desc.response_info & 0x3FFFFFFF); i++)
				conf_data.resp3[i] = toupper(DATA_RESPONSE[i+4]);
			if (LAMS_DEBUG)
				printf(" : Specified scan mode name is %s\r\n", conf_data.resp3);
			return;
	}
}
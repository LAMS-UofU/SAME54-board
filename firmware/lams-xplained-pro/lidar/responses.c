#include "lidar.h"

/* See lidar.h for declarations */
extern response_descriptor resp_desc;
extern cabin_data cabins[MAX_SCANS];
extern scan_data scans[MAX_SCANS];
extern uint32_t scan_count;
extern uint32_t invalid_exp_scans;
extern uint8_t lidar_request;
extern volatile uint32_t byte_count;
extern uint16_t buffer_length;
extern uint8_t processing;
extern volatile char DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];
extern volatile uint16_t lidar_timer;
extern volatile uint8_t lidar_timing;

/** 
  * "STOP" request has no response.
  */
void LIDAR_RES_stop(void) 
{
	if (!lidar_timer) {
		lidar_timing = 0;
		processing = 0;
		if (DEBUG) 
			printf("LiDAR stopped\r\n");
	}
}
	
/** 
  * "RESET" request has no response. 
  *
  * Note: If no command sent after 2ms, sends back partial info resulting in:
  *           RP LIDAR System.\r\n
  *           Firmware Ver 1.27 - rc9, HW Ver 5\r\n
  *           Model: 28\r\n
  */
void LIDAR_RES_reset(void) 
{   
	if (!lidar_timer) {
		lidar_timing = 0;
		processing = 0;
		if (DEBUG) 
			printf("LiDAR reset\r\n");
			
	}
}
	
/** 
  * Process "SCAN" or "FORCE SCAN" request's response
  *
  * Response descriptor: A5 5A 05 00 00 40 81
  * 
  *		Byte Offset:	+0		quality, ~start, start
  *		Order 8..0		+1		angle[6:0], check
  *						+2		angle_q6[14:7]
  *						+3		distance_q2[7:0]
  *						+4		distance_q2[15:8]
  */
void LIDAR_RES_scan(void) 
{
	/* check[0] - start
	   check[1] - ~start
	   check[2] - check	 */
	uint8_t check = ((DATA_RESPONSE[0] & 0x3) << 1) | (DATA_RESPONSE[1] & 0x1);
	uint16_t angle = ((uint16_t)DATA_RESPONSE[1] >> 1) + 
									 ((uint16_t)DATA_RESPONSE[2] << 7);
	uint16_t distance = DATA_RESPONSE[3] + ((uint16_t)DATA_RESPONSE[4] << 8);
	
	/* Decrement byte_count so next scan rewrites same DATA_RESPONSE bytes */
	byte_count -= 5;
	
	/* Checking: check=1, ~start=0, start=1 */
	if (check == 0x5 || check == 0x6) {
		scans[scan_count].quality = DATA_RESPONSE[0] >> 2;
		scans[scan_count].angle = angle;
		scans[scan_count++].distance = distance;
	}
	
    if (DEBUG)
        if (scan_count % 16 == 0)
            printf("gathered %0"PRIu32"/%0d scans...\r\n", scan_count, MAX_SCANS);
	
	/* Prints invalid responses...not all responses are supposed to valid, so no
		 need to print these except for debugging */
	/*
    else {
	    printf("Invalid response: C=%u, !S=%u, S=%u\r\n",
		        (check >> 3), ((check >> 2) & 0x1), (check & 0x1));
	}
    */
}

/** 
  * Process "EXPRESS_SCAN" request's response
  *
  * Response descriptor: A5 5A 54 00 00 40 82
  *
  *		Byte Offset:	+0		sync1 (0xA), checksum[3:0]
  *		Order 8..0		+1		sync2 (0x5), checksum[7:4]
  *						+2		start_angle_q6[7:0]
  *						+3		start, start_angle_q6[14:8]
  *						+4		cabin[0]
  *						+9		cabin[1]
  *								...
  *						+79		cabin[15]
  *	Cabin Bytes Offset:	+0		distance1[6:0], angle_val1[4] (sign)
  *						+1		distance1[14:7]
  *						+2		distance2[6:0], angle_val2[4] (sign)
  *						+3		distance2[14:7]
  *						+4		angle_val2[3:0], angle_val1[3:0]
  */
void LIDAR_RES_express_scan(void) 
{ 
	uint8_t calc_checksum;
	uint8_t PAYLOAD_SIZE=84, CABIN_COUNT=16, CABIN_START=4, CABIN_BYTE_COUNT=5;
	uint16_t i, pos;
	
	uint8_t checksum = ((uint8_t)DATA_RESPONSE[1] << 4) | ((uint8_t)DATA_RESPONSE[0] & 0x0F);
	uint16_t start_angle = DATA_RESPONSE[2] | (((uint8_t)DATA_RESPONSE[3] & 0x7F) << 8);
	uint8_t S_flag = DATA_RESPONSE[3] >> 7;
	
    /* Decrement byte count so next scan rewrites same DATA_RESPONSE bytes */
	byte_count -= PAYLOAD_SIZE;
	
	/* Check if data valid */
	calc_checksum = 0;
	for (i=2; i<PAYLOAD_SIZE; i++)
		calc_checksum ^= DATA_RESPONSE[i];
		
	if (checksum != calc_checksum) {
		/*
        if (invalid_exp_scans % 100 == 0) {
			printf("Invalid checksum! Given: %u -- Calculated: %u\r\n", checksum, calc_checksum);
			printf("First 8: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", 
					DATA_RESPONSE[0], DATA_RESPONSE[1], DATA_RESPONSE[2], DATA_RESPONSE[3], 
					DATA_RESPONSE[4], DATA_RESPONSE[5], DATA_RESPONSE[6], DATA_RESPONSE[7]);
		}
        */
		invalid_exp_scans++;
		return;
	}
	
	for (i=0; i<CABIN_COUNT; i++) {
		pos = CABIN_START+(CABIN_BYTE_COUNT*i);
		cabins[scan_count].S = S_flag;
		cabins[scan_count].distance1 = ((uint8_t)DATA_RESPONSE[pos+0] >> 1) | 
											((uint8_t)DATA_RESPONSE[pos+1] << 7);
		cabins[scan_count].distance2 = (DATA_RESPONSE[pos+2] >> 1) |
											((uint8_t)DATA_RESPONSE[pos+3] << 7);
		cabins[scan_count].angle_value1 = (((uint8_t)DATA_RESPONSE[pos+0] & 0x1) << 4) |
											((uint8_t)DATA_RESPONSE[pos+4] >> 4);
		cabins[scan_count].angle_value2 = (((uint8_t)DATA_RESPONSE[pos+2] & 0x1) << 4) |
											(DATA_RESPONSE[pos+4] & 0x0F);
		cabins[scan_count++].start_angle = start_angle;
	}
	
    if (DEBUG)
	    printf("%0"PRIu32" invalid scans -- gathered %0"PRIu32"/%0d scans...\r\n", 
				invalid_exp_scans, scan_count, MAX_SCANS);
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
void LIDAR_RES_get_info(void) 
{	
	uint8_t model_id 		 = DATA_RESPONSE[0];
	uint8_t firmware_minor	 = DATA_RESPONSE[1];
	uint8_t firmware_major	 = DATA_RESPONSE[2];
	uint8_t hardware_version = DATA_RESPONSE[3];
	char serial_number[16]	 = {0};
	
	/** Get hexadecimal string output */
	int i, j=0;
	for (i=15; i>=0; i--) {
		sprintf(&serial_number[j++], "%02X", DATA_RESPONSE[i+4]);
	}
	
    if (DEBUG) {
        printf(" : RPLiDAR Model ID: %u\r\n", model_id);	
        printf(" : Firmware Version: %u.%u\r\n", firmware_major, firmware_minor);
        printf(" : Hardware Version: %u\r\n", hardware_version);
        printf(" : Serial Number: 0x%s\r\n", serial_number);
    }
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
	
    if (DEBUG) {
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
    if (DEBUG) {
	    printf(" : Standard Scan Samplerate: %u\r\n", 
			    DATA_RESPONSE[0] + ((unsigned)DATA_RESPONSE[1] << 8));
		printf(" : Express Scan Samplerate: %u\r\n", 
			    DATA_RESPONSE[2] + ((unsigned)DATA_RESPONSE[3] << 8));
    }
}

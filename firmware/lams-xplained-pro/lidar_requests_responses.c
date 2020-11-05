#include "lidar_requests_responses.h"

#define LIDAR_START_BYTE			0xA5
#define LIDAR_RES_BYTE				0x5A
#define LIDAR_REQ_STOP 				0x25
#define LIDAR_REQ_RESET 			0x40
#define LIDAR_REQ_SCAN				0x20
#define LIDAR_REQ_EXPRESS_SCAN 		0x82
#define LIDAR_REQ_FORCE_SCAN		0x21
#define LIDAR_REQ_GET_INFO			0x50
#define LIDAR_REQ_GET_HEALTH		0x52
#define LIDAR_REQ_GET_SAMPLERATE	0x59

/*  start1        - 1 byte (0xA5)
	start2        - 1 byte (0x5A)
	response_info - 32 bits ([32:30] send_mode, [29:0] data_length)
	data_type 	  - 1 byte	*/
struct response_descriptor {
	uint8_t start1;
	uint8_t start2;
	uint32_t response_info;
	uint8_t data_type;
} resp_desc;

/*	distance1    - 15 bits 
	distance2    - 15 bits
	angle_value1 - 5 bits
	angle_value2 - 5 bits */
struct cabin_data {
	uint8_t angle_value1;
	uint8_t angle_value2;
	uint16_t distance1;
	uint16_t distance2;
};

/* Buffer for printing */
static char print_buffer[MAX_PRINT_BUFFER_SIZE];

/* Preserved value of which request was given until new request is sent */
uint8_t lidar_request;

/* Number of bytes received for processing LiDAR responses */
static uint32_t byte_count;
static uint16_t buffer_length;

/* For "STOP" and "RESET" commands requiring waiting specific times before another
	 request is allowed */
volatile uint8_t lidar_timer;
volatile uint8_t lidar_timing;

/* Preserve response bytes */
volatile char DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

/**
	*	Resets response descriptor for new incoming responses.
	*	@return None
	*/
void LIDAR_reset_response_descriptor(void)
{
	resp_desc.start1 = 0;
	resp_desc.start2 = 0;
	resp_desc.response_info = 0;
	resp_desc.data_type = 0;
}

/**	
	* Request LiDAR to exit scanning state and enter idle state. No response!
	* Need to wait at least 1 millisecond before sending another request.
	*		Byte Order:	+0		Request Start Bit (0xA5)
	*					+1		LIDAR_REQ_STOP (0x25)
	* @see SysTick_Handler
	* @return None
	*/
void LIDAR_REQ_stop(void)
{
	lidar_request = LIDAR_REQ_STOP;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_REQ_STOP);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
	
	/** Wait 1 ms. Decrementing in @see SysTick_Handler every 1 ms */
	lidar_timer = 1;
	lidar_timing = 1;
}

/** 
	* Request LiDAR to reset (reboot) itself by sending this request. State 
	* similar to as it LiDAR had just powered up. No response! Need to wait at
	* least 2 milliseconds before sending another request.
	*		Byte Order:	+0		Request Start Bit (0xA5)
	*					+1		LIDAR_REQ_RESET (0x40)
	* @see SysTick_Handler
	* @return None
	*/
void LIDAR_REQ_reset(void)
{
	lidar_request = LIDAR_REQ_RESET;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_REQ_RESET);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
	
	/** Wait 2 ms. Decrementing in @see SysTick_Handler every 1 ms */
	lidar_timer = 2;
	lidar_timing = 1;
}


/** 
	* Request LiDAR to enter scanning state and start receiving measurements.
	* Note: RPLIDAR A2 and other device models support 4khz sampling rate will 
	* lower the sampling rate when processing this request. Please use EXPRESS_SCAN 
	* for the best performance.
	*		Byte Order:	+0		Request Start Bit (0xA5)
	*					+1		LIDAR_REQ_SCAN (0x20)
	*	@return None
	*/
void LIDAR_REQ_scan(void)
{
	lidar_request = LIDAR_REQ_SCAN;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_REQ_SCAN);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}



/** 
	* Request LiDAR to enter scanning state and start receiving measurements.
	* Different from SCAN request as this will make LiDAR work at the sampling 
	* rate as high as it can be.  For RPLIDAR A2 and the device models support 
	* 4khz sampling rate, the host system is required to send this request to 
	* let the RPLIDAR work at 4khz sampling rate and output measurement sample 
	* data accordingly; for the device models with the 2khz sample rate (such as 
	* RPLIDAR A1 series), this request will implement the same sampling rate as 
	* the scan(SCAN) request.
	*		Byte Order:	+0		Request Start Bit (0xA5)
	*					+1		LIDAR_REQ_EXPRESS_SCAN (0x82)	
	*					+2		Payload Size (0x5)
	*					+3		working_mode (set to 0)
	*					+4		reserved_field (set to 0)
	*					+5		reserved_field (set to 0)
	*					+6		reserved_field (set to 0)
	*					+7		reserved_field (set to 0)	
	*					+8		checksum = 0 XOR 0xA5 XOR CmdType XOR PayloadSize XOR 
	* 							Payload[0] XOR ... XOR Payload[n]
	* @return None
	*/
void LIDAR_REQ_express_scan(void)
{
	char working_mode = 0;
	char reserved_fields = 0;
	char payload_size = 0x5;
	char checksum = 0 ^ LIDAR_START_BYTE ^ LIDAR_REQ_EXPRESS_SCAN ^ payload_size;
	
	lidar_request = LIDAR_REQ_EXPRESS_SCAN;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
						"%c%c%c", LIDAR_START_BYTE, LIDAR_REQ_EXPRESS_SCAN, payload_size); 
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
	
	/** @bug STM32 won't send these fields since they are set to 0 */
	LIDAR_USART_send((uint8_t *)&working_mode, 1);
	LIDAR_USART_send((uint8_t *)&reserved_fields, 1);
	LIDAR_USART_send((uint8_t *)&reserved_fields, 1);
	LIDAR_USART_send((uint8_t *)&reserved_fields, 1);
	LIDAR_USART_send((uint8_t *)&reserved_fields, 1);
	LIDAR_USART_send((uint8_t *)&checksum, 1);
}



/** 
	* Request LiDAR to forcefully start measurement sampling and send out the 
	* results immediately. Useful for device debugging.	
	*		Byte Order:	+0		Request Start Bit (0xA5)
	*					+1		LIDAR_REQ_FORCE_SCAN (0x21)		
	* @return None
	*/
void LIDAR_REQ_force_scan(void)
{
	lidar_request = LIDAR_REQ_FORCE_SCAN;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_REQ_FORCE_SCAN);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}



/** 
	* Request LiDAR to send device information.
	*		Byte Order:	+0		Request Start Bit (0xA5)
	*					+1		LIDAR_REQ_GET_INFO (0x50)
	* @return None
	*/
void LIDAR_REQ_get_info(void)
{
	lidar_request = LIDAR_REQ_GET_INFO;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_REQ_GET_INFO);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}



/** 
	* Request LiDAR health state. If it has entered the Protection Stop state 
	* caused by hardware failure, relate code of the failure will be sent out.
	*		Byte Order:	+0		Request Start Bit (0xA5)
	*					+1		LIDAR_REQ_GET_HEALTH (0x52)
	* @return None
	*/
void LIDAR_REQ_get_health(void)
{
	lidar_request = LIDAR_REQ_GET_HEALTH;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_REQ_GET_HEALTH);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}



/** 
	* Request LiDAR to send single measurement duration for scan mode and express 
	*	scan mode. 
	*		Byte Order:	+0		Request Start Bit (0xA5)
	*					+1		LIDAR_REQ_GET_SAMPLERATE (0x59)
	* @return None
	*/
void LIDAR_REQ_get_samplerate(void)
{
	lidar_request = LIDAR_REQ_GET_SAMPLERATE;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_REQ_GET_SAMPLERATE);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}



/** 
	* "STOP" request has no response. Instead using @see SysTick_Handler to wait 
	* 1 ms with precision. 
	* @return None
	*/
void LIDAR_RES_stop(void) 
{
	if (!lidar_timer) {
		byte_count = 0;
		lidar_timing = 0;
		printf("LiDAR stopped\r\nCMD?");
	}
}
	
/** 
	* "RESET" request has no response. Instead using @see SysTick_Handler to wait 
	* 2 ms with precision. 
	* @return None
	*/
void LIDAR_RES_reset(void) 
{
	if (!lidar_timer) {
		byte_count = 0;
		lidar_timing = 0;
		printf("LiDAR reset\r\nCMD?");
	}
}
	


/** 
	* Process "SCAN" or "FORCE SCAN" request's response
	*		Byte Offset:	+0		quality, ~start, start
	*		Order 8..0		+1		angle[6:0], check
	*						+2		angle_q6[14:7]
	*						+3		distance_q2[7:0]
	*						+4		distance_q2[15:8]
	* @return None
	*/
void LIDAR_RES_scan(void) 
{
	LIDAR_reset_print_buffer();
	
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
		snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE,
							"{\"Q\":%u,\"A\":%u,\"D\":%u}\r\n",
							DATA_RESPONSE[0] >> 2, angle, distance);
		printf(print_buffer);
	}
	
	/* Prints invalid responses...not all responses are supposed to valid, so no
		 need to print these except for debugging
	else {
		snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Invalid response: C=%u, !S=%u, S=%u\r\n",
							(check >> 3), ((check >> 2) & 0x1), (check & 0x1));
		USB_send(print_buffer);
	} */
}



/** 
	* Process "EXPRESS_SCAN" request's response
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
	* @return None
	*/
void LIDAR_RES_express_scan(void) 
{ 
	uint8_t calc_checksum;
	uint8_t PAYLOAD_SIZE=80, CABIN_COUNT=16, CABIN_START=4, CABIN_BYTE_COUNT=5;
	uint16_t i, pos;
	
	uint8_t checksum = (DATA_RESPONSE[0] & 0x0F) | 
											(((uint8_t)DATA_RESPONSE[1] & 0x0F) << 4);
	uint16_t start_angle = DATA_RESPONSE[2] | 
											(((uint8_t)DATA_RESPONSE[3] & 0x7F) << 8);
	struct cabin_data cabins[CABIN_COUNT];
	
	/* Decrement byte count so next scan rewrites same DATA_RESPONSE bytes */
	byte_count -= PAYLOAD_SIZE;
	
	/* Check if in sync */
	if ((DATA_RESPONSE[0] >> 4) != 0xA)
		return;
	if ((DATA_RESPONSE[1] >> 4) != 0x5)
		return;
	
	/* Check if data valid */
	calc_checksum = 0 ^ LIDAR_START_BYTE ^ LIDAR_RES_BYTE ^ PAYLOAD_SIZE;
	for (i=0; i<PAYLOAD_SIZE; i++)
		calc_checksum = calc_checksum ^ DATA_RESPONSE[i];
	if (checksum != calc_checksum) {
		snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							"Invalid checksum!\r\nGiven: %u\r\nCalculated: %u",
							checksum, calc_checksum);
		printf(print_buffer);
		return;
	}
	
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						"{\"A\":%u,", start_angle);
	printf(print_buffer);
	
	for (i=0; i<CABIN_COUNT; i++) {
		pos = CABIN_START+(CABIN_BYTE_COUNT*i);
		cabins[i].distance1 = ((uint8_t)DATA_RESPONSE[pos+0] >> 1) | 
											((uint8_t)DATA_RESPONSE[pos+1] << 7);
		cabins[i].distance2 = (DATA_RESPONSE[pos+2] >> 1) |
											((uint8_t)DATA_RESPONSE[pos+3] << 7);
		cabins[i].angle_value1 = (((uint8_t)DATA_RESPONSE[pos+0] & 0x1) << 4) |
											((uint8_t)DATA_RESPONSE[pos+4] >> 4);
		cabins[i].angle_value2 = (((uint8_t)DATA_RESPONSE[pos+2] & 0x1) << 4) |
											(DATA_RESPONSE[pos+4] & 0x0F);
		snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							"\t{\"C[%u]\":{\"A1\":%u,\"A2\":%u,\"D1\":%u,\"D2\":%u}\r\n",
							i, cabins[i].angle_value1, cabins[i].angle_value2,
							cabins[i].distance1, cabins[i].distance2);
		printf(print_buffer);
	}	
	
	printf("}\r\n");
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
	* @return None
	*/
void LIDAR_RES_get_info(void) 
{
	printf("DONE\r\n");
	
	uint8_t model_id 							 = DATA_RESPONSE[0];
	uint8_t firmware_version[2] 	 = { DATA_RESPONSE[1], DATA_RESPONSE[2] };
	uint8_t hardware_version 			 = DATA_RESPONSE[3];
	char serial_number[16] = {0};
	
	/** Get hexadecimal string output */
	int i, j=0;
	for (i=15; i>=0; i--) {
		sprintf(&serial_number[j++], "%02X", DATA_RESPONSE[i+4]);
	}
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : RPLiDAR Model ID: %u\r\n", model_id);
	printf(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : Firmware Version: %u.%u\r\n", 
						firmware_version[0], firmware_version[1]);
	printf(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : Hardware Version: %u\r\n", hardware_version);
	printf(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : Serial Number: 0x%s\r\n", serial_number);
	printf(print_buffer);
}



/**	
	* Process "GET_HEALTH" request's response
	*		Byte Offset:	+0		status
	*		Order 8..0		+1		error_code[7:0]
	*						+2		error_code[15:8]	
	* @return None
	*/
void LIDAR_RES_get_health(void) 
{
	printf("DONE\r\n");
	
	char* status;
	uint16_t error_code;  
	
	switch(DATA_RESPONSE[0]) {
		case 0: status = "GOOD"; break;
		case 1: status = "WARNING"; break;
		case 2: status = "ERROR"; break;
		default: status = "UNKNOWN"; break;
	}
	
	error_code = DATA_RESPONSE[0] + ((unsigned)DATA_RESPONSE[1] << 8);
	
	LIDAR_reset_print_buffer();
	
	if (error_code == 0)
		snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							" : LiDAR Health is %s!\r\n", status);
	else
		snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							" : LiDAR Health is %s!\r\n : Error code: %u\r\n", 
							status, error_code);
	
	printf(print_buffer);
}


/**	
	* Process "GET_SAMPLERATE" request's response
	*		Byte Offset:	+0		Tstandard[7:0]
	*		Order 8..0		+1		Tstandard[15:8]
	*						+2		Texpress[7:0]
	*						+3		Texpress[15:8]	
	* @return None
	*/
void LIDAR_RES_get_samplerate(void)
{
	printf("DONE\r\n");
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE, 
						" : Standard Scan Samplerate: %u\r\n", 
						DATA_RESPONSE[0] + ((unsigned)DATA_RESPONSE[1] << 8));
	printf(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf(	print_buffer, MAX_PRINT_BUFFER_SIZE, 
						" : Express Scan Samplerate: %u\r\n", 
						DATA_RESPONSE[2] + ((unsigned)DATA_RESPONSE[3] << 8));
	printf(print_buffer);
}
#include "lidar.h"

static void LIDAR_reset_print_buffer(void);

/* See lidar.h for declarations */
extern response_descriptor resp_desc;
extern cabin_data cabins;
extern scan_data scans;
extern uint32_t scan_count;
extern uint32_t invalid_exp_scans;
extern uint8_t lidar_request;
extern volatile uint32_t byte_count;
extern uint16_t buffer_length;
extern uint8_t processing;

/* Buffer for sending data to lidar */
static char print_buffer[MAX_PRINT_BUFFER_SIZE];

/* For "STOP" and "RESET" commands requiring waiting specific times before another
   request is allowed */
volatile uint16_t lidar_timer;
volatile uint8_t lidar_timing;

volatile uint32_t start_time;

/**	
  * Request LiDAR to exit scanning state and enter idle state. No response!
  * Need to wait at least 1 millisecond before sending another request.
  *		Byte Order:	+0		Request Start Bit (0xA5)
  *					+1		LIDAR_REQ_STOP (0x25)
  */
void LIDAR_REQ_stop(void)
{
	if (DEBUG)
		printf("\r\nRequesting LiDAR stop\r\n");
	
	lidar_request = LIDAR_STOP;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_STOP);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
	
    /* Wait 1 ms */
	if (SYSTICK_EN) {
		lidar_timer = 1;
		lidar_timing = 1;	
	} else {
		delay_ms(1);
	}
}

/** 
  * Request LiDAR to reset (reboot) itself by sending this request. State 
  * similar to as it LiDAR had just powered up. No response! Need to wait at
  * least 2 milliseconds before sending another request.
  *		Byte Order:	+0		Request Start Bit (0xA5)
  *					+1		LIDAR_REQ_RESET (0x40)
  */
void LIDAR_REQ_reset(void)
{
	if (DEBUG)
		printf("\r\nRequesting LiDAR reset\r\n");
	
	lidar_request = LIDAR_RESET;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_RESET);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
	
	/* Wait 2 ms -- taking 0.7 seconds to reboot for some reason, giving full second */
	if (SYSTICK_EN) {
		lidar_timer = 1000;
		lidar_timing = 1;
	} else {
		delay_ms(1000);
	}
}

/** 
  * Request LiDAR to enter scanning state and start receiving measurements.
  * Note: RPLIDAR A2 and other device models support 4khz sampling rate will 
  * lower the sampling rate when processing this request. Please use EXPRESS_SCAN 
  * for the best performance.
  *		Byte Order:	+0		Request Start Bit (0xA5)
  *					+1		LIDAR_REQ_SCAN (0x20)
  */
void LIDAR_REQ_scan(void)
{
	if (DEBUG)
		printf("\r\nRequesting LiDAR start scan\r\n");
	
	start_time = systick_count;
	
	lidar_request = LIDAR_SCAN;
	byte_count = 0;
	scan_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
							 "%c%c", LIDAR_START_BYTE, LIDAR_SCAN);
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
  * 							       Payload[0] XOR ... XOR Payload[n]
  */
void LIDAR_REQ_express_scan(void)
{	
	if (DEBUG)
		printf("\r\nRequesting LiDAR start express scan\r\n");
	
	start_time = systick_count;
	
	char working_mode = 0;
	char reserved_fields = 0;
	char payload_size = 0x5;
	char checksum = 0 ^ LIDAR_START_BYTE ^ LIDAR_EXPRESS_SCAN ^ payload_size;
	
	lidar_request = LIDAR_EXPRESS_SCAN;
	invalid_exp_scans = 0;
    scan_count = 0;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
						"%c%c%c", LIDAR_START_BYTE, LIDAR_EXPRESS_SCAN, payload_size); 
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
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
  */
void LIDAR_REQ_force_scan(void)
{
	if (DEBUG)
		printf("\r\nRequesting LiDAR start force scan\r\n");
	
	start_time = systick_count;
	
	lidar_request = LIDAR_FORCE_SCAN;
	byte_count = 0;
	scan_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_FORCE_SCAN);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}

/** 
  * Request LiDAR to send device information.
  *		Byte Order:	+0		Request Start Bit (0xA5)
  *					+1		LIDAR_REQ_GET_INFO (0x50)
  */
void LIDAR_REQ_get_info(void)
{
	if (DEBUG)
		printf("\r\nRequesting LiDAR info\r\n");
		
	start_time = systick_count;
	
	lidar_request = LIDAR_GET_INFO;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_GET_INFO);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}

/** 
  * Request LiDAR health state. If it has entered the Protection Stop state 
  * caused by hardware failure, relate code of the failure will be sent out.
  *		Byte Order:	+0		Request Start Bit (0xA5)
  *					+1		LIDAR_REQ_GET_HEALTH (0x52)
  */
void LIDAR_REQ_get_health(void)
{
	if (DEBUG)
		printf("\r\nRequesting LiDAR health\r\n");
		
	start_time = systick_count;
	
	lidar_request = LIDAR_GET_HEALTH;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_GET_HEALTH);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}

/** 
  * Request LiDAR to send single measurement duration for scan mode and express 
  *	scan mode. 
  *		Byte Order:	+0		Request Start Bit (0xA5)
  *					+1		LIDAR_REQ_GET_SAMPLERATE (0x59)
  */
void LIDAR_REQ_get_samplerate(void)
{
	if (DEBUG)
		printf("\r\nRequesting LiDAR samplerates\r\n");
		
	start_time = systick_count;
	
	lidar_request = LIDAR_GET_SAMPLERATE;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
					 "%c%c", LIDAR_START_BYTE, LIDAR_GET_SAMPLERATE);
	LIDAR_USART_send((uint8_t *)print_buffer, buffer_length);
}


/** 
  * Resets local print buffer.
  */
void LIDAR_reset_print_buffer(void)
{
	int i;
	for (i=0; i<MAX_PRINT_BUFFER_SIZE; i++)
		print_buffer[i] = '\0';
}

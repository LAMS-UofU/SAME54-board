#include <atmel_start.h>
#include <hal_init.h>
#include <peripheral_clk_config.h>
#include <utils.h>
#include "driver_init.h"
#include "lidar.h"

/* LIDAR PWM Frequency 25kHz, 60% duty cycle */
#define LIDAR_PWM_COUNT				60
#define LIDAR_PWM_CC1				36

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

#define LIDAR_SEND_MODE_SINGLE_RES	0x0
#define LIDAR_SEND_MODE_MULTI_RES	0x1

#define MAX_PRINT_BUFFER_SIZE		256
#define LIDAR_RESP_DESC_SIZE		7
#define LIDAR_RESP_MAX_SIZE			128

#define MAX_SCANS					1024

struct usart_sync_descriptor LIDAR_USART;

/*  start1        - 1 byte (0xA5)
	start2        - 1 byte (0x5A)
	response_info - 32 bits ([32:30] send_mode, [29:0] data_length)
	data_type 	  - 1 byte	*/
struct response_descriptor {
	uint8_t start1;
	uint8_t start2;
	uint32_t response_info;
	uint8_t data_type;
} volatile resp_desc;

/*	distance1    - 15 bits 
	distance2    - 15 bits
	angle_value1 - 5 bits
	angle_value2 - 5 bits 
	S			 - 1 bit */
struct cabin_data {
	uint8_t S;
	uint8_t angle_value1;
	uint8_t angle_value2;
	uint16_t distance1;
	uint16_t distance2;
	uint16_t start_angle;
} cabins[MAX_SCANS];

/*  quality - 6 bits
	angle   - 15 bits
	distance - 16 bits */
struct scan_data {
	uint8_t quality;
	uint16_t angle;
	uint16_t distance;	
} scans[MAX_SCANS];
uint32_t scan_count;
uint32_t invalid_exp_scans;

/* Buffer for printing */
static char print_buffer[MAX_PRINT_BUFFER_SIZE];

/* Populated from USART3_4_IRQHandler, processed in LIDAR_process */
extern volatile char lidar_received_value;
extern volatile int lidar_newData_flag;

/* For "STOP" and "RESET" commands requiring waiting specific times before another
	 request is allowed */
volatile uint8_t lidar_timer;
volatile uint8_t lidar_timing;

/* Preserved value of which request was given until new request is sent */
uint8_t lidar_request;

/* Number of bytes received for processing LiDAR responses */
static uint32_t byte_count;
static uint16_t buffer_length;
static uint8_t processing;

/* Preserve response bytes */
static char DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

uint8_t lidar_menu_txt[] = "\r\n******** Enter choice ******** \r\n \
1. Back to main menu\r\n \
2. Start motor\r\n \
3. Stop motor\r\n \
4. Send stop command\r\n \
5. Send reset command\r\n \
6. Send scan command\r\n \
7. Send express scan command\r\n \
8. Send force scan command\r\n \
9. Send get_info command\r\n \
A. Send get_health command\r\n \
B. Send get_samplerate command\r\n";

void LIDAR_menu(void)
{
	uint16_t user_selection = 0;
	while (1) {
		if (processing)
			LIDAR_process();
		else {
			printf("%s", lidar_menu_txt);
		
			if (scanf("%hx", &user_selection) == 0) {
				/* If its not a number, flush stdin */
				fflush(stdin);
				continue;
			}
		
			printf("\r\nSelected option is %d\r\n", (int)user_selection);
		
			switch (user_selection) {
				case 1:
					printf("\r\nReturning to main menu\r\n");
					return;
			
				case 2:
					printf("\r\nStarting LiDAR motor\r\n");
					LIDAR_PWM_start();
					break;
			
				case 3:
					printf("\r\nStopping LiDAR motor\r\n");
					LIDAR_PWM_stop();
					break;
			
				case 4:
					printf("\r\nRequesting LiDAR stop\r\n");
					LIDAR_REQ_stop();
					break;
			
				case 5:
					printf("\r\nRequesting LiDAR reset\r\n");
					LIDAR_REQ_reset();
					break;
			
				case 6:
					printf("\r\nRequesting LiDAR start scan\r\n");
					LIDAR_REQ_scan();
					processing = 1;
					break;
			
				case 7:
					printf("\r\nRequesting LiDAR start express scan\r\n");
					LIDAR_REQ_express_scan();
					processing = 1;
					break;
			
				case 8:
					printf("\r\nRequesting LiDAR start force scan\r\n");
					LIDAR_REQ_force_scan();
					processing = 1;
					break;
			
				case 9:
					printf("\r\nRetrieving LiDAR info\r\n");
					LIDAR_REQ_get_info();
					processing = 1;
					break;
			
				case 10:
					printf("\r\nRetrieving LiDAR health\r\n");
					LIDAR_REQ_get_health();
					processing = 1;
					break;
			
				case 11:
					printf("\r\nRetrieving LiDAR samplerates\r\n");
					LIDAR_REQ_get_samplerate();
					processing = 1;
					break;
			
				default:
					printf("\r\nInvalid option\r\n");
					break;
			}
		}
	}
}

//void LIDAR_NewData_Handler(void)
//{
	//printf("Inside LIDAR_NewData_Handler\r\n");
//}
//
//void LIDAR_NewData_Config(void)
//{
	//// Disable USART to edit
	//hri_sercomusart_clear_CTRLA_ENABLE_bit(SERCOM2);
//
	//// enable interrupt flag
	//hri_sercomusart_set_INTEN_RXC_bit(SERCOM2);
	//
	//// Enable 
	//NVIC_SetPriority(SERCOM0_0_IRQn, 1);
	//NVIC_EnableIRQ(SERCOM0_0_IRQn);
	//NVIC_SetPriority(SERCOM0_1_IRQn, 1);
	//NVIC_EnableIRQ(SERCOM0_1_IRQn);
	//
	//
	//// Enable USART once again
	//hri_sercomusart_set_CTRLA_ENABLE_bit(SERCOM2);
//}

void LIDAR_process(void)
{
	unsigned data_idx;
	
	while (!usart_sync_is_rx_not_empty(&LIDAR_USART));
	
	/* Process response descriptor */
	switch (byte_count) {
		case 0:
			resp_desc.start1 = LIDAR_USART_read_byte();
			byte_count++;
			return;
		
		case 1:
			resp_desc.start2 = LIDAR_USART_read_byte();
			byte_count++;
			return;
		
		case 2:
			resp_desc.response_info = LIDAR_USART_read_byte();
			byte_count++;
			return;
		
		case 3:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 8);
			byte_count++;
			return;
		
		case 4:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 16);
			byte_count++;
			return;
		
		case 5:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 24);
			byte_count++;
			return;
		
		case 6:
			resp_desc.data_type = LIDAR_USART_read_byte();
			byte_count++;
			return;
		
		default:
			data_idx = byte_count - LIDAR_RESP_DESC_SIZE;
			DATA_RESPONSE[data_idx] = LIDAR_USART_read_byte();
			if (lidar_request == LIDAR_REQ_EXPRESS_SCAN) {
				if (data_idx == 0) { // check sync -- 0xA
					if ((DATA_RESPONSE[data_idx] >> 4) != 0x0A )
						return;	 
				}
				else if (data_idx == 1) { // check sync -- 0x5
					if ((DATA_RESPONSE[data_idx] >> 4) != 0x05) {
						byte_count--; // decrement and wait for another 0xA to come back in sync
						return;
					}
				}
			}
			byte_count++;
	};
	
	if (byte_count == (resp_desc.response_info & 0x3FFFFFFF) + LIDAR_RESP_DESC_SIZE) {
		switch(lidar_request) {
			// RESET system to stop scans in debugging
			case  LIDAR_REQ_SCAN:
			case  LIDAR_REQ_FORCE_SCAN:
				LIDAR_RES_scan();
				if (scan_count >= MAX_SCANS) {
					LIDAR_print_scans();
					break;
				}
				return;
			case  LIDAR_REQ_EXPRESS_SCAN:
				LIDAR_RES_express_scan();
				if (scan_count >= MAX_SCANS) {
					LIDAR_PWM_stop();
					LIDAR_REQ_stop();
					LIDAR_print_cabins();
					break;
				}
				return;
			
			case  LIDAR_REQ_GET_INFO:
				LIDAR_RES_get_info();
				break;
			case  LIDAR_REQ_GET_HEALTH:
				LIDAR_RES_get_health();
				break;
			case  LIDAR_REQ_GET_SAMPLERATE:
				LIDAR_RES_get_samplerate();
				break;
			default:
				return;
		};
		byte_count = 0;
		processing = 0;
		return;
	}
}

/** 
	*	Resets local print buffer.
	*	@return None
	*/
void LIDAR_reset_print_buffer(void)
{
	int i;
	for (i=0; i<MAX_PRINT_BUFFER_SIZE; i++)
		print_buffer[i] = '\0';
}

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

void LIDAR_print_scans(void) 
{
	int i;
	for (i=0; i<MAX_SCANS; i++) {
		printf("{\"S[%04u]\":{\"Q\":%u,\"A\":%u,\"D\"%u}}\r\n",
				i, scans[i].quality, scans[i].angle, scans[i].distance);
	}	
}
	
void LIDAR_print_cabins(void) 
{
	int i;
	for (i=0; i<MAX_SCANS; i++) {
		printf("{\"C[%04u]\":{\"S\":%u,\"SA\":%u,\"A1\":%u,\"A2\":%u,\"D1\":%u,\"D2\":%u}}\r\n",
			   i, cabins[i].S, cabins[i].start_angle, cabins[i].angle_value1, cabins[i].angle_value2, cabins[i].distance1, cabins[i].distance2);
	}
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
	
	///** Wait 1 ms. Decrementing in @see SysTick_Handler every 1 ms */
	//lidar_timer = 1;
	//lidar_timing = 1;
	delay_ms(1);
	
	LIDAR_RES_stop();
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
	
	///** Wait 2 ms. Decrementing in @see SysTick_Handler every 1 ms */
	//lidar_timer = 2;
	//lidar_timing = 1;
	delay_ms(2);
	
	LIDAR_RES_reset();
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
	// wait 1ms to ensure spinning
	//uint32_t present_count = systick_count;
	//while (systick_count < present_count + 1);
	
	lidar_request = LIDAR_REQ_SCAN;
	byte_count = 0;
	scan_count = 0;
	
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
	scan_count = 0;
	invalid_exp_scans = 0;
	
	lidar_request = LIDAR_REQ_EXPRESS_SCAN;
	byte_count = 0;
	
	LIDAR_reset_print_buffer();
	buffer_length = snprintf(print_buffer, MAX_PRINT_BUFFER_SIZE, 
						"%c%c%c", LIDAR_START_BYTE, LIDAR_REQ_EXPRESS_SCAN, payload_size); 
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
	* @return None
	*/
void LIDAR_REQ_force_scan(void)
{
	LIDAR_PWM_start();
	
	lidar_request = LIDAR_REQ_FORCE_SCAN;
	byte_count = 0;
	scan_count = 0;
	
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
	LIDAR_reset_response_descriptor();
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
	LIDAR_reset_response_descriptor();
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
	LIDAR_reset_response_descriptor();
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
	//if (!lidar_timer) {
		//byte_count = 0;
		//lidar_timing = 0;
		//printf("lidar_timer = 0\r\nLiDAR stopped\r\n");
	//}
	printf("LiDAR stopped\r\n");
}
	
/** 
	* "RESET" request has no response. Instead using @see SysTick_Handler to wait 
	* 2 ms with precision.
	
	* If no command sent after 2ms, sends back partial info resulting in:
	* RP LIDAR System.\r\n
	* Firmware Ver 1.27 - rc9, HW Ver 5\r\n
	* Model: 28\r\n
	* @return None
	*/
void LIDAR_RES_reset(void) 
{
	//if (!lidar_timer) {
		//byte_count = 0;
		//lidar_timing = 0;
		//printf("lidar_timer = 0\r\nLiDAR reset\r\n");
	//}
	printf("LiDAR reset\r\n");
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
	
	if (scan_count % 16 == 0)
		printf("gathered %0d/%0d scans...\r\n", scan_count, MAX_SCANS);
	
	/* Prints invalid responses...not all responses are supposed to valid, so no
		 need to print these except for debugging */
	//else {
		//printf("Invalid response: C=%u, !S=%u, S=%u\r\n",
			   //(check >> 3), ((check >> 2) & 0x1), (check & 0x1));
	//}
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
	* @return None
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
	
	/* Check if in sync -- moved up into LIDAR_process in attempt to reduce errors/wasted time */
	//if ((DATA_RESPONSE[0] >> 4) != 0x0A)
		//return;
	//if ((DATA_RESPONSE[1] >> 4) != 0x05)
		//return;
	
	/* Check if data valid */
	calc_checksum = 0;
	for (i=2; i<PAYLOAD_SIZE; i++)
		calc_checksum ^= DATA_RESPONSE[i];
		
	if (checksum != calc_checksum) {
		//if (invalid_exp_scans % 100 == 0) {
			//printf("Invalid checksum! Given: %u -- Calculated: %u\r\n", checksum, calc_checksum);
			//printf("First 8: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", 
					//DATA_RESPONSE[0], DATA_RESPONSE[1], DATA_RESPONSE[2], DATA_RESPONSE[3], 
					//DATA_RESPONSE[4], DATA_RESPONSE[5], DATA_RESPONSE[6], DATA_RESPONSE[7]);
		//}
		invalid_exp_scans++;
		//printf("%0d invalid scans -- gathered %0d/%0d scans...\r\n", invalid_exp_scans, scan_count, MAX_SCANS);
		//snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							//"Invalid checksum!\r\nGiven: %u\r\nCalculated: %u\r\n",
							//checksum, calc_checksum);
		//printf(print_buffer);
		return;
	}
	
	//snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						//"{\"A\":%u,", start_angle);
	//printf(print_buffer);
	
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
		//snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
							//"\t{\"C[%u]\":{\"A1\":%u,\"A2\":%u,\"D1\":%u,\"D2\":%u}\r\n",
							//i, cabins[i].angle_value1, cabins[i].angle_value2,
							//cabins[i].distance1, cabins[i].distance2);
		//printf(print_buffer);
	}
	
	printf("%0d invalid scans -- gathered %0d/%0d scans...\r\n", invalid_exp_scans, scan_count, MAX_SCANS);
	
	//printf("}\r\n");
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
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : RPLiDAR Model ID: %u\r\n", model_id);
	printf(print_buffer);
	
	LIDAR_reset_print_buffer();
	snprintf( print_buffer, MAX_PRINT_BUFFER_SIZE,
						" : Firmware Version: %u.%u\r\n", 
						firmware_major, firmware_minor);
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
	
	if (error_code == 0)
		printf(" : LiDAR Health is %s!\r\n", status);
	else
		printf(" : LiDAR Health is %s!\r\n : Error code: %u\r\n", 
				status, error_code);
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
	printf(" : Standard Scan Samplerate: %u\r\n", 
			 DATA_RESPONSE[0] + ((unsigned)DATA_RESPONSE[1] << 8));
	
	printf(" : Express Scan Samplerate: %u\r\n", 
						DATA_RESPONSE[2] + ((unsigned)DATA_RESPONSE[3] << 8));
}

// LIDAR requires internal pull-down resistor
void LIDAR_PWM_PORT_init(void)
{
	gpio_set_pin_function(PB09, PINMUX_PB09E_TC4_WO1);
	gpio_set_pin_pull_mode(PB09, GPIO_PULL_DOWN);
}

void LIDAR_PWM_CLOCK_init(void)
{
	hri_mclk_set_APBCMASK_TC4_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, 
							   TC4_GCLK_ID, 
							   CONF_GCLK_TC4_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void LIDAR_PWM_init(void)
{
	if (!hri_tc_is_syncing(TC4, TC_SYNCBUSY_SWRST)) {
		if (hri_tc_get_CTRLA_reg(TC4, TC_CTRLA_ENABLE)) {
			hri_tc_clear_CTRLA_ENABLE_bit(TC4);
			hri_tc_wait_for_sync(TC4, TC_SYNCBUSY_ENABLE);
		}
		hri_tc_write_CTRLA_reg(TC4, TC_CTRLA_SWRST);
	}
	hri_tc_wait_for_sync(TC4, TC_SYNCBUSY_SWRST);
	
	hri_tc_write_CTRLA_reg(TC4,
			  2 << TC_CTRLA_CAPTMODE0_Pos	/* Capture mode Channel 0: 2 */
			| 2 << TC_CTRLA_CAPTMODE1_Pos	/* Capture mode Channel 1: 2 */
			| 0 << TC_CTRLA_COPEN0_Pos		/* Capture Pin 0 Enable: disabled */
			| 0 << TC_CTRLA_COPEN1_Pos		/* Capture Pin 1 Enable: disabled */
			| 0 << TC_CTRLA_CAPTEN0_Pos		/* Capture Channel 0 Enable: disabled */
			| 0 << TC_CTRLA_CAPTEN1_Pos		/* Capture Channel 1 Enable: disabled */
			| 0 << TC_CTRLA_ALOCK_Pos		/* Auto Lock: disabled */
			| 1 << TC_CTRLA_PRESCSYNC_Pos	/* Prescaler and Counter Synchronization: 1 */
			| 0 << TC_CTRLA_ONDEMAND_Pos	/* Clock On Demand: disabled */
			| 0 << TC_CTRLA_RUNSTDBY_Pos	/* Run in Standby: disabled */
			| 3 << TC_CTRLA_PRESCALER_Pos	/* Setting: 3 - divide by 8 */
			| 0x0 << TC_CTRLA_MODE_Pos);	/* Operating Mode: 0x0 */
	hri_tc_write_CTRLB_reg(TC4,
			  0 << TC_CTRLBSET_CMD_Pos		/* Command: 0 */
			| 0 << TC_CTRLBSET_ONESHOT_Pos	/* One-Shot: disabled */
			| 0 << TC_CTRLBCLR_LUPD_Pos		/* Setting: disabled */
			| 0 << TC_CTRLBSET_DIR_Pos);	/* Counter Direction: disabled */
	hri_tc_write_WAVE_reg(TC4, 3);	/* Waveform Generation Mode: 3 - MPWM */
	hri_tccount16_write_CC_reg(TC4, 0, LIDAR_PWM_COUNT); /* Compare/Capture Value: 60 */
	hri_tccount16_write_CC_reg(TC4, 1, 0); /* Compare/Capture Value: 60 - OFF */
	hri_tc_write_CTRLA_ENABLE_bit(TC4, 1 << TC_CTRLA_ENABLE_Pos); /* Enable: enabled */
}

void LIDAR_PWM_start(void)
{
	if (!hri_tc_is_syncing(TC4, TC_SYNCBUSY_SWRST)) {
		if (hri_tc_get_CTRLA_reg(TC4, TC_CTRLA_ENABLE)) {
			hri_tc_clear_CTRLA_ENABLE_bit(TC4);
			hri_tc_wait_for_sync(TC4, TC_SYNCBUSY_ENABLE);
		}
	}
	hri_tc_wait_for_sync(TC4, TC_SYNCBUSY_SWRST);
	
	hri_tccount16_write_CC_reg(TC4, 1, LIDAR_PWM_CC1);
	hri_tc_write_CTRLA_ENABLE_bit(TC4, 1 << TC_CTRLA_ENABLE_Pos);
}

void LIDAR_PWM_stop(void)
{
	if (!hri_tc_is_syncing(TC4, TC_SYNCBUSY_SWRST)) {
		if (hri_tc_get_CTRLA_reg(TC4, TC_CTRLA_ENABLE)) {
			hri_tc_clear_CTRLA_ENABLE_bit(TC4);
			hri_tc_wait_for_sync(TC4, TC_SYNCBUSY_ENABLE);
		}
	}
	hri_tc_wait_for_sync(TC4, TC_SYNCBUSY_SWRST);
	
	hri_tccount16_write_CC_reg(TC4, 1, 0);
	hri_tc_write_CTRLA_ENABLE_bit(TC4, 1 << TC_CTRLA_ENABLE_Pos);
}

void LIDAR_USART_PORT_init(void)
{
	gpio_set_pin_function(PA04, PINMUX_PA04D_SERCOM0_PAD0); // TX
	gpio_set_pin_function(PA05, PINMUX_PA05D_SERCOM0_PAD1); // RX
}

void LIDAR_USART_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK,
							   SERCOM0_GCLK_ID_CORE,
							   CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK,
							   SERCOM0_GCLK_ID_SLOW,
							   CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBAMASK_SERCOM0_bit(MCLK);
}

void LIDAR_USART_init(void)
{
	LIDAR_USART_CLOCK_init();
	usart_sync_init(&LIDAR_USART, SERCOM0, (void *)NULL);
	//usart_sync_set_mode(&LIDAR_USART, USART_MODE_SYNCHRONOUS);
	//usart_sync_set_baud_rate(&LIDAR_USART, 115200);
	//usart_sync_set_data_order(&LIDAR_USART, USART_DATA_ORDER_LSB);
	//usart_sync_set_parity(&LIDAR_USART, USART_PARITY_NONE);
	//usart_sync_set_stopbits(&LIDAR_USART, USART_STOP_BITS_ONE);
	//usart_sync_set_character_size(&LIDAR_USART, USART_CHARACTER_SIZE_8BITS);
	usart_sync_enable(&LIDAR_USART);
	LIDAR_USART_PORT_init();
}

void LIDAR_USART_send(uint8_t* message, uint16_t length)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&LIDAR_USART, &io);
	
	io_write(io, message, length);
}

uint8_t LIDAR_USART_read_byte(void)
{
	uint8_t buf;
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&LIDAR_USART, &io);
	
	io_read(io, &buf, 1);
	return buf;
}

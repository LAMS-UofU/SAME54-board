#include "scan.h"

#define PROCESSING 0
#define COMPLETED  1

static void scan_error(uint16_t);
static uint8_t process(void);
static void process_cabins(void);

extern volatile uint8_t status;
extern volatile uint32_t start_time;

/* LIDAR variables */
extern response_descriptor resp_desc;
extern cabin_data cabins[MAX_SCANS];
extern uint32_t scan_count;
extern uint8_t lidar_request;
extern volatile uint32_t byte_count;
extern volatile uint8_t lidar_timing;
extern volatile char DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

void scan(void)
{
	int angle;
	uint16_t error_code;
	
	if (DEBUG)
		printf("\r\nPress button to start\r\n");
					
	/* Wait until button pressed */
	while (!gpio_get_pin_level(START_BTN));
	
	if (DEBUG)
		printf("\r\nStarting scan\r\n");
	
	LIDAR_REQ_reset();
	while (!process()) {
		LIDAR_RES_reset();
	}
	
	LIDAR_REQ_get_health();
	while (!process());
	error_code = LIDAR_RES_get_health();
	
	if (error_code)
		scan_error(error_code);
	
	LIDAR_REQ_get_info();
	while (!process());
	LIDAR_RES_get_info();
	
	LIDAR_PWM_start();
	LIDAR_REQ_express_scan();
		
	for (angle = 0; angle <= 180; angle++) {
		SERVO_set_angle(angle);
		
		while (scan_count < MAX_SCANS) {
			while (!process());
			LIDAR_RES_express_scan();
		}
		
		process_cabins();
	}
	
	LIDAR_PWM_stop();
	LIDAR_REQ_stop();
	while (!process()) {
		LIDAR_RES_stop();
	}
	
	status = STATUS_IDLE;
}

/**
  *
  */
uint8_t process(void)
{
	unsigned data_idx;
	
	status = STATUS_PROCESSING; 
	
	/* STOP and RESET requests */
	if (lidar_request == LIDAR_STOP || 
		lidar_request == LIDAR_RESET) {
		if (lidar_timing) 
			return PROCESSING;
		else
			return COMPLETED;
	}
	
	/* Check for timeout -- wait 1 second */
	//if (systick_count >= start_time + 1000) {
		//scan_error(0);
	//}
	
	if (!usart_sync_is_rx_not_empty(&LIDAR_USART))
		return PROCESSING;
	
	/* Process response descriptor */
	switch (byte_count) {
		case 0:
			resp_desc.start1 = LIDAR_USART_read_byte();
			/* check sync -- 0xA5 */
			if (resp_desc.start1 == 0xA5)
				byte_count++;
			return PROCESSING;
		
		case 1:
			resp_desc.start2 = LIDAR_USART_read_byte();
			/* check sync -- 0x5A */
			if (resp_desc.start2 != 0x5A)
				byte_count--;
			else
				byte_count++;
			return PROCESSING;
		
		case 2:
			resp_desc.response_info = LIDAR_USART_read_byte();
			byte_count++;
			return PROCESSING;
		
		case 3:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 8);
			byte_count++;
			return PROCESSING;
		
		case 4:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 16);
			byte_count++;
			return PROCESSING;
		
		case 5:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 24);
			byte_count++;
			return PROCESSING;
		
		case 6:
			resp_desc.data_type = LIDAR_USART_read_byte();
			byte_count++;
			return PROCESSING;
		
		/* Process response data packets */
		default:
			data_idx = byte_count - LIDAR_RESP_DESC_SIZE;
			DATA_RESPONSE[data_idx] = LIDAR_USART_read_byte();
			
			if (lidar_request == LIDAR_EXPRESS_SCAN) {
				/* check sync -- 0xA */
				if (data_idx == 0) {
					if ((DATA_RESPONSE[data_idx] >> 4) != 0x0A )
						return PROCESSING;
				}
				/* check sync -- 0x5 */
				else if (data_idx == 1) {
					if ((DATA_RESPONSE[data_idx] >> 4) != 0x05) {
						/* decrement and wait for another 0xA to come back in sync */
						byte_count--;
						return PROCESSING;
					}
				}
			}
			byte_count++;
	};
	
	if (byte_count == (resp_desc.response_info & 0x3FFFFFFF) + LIDAR_RESP_DESC_SIZE) {
		return COMPLETED;
	} else if (byte_count > (resp_desc.response_info & 0x3FFFFFFF) + LIDAR_RESP_DESC_SIZE) {
		scan_error(1);
	}
	
	return PROCESSING;
}

void scan_error(uint16_t error_code)
{
	printf("\r\n[Scan Error]\r\n");

	LIDAR_PWM_stop();

	status = STATUS_ERROR;

	switch (error_code) {
		case 0: 
			printf(" | Timeout Error\r\n");
			break;
		case 1:
			printf(" | Out-of-Bounds Error\r\n");
			printf(" | | byte count (%"PRIu32") went past the response descriptor limit (%lu)\r\n",
					byte_count, (resp_desc.response_info & 0x3FFFFFFF) + LIDAR_RESP_DESC_SIZE);
			break;
		default:
			printf(" | Error code %u\r\n", error_code);
	};

	while (1);
}

/**
  *	Process cabin data
  */
void process_cabins(void)
{
	int i;
	
	if (DEBUG) {
		for (i=0; i<MAX_SCANS; i++) {
			printf("{\"C[%04u]\":{\"S\":%u,\"SA\":%u,\"A1\":%u,\"A2\":%u,\"D1\":%u,\"D2\":%u}}\r\n",
					i, cabins[i].S, cabins[i].start_angle, cabins[i].angle_value1, cabins[i].angle_value2, cabins[i].distance1, cabins[i].distance2);
		}
	}
}
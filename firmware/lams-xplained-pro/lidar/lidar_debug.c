#include "lidar.h"

static void LIDAR_process(void);
static void LIDAR_print_scans(void);
static void LIDAR_print_cabins(void);

/* See lidar.h for declarations */
extern response_descriptor resp_desc;
extern cabin_data cabins[MAX_SCANS];
extern scan_data scans[MAX_SCANS];
extern uint32_t scan_count;
extern uint32_t invalid_exp_scans;
extern uint8_t lidar_request;
extern uint32_t byte_count;
extern uint16_t buffer_length;
extern uint8_t processing;
extern volatile char DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];
extern volatile uint8_t lidar_timer;
extern volatile uint8_t lidar_timing;

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

/**
  * Menu for LiDAR command options in order to test and see printouts for each
  * of the requests and responses
  */ 
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
					processing = 1;
					break;
			
				case 5:
					printf("\r\nRequesting LiDAR reset\r\n");
					LIDAR_REQ_reset();
					processing = 1;
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

/**
  * Process incoming data from LiDAR based on the request given.
  */ 
void LIDAR_process(void)
{
	unsigned data_idx;
	
	/* STOP and RESET requests */
	if (lidar_timing) {
		switch (lidar_request) {
			case LIDAR_REQ_STOP:
				LIDAR_RES_stop();
				break;
			case LIDAR_REQ_RESET:
				LIDAR_RES_reset();
				break;
		};
		return;
	}
	
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
		
        /* Process response data packets */
		default:
			data_idx = byte_count - LIDAR_RESP_DESC_SIZE;
			DATA_RESPONSE[data_idx] = LIDAR_USART_read_byte();
			if (lidar_request == LIDAR_REQ_EXPRESS_SCAN) {
				/* check sync -- 0xA */
                if (data_idx == 0) { 
					if ((DATA_RESPONSE[data_idx] >> 4) != 0x0A )
						return;	 
				}
                /* check sync -- 0x5 */
				else if (data_idx == 1) {
					if ((DATA_RESPONSE[data_idx] >> 4) != 0x05) {
                        /* decrement and wait for another 0xA to come back in sync */
						byte_count--;
						return;
					}
				}
			}
			byte_count++;
	};
	
	if (byte_count == (resp_desc.response_info & 0x3FFFFFFF) + LIDAR_RESP_DESC_SIZE) {
		switch(lidar_request) {
			/* RESET system to stop scans while debugging */
			case LIDAR_REQ_SCAN:
			case LIDAR_REQ_FORCE_SCAN:
				LIDAR_RES_scan();
				if (scan_count >= MAX_SCANS) {
					LIDAR_PWM_stop();
					LIDAR_REQ_stop();
					LIDAR_print_scans();
					break;
				}
				return;

			case LIDAR_REQ_EXPRESS_SCAN:
				LIDAR_RES_express_scan();
				if (scan_count >= MAX_SCANS) {
					LIDAR_PWM_stop();
					LIDAR_REQ_stop();
					//LIDAR_print_cabins();
					break;
				}
				return;
			
			case LIDAR_REQ_GET_INFO:
				LIDAR_RES_get_info();
				break;

			case LIDAR_REQ_GET_HEALTH:
				LIDAR_RES_get_health();
				break;

			case LIDAR_REQ_GET_SAMPLERATE:
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
  *	Prints scan data
  */ 
void LIDAR_print_scans(void) 
{
	int i;
	for (i=0; i<MAX_SCANS; i++) {
		printf("{\"S[%04u]\":{\"Q\":%u,\"A\":%u,\"D\"%u}}\r\n",
				i, scans[i].quality, scans[i].angle, scans[i].distance);
	}	
}

/**
  *	Prints cabin data
  */ 	
void LIDAR_print_cabins(void) 
{
	int i;
	for (i=0; i<MAX_SCANS; i++) {
		printf("{\"C[%04u]\":{\"S\":%u,\"SA\":%u,\"A1\":%u,\"A2\":%u,\"D1\":%u,\"D2\":%u}}\r\n",
			   i, cabins[i].S, cabins[i].start_angle, cabins[i].angle_value1, cabins[i].angle_value2, cabins[i].distance1, cabins[i].distance2);
	}
}

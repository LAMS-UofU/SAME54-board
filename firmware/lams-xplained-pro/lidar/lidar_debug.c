#include "lidar.h"
#include "common.h"

static void LIDAR_process(void);

/* See lidar.c for declarations */
extern resp_desc_s		resp_desc;
extern write_data_s     sd_scan_data[MAX_SCANS];
extern conf_data_t		conf_data;
extern uint32_t			scan_count;
extern uint32_t			invalid_exp_scans;
extern uint8_t			lidar_request;
extern uint32_t			byte_count;
extern uint16_t			buffer_length;
extern uint8_t			processing;
extern volatile char	DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

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
B. Send get_samplerate command\r\n \
C. Send get_lidar_conf command\r\n \
D. Send motor_speed_ctrl_command\r\n";

uint8_t lidar_conf_menu[] = "\r\n******** Enter choice ******** \r\n \
1. Back to lidar menu\r\n \
2. SCAN_MODE_COUNT\r\n \
3. SCAN_MODE_US_PER_SAMPLE\r\n \
4. SCAN_MODE_MAX_DISTANCE\r\n \
5. SCAN_MODE_ANS_TYPE\r\n \
6. SCAN_MODE_TYPICAL\r\n \
7. SCAN_MODE_NAME\r\n";

uint8_t lidar_conf_scan_menu[] = "\r\n******** Enter choice ******** \r\n \
1. Standard\r\n \
2. Express\r\n \
3. Boost\r\n \
4. Stability\r\n";

uint8_t lidar_exp_scan_menu[] = "\r\n******** Enter choice ******** \r\n \
1. Standard\r\n \
2. Boost\r\n \
3. Stability\r\n";

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
					LIDAR_REQ_express_scan(LIDAR_express_scan_menu());
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
				
				case 12:
					LIDAR_conf_menu();
					break;
					
				case 13:
					break;
			
				default:
					printf("\r\nInvalid option\r\n");
					break;
			}
		}
	}
}

void LIDAR_conf_menu(void)
{
	uint16_t user_selection = 0;
	
	while (1) {
		if (processing)
			LIDAR_process();
		else {
			printf("%s", lidar_conf_menu);
		
			if (scanf("%hx", &user_selection) == 0) {
				/* If its not a number, flush stdin */
				fflush(stdin);
				continue;
			}
			
			printf("\r\nSelected option is %d\r\n", (int)user_selection);
		
			switch (user_selection) {
				case 1:
					printf("\r\nReturning to lidar menu\r\n");
					return;
				
				case 2:
					printf("\r\nRequesting LiDAR configuration mode count\r\n");
					LIDAR_REQ_get_lidar_conf(CONF_SCAN_MODE_COUNT, 0);
					processing = 1;
					break;
				
				case 3:
					printf("\r\nRequesting LiDAR sample duration of scan mode\r\n");
					LIDAR_REQ_get_lidar_conf(CONF_SCAN_MODE_US_PER_SAMPLE, LIDAR_conf_scan_menu());
					processing = 1;
					break;
				
				case 4:
					printf("\r\nRequesting LiDAR scan mode max distance\r\n");
					LIDAR_REQ_get_lidar_conf(CONF_SCAN_MODE_MAX_DISTANCE, LIDAR_conf_scan_menu());
					processing = 1;
					break;
				
				case 5:
					printf("\r\nRequesting LiDAR start force scan\r\n");
					LIDAR_REQ_get_lidar_conf(CONF_SCAN_MODE_ANS_TYPE, LIDAR_conf_scan_menu());
					processing = 1;
					break;
				
				case 6:
					printf("\r\nRequesting LiDAR answer command type for scan mode\r\n");
					LIDAR_REQ_get_lidar_conf(CONF_SCAN_MODE_TYPICAL, 0);
					processing = 1;
					break;
				
				case 7:
					printf("\r\nRequesting LiDAR scan mode name\r\n");
					LIDAR_REQ_get_lidar_conf(CONF_SCAN_MODE_NAME, LIDAR_conf_scan_menu());
					processing = 1;
					break;
				
				default:
					printf("\r\nInvalid option\r\n");
					break;
			}
		}
	}
}

scan_mode_t LIDAR_conf_scan_menu(void)
{
	uint16_t user_selection = 0;
	
	while (1) {
		printf("%s", lidar_conf_scan_menu);
		
		if (scanf("%hx", &user_selection) == 0) {
			/* If its not a number, flush stdin */
			fflush(stdin);
			continue;
		}
		
		printf("\r\nSelected option is %d\r\n", (int)user_selection);
		
		switch (user_selection) {
			case 1: return SCAN_MODE_STANDARD;
			case 2: return SCAN_MODE_EXPRESS;
			case 3: return SCAN_MODE_BOOST;
			case 4: return SCAN_MODE_STABILITY;
			default:
				printf("\r\nInvalid option\r\n");
				break;
		}
	}
}

express_mode_t LIDAR_express_scan_menu(void)
{
	uint16_t user_selection = 0;
	
	while (1) {
		printf("%s", lidar_conf_scan_menu);
		
		if (scanf("%hx", &user_selection) == 0) {
			/* If its not a number, flush stdin */
			fflush(stdin);
			continue;
		}
		
		printf("\r\nSelected option is %d\r\n", (int)user_selection);
		
		switch (user_selection) {
			case 1: return EXPRESS_SCAN_STANDARD;
			case 2: return EXPRESS_SCAN_BOOST;
			case 3: return EXPRESS_SCAN_STABILITY;
			default:
			printf("\r\nInvalid option\r\n");
			break;
		}
	}
}

/**
  * Process incoming data from LiDAR based on the request given.
  */ 
void LIDAR_process(void)
{
	unsigned data_idx;
	
	while (!usart_sync_is_rx_not_empty(&LIDAR_USART));
	
	/* Process response descriptor */
	switch (byte_count) {
		case 0:
			resp_desc.start1 = LIDAR_USART_read_byte();
			/* check sync -- 0xA5 */
			if (resp_desc.start1 == 0xA5)
				byte_count++;
			return;
		
		case 1:
			resp_desc.start2 = LIDAR_USART_read_byte();
			/* check sync -- 0x5A */
			if (resp_desc.start2 != 0x5A)
				byte_count--;
			else
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
			if (lidar_request == LIDAR_EXPRESS_SCAN) {
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
			case LIDAR_SCAN:
			case LIDAR_FORCE_SCAN:
				//LIDAR_RES_scan();
				//if (scan_count >= MAX_SCANS) {
					//LIDAR_PWM_stop();
					//LIDAR_REQ_stop();
					//break;
				//}
				return;

			case LIDAR_EXPRESS_SCAN:
				LIDAR_RES_express_scan();
				if (scan_count >= MAX_SCANS) {
					LIDAR_PWM_stop();
					LIDAR_REQ_stop();
					break;
				}
				return;
			
			case LIDAR_GET_INFO:
				printf("%s\r\n", LIDAR_RES_get_info());
				break;

			case LIDAR_GET_HEALTH:
				LIDAR_RES_get_health();
				break;

			case LIDAR_GET_SAMPLERATE:
				LIDAR_RES_get_samplerate();
				break;
			
			case LIDAR_GET_LIDAR_CONF:
				LIDAR_RES_get_lidar_conf();
				break;

			default:
				return;
		};
		byte_count = 0;
		processing = 0;
		return;
	}
}
#include <atmel_start.h>
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include "driver_init.h"
#include "lidar.h"

/* LIDAR PWM Frequency 25kHz, 60% duty cycle */
#define LIDAR_PWM_COUNT			60
#define LIDAR_PWM_CC1			36

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

#define LIDAR_RESP_DESC_SIZE 7
#define LIDAR_RESP_MAX_SIZE	 1080

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
10. Send get_health command\r\n \
11. Send get_samplerate command\r\n";

void LIDAR_USART_PORT_init(void)
{
	gpio_set_pin_function(PA04, PINMUX_PA04D_SERCOM0_PAD0);
	gpio_set_pin_function(PA05, PINMUX_PA05D_SERCOM0_PAD1);
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
	LIDAR_USART_PORT_init();
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

void LIDAR_menu(void)
{
	uint32_t user_selection = 0;
	
	while (1) {
		printf("%s", lidar_menu_txt);
		
		if (scanf("%d", &user_selection) == 0) {
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
				// TODO 
				printf("\r\nERROR: not implemented yet\r\n");
				break;
			
			case 5:
				printf("\r\nRequesting LiDAR reset\r\n");
				// TODO
				printf("\r\nERROR: not implemented yet\r\n");
				break;
				
			case 6:
				printf("\r\nRequesting LiDAR start scan\r\n");
				// TODO
				printf("\r\nERROR: not implemented yet\r\n");
				break;
				
			case 7:
				printf("\r\nRequesting LiDAR start express scan\r\n");
				// TODO
				printf("\r\nERROR: not implemented yet\r\n");
				break;
			
			case 8:
				printf("\r\nRequesting LiDAR start force scan\r\n");
				// TODO
				printf("\r\nERROR: not implemented yet\r\n");
				break;
			
			case 9:
				printf("\r\nRetrieving LiDAR info\r\n");
				// TODO
				printf("\r\nERROR: not implemented yet\r\n");
				break;
			
			case 10:
				printf("\r\nRetrieving LiDAR health\r\n");
				// TODO
				printf("\r\nERROR: not implemented yet\r\n");
				break;
				
			case 11:
				printf("\r\nRetrieving LiDAR samplerates\r\n");
				// TODO
				printf("\r\nERROR: not implemented yet\r\n");
				break;
			
			default:
				printf("\r\nInvalid option\r\n");
				break;
		}
	}
}


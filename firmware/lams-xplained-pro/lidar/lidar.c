#include "lidar.h"
#include "common.h"
#include "drivers.h"
#include <peripheral_clk_config.h>
#include <string.h>

static void LIDAR_PWM_PORT_init(void);
static void LIDAR_PWM_CLOCK_init(void);
static void LIDAR_USART_PORT_init(void);
static void LIDAR_USART_CLOCK_init(void);

struct usart_sync_descriptor LIDAR_USART;

resp_desc_s				resp_desc = {0};
volatile write_data_s	sd_scan_data[MAX_SCANS] = {0};
conf_data_t				conf_data = { .resp2 = 0 };
uint8_t					processing = 0;

/* Scan counts so not to print/transfer data while scanning */
uint32_t scan_count = 0;
uint32_t invalid_exp_scans = 0;

/* Preserved value of which request was given until new request is sent */
uint8_t lidar_request = 0;
uint8_t lidar_config = 0;

/* Number of bytes received for processing LiDAR responses */
volatile uint32_t byte_count = 0;
uint16_t buffer_length = 0;

/* Preserve response bytes */
volatile char DATA_RESPONSE[LIDAR_RESP_MAX_SIZE] = {0};

/**
  * Initializes PWM port for LiDAR
  * 
  * Note: LIDAR requires internal pull-down resistor
  */  
void LIDAR_PWM_PORT_init(void)
{
	gpio_set_pin_function(PB09, PINMUX_PB09E_TC4_WO1);
	gpio_set_pin_pull_mode(PB09, GPIO_PULL_DOWN);
}

/**
  * Initializes PWM clock for LiDAR
  */ 
void LIDAR_PWM_CLOCK_init(void)
{
	hri_mclk_set_APBCMASK_TC4_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, 
							   TC4_GCLK_ID, 
							   CONF_GCLK_TC4_SRC | 
                               (1 << GCLK_PCHCTRL_CHEN_Pos));
}

/**
  * Initializes PWM LiDAR
  */ 
void LIDAR_PWM_init(void)
{
	LIDAR_PWM_CLOCK_init();
	LIDAR_PWM_PORT_init();
	
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

/**
  * Starts LiDAR PWM motor
  */ 
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

/**
  * Stops LiDAR PWM motor
  */ 
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

/**
  * Initializes USART ports for LiDAR
  */ 
void LIDAR_USART_PORT_init(void)
{
	gpio_set_pin_function(PA04, PINMUX_PA04D_SERCOM0_PAD0); /* Tx */
	gpio_set_pin_function(PA05, PINMUX_PA05D_SERCOM0_PAD1); /* Rx */
}

/**
  * Initializes USART clock for LiDAR
  */ 
void LIDAR_USART_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK,
							   SERCOM0_GCLK_ID_CORE,
							   CONF_GCLK_SERCOM0_CORE_SRC | 
                                    (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK,
							   SERCOM0_GCLK_ID_SLOW,
							   CONF_GCLK_SERCOM0_SLOW_SRC | 
                                    (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBAMASK_SERCOM0_bit(MCLK);
}

/**
  * Initializes USART for LiDAR
  */ 
void LIDAR_USART_init(void)
{
	LIDAR_USART_CLOCK_init();
	usart_sync_init(&LIDAR_USART, SERCOM0, (void *)NULL);
	usart_sync_enable(&LIDAR_USART);
	LIDAR_USART_PORT_init();
}

/**
  * Sends data through USART to LiDAR
  */ 
void LIDAR_USART_send(uint8_t* message, uint16_t length)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&LIDAR_USART, &io);
	
	io_write(io, message, length);
}

/**
  * Reads received byte from LiDAR through USART
  * 
  * @return uint8_t : received byte
  */ 
uint8_t LIDAR_USART_read_byte(void)
{
	uint8_t buf;
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&LIDAR_USART, &io);
	
	io_read(io, &buf, 1);
	return buf;
}

/**
  *	Returns string representation of request number
  */
char* request_str_repr(uint8_t req)
{
	switch (req) {
		case LIDAR_STOP:				return "LIDAR_STOP"; 
		case LIDAR_RESET:				return "RESET";
		case LIDAR_SCAN:				return "SCAN";
		case LIDAR_EXPRESS_SCAN:		return "EXPRESS_SCAN";
		case LIDAR_FORCE_SCAN:			return "FORCE_SCAN";
		case LIDAR_GET_INFO:			return "GET_INFO";
		case LIDAR_GET_HEALTH:			return "GET_HEALTH";
		case LIDAR_GET_SAMPLERATE:		return "GET_SAMPLERATE";
		case LIDAR_GET_LIDAR_CONF:		return "GET_LIDAR_CONF";
		case LIDAR_MOTOR_SPEED_CTRL:	return "MOTOR_SPEED_CTRL";
	}
	return "INVALID REQUEST GIVEN";
}
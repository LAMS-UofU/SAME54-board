#include <atmel_start.h>
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include "driver_init.h"
#include "servo.h"

#define SERVO_PWM_PERIOD_us		3030
#define SERVO_PWM_MINIMUM_us	500
#define SERVO_PWM_MAXIMUM_us	2500
#define SERVO_PWM_COUNT			2273
#define SERVO_PWM_CC1_MIN		375 

uint8_t servo_menu_txt[] = "\r\n******** Enter choice ******** \r\n \
1. Back to main menu\r\n \
2. Set servo angle\r\n";

void SERVO_PWM_PORT_init(void)
{
	gpio_set_pin_function(PA07, PINMUX_PA07E_TC1_WO1);
}

void SERVO_PWM_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_TC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, 
							   TC1_GCLK_ID, 
							   CONF_GCLK_TC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

/* Call in driver_init.c
 * Initial servo position at 0 degrees */
void SERVO_PWM_init(void)
{
	if (!hri_tc_is_syncing(TC1, TC_SYNCBUSY_SWRST)) {
		if (hri_tc_get_CTRLA_reg(TC1, TC_CTRLA_ENABLE)) {
			hri_tc_clear_CTRLA_ENABLE_bit(TC1);
			hri_tc_wait_for_sync(TC1, TC_SYNCBUSY_ENABLE);
		}
		hri_tc_write_CTRLA_reg(TC1, TC_CTRLA_SWRST);
	}
	hri_tc_wait_for_sync(TC1, TC_SYNCBUSY_SWRST);
	hri_tc_write_CTRLA_reg(TC1,
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
			| 4 << TC_CTRLA_PRESCALER_Pos	/* Setting: 4 - divide by 16 */
			| 0x0 << TC_CTRLA_MODE_Pos);	/* Operating Mode: 0x0 */
	hri_tc_write_CTRLB_reg(TC1,
			  0 << TC_CTRLBSET_CMD_Pos		/* Command: 0 */
			| 0 << TC_CTRLBSET_ONESHOT_Pos	/* One-Shot: disabled */
			| 0 << TC_CTRLBCLR_LUPD_Pos		/* Setting: disabled */
			| 0 << TC_CTRLBSET_DIR_Pos);	/* Counter Direction: disabled */
	hri_tc_write_WAVE_reg(TC1,3);			/* Waveform Generation Mode: 3 - MPWM */
	hri_tccount16_write_CC_reg(TC1, 0, SERVO_PWM_COUNT);	/* Compare/Capture Value: 2273 */
	hri_tccount16_write_CC_reg(TC1, 1, SERVO_PWM_CC1_MIN);	/* Compare/Capture Value: 375 */
	hri_tc_write_CTRLA_ENABLE_bit(TC1, 1 << TC_CTRLA_ENABLE_Pos);	/* Enable: enabled */
}

void SERVO_set_angle(int angle)
{
	double angle_ratio = angle / 180.0;
	uint16_t pwm_spread = SERVO_PWM_MAXIMUM_us - SERVO_PWM_MINIMUM_us;
	double angle_us = SERVO_PWM_MINIMUM_us + (double)(angle_ratio * pwm_spread);
	uint16_t angle_val = (angle_us * SERVO_PWM_COUNT) / SERVO_PWM_PERIOD_us;
	
	if (!hri_tc_is_syncing(TC1, TC_SYNCBUSY_SWRST)) {
		if (hri_tc_get_CTRLA_reg(TC1, TC_CTRLA_ENABLE)) {
			hri_tc_clear_CTRLA_ENABLE_bit(TC1);
			hri_tc_wait_for_sync(TC1, TC_SYNCBUSY_ENABLE);
		}
	}
	hri_tc_wait_for_sync(TC1, TC_SYNCBUSY_SWRST);
	
	hri_tccount16_write_CC_reg(TC1, 1, angle_val);	
	hri_tc_write_CTRLA_ENABLE_bit(TC1, 1 << TC_CTRLA_ENABLE_Pos); 
}

void SERVO_menu(void)
{
	uint32_t user_selection = 0;
	uint32_t servo_angle	= 0;
	
	while (1) {
		printf("%s", servo_menu_txt);
		
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
				printf("\r\nEnter angle >> ");
				scanf("%d", &servo_angle);
				printf("%d", servo_angle);
			
				if (servo_angle < 0 || servo_angle > 180) {
					printf("\r\nERROR: Invalid angle. Angle must be between 0 and 180\r\n");
					break;
				}
			
				printf("\r\nSetting servo angle to %0d\r\n", servo_angle);
				SERVO_set_angle(servo_angle);
				break;
			
			default:
				printf("\r\nInvalid option\r\n");
				break;
		}
	}
}


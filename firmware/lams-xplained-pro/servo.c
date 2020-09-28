#include <atmel_start.h>
#include "servo.h"
#include "driver_init.h"
#include "utils.h"

#define TC1_PWM_PERIOD_us 3030
#define TC1_PWM_MINIMUM_us 500
#define TC1_PWM_MAXIMUM_us 2500

#define TC1_COUNT 2273

uint8_t servo_menu_txt[] = "\r\n******** Enter choice ******** \r\n \
1. Back to main menu\r\n \
2. Set servo angle\r\n";

// Call in driver_init.c
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
	                       2 << TC_CTRLA_CAPTMODE0_Pos       /* Capture mode Channel 0: 0 */
	                           | 2 << TC_CTRLA_CAPTMODE1_Pos /* Capture mode Channel 1: 0 */
	                           | 0 << TC_CTRLA_COPEN0_Pos    /* Capture Pin 0 Enable: disabled */
	                           | 0 << TC_CTRLA_COPEN1_Pos    /* Capture Pin 1 Enable: disabled */
	                           | 0 << TC_CTRLA_CAPTEN0_Pos   /* Capture Channel 0 Enable: enabled */
	                           | 0 << TC_CTRLA_CAPTEN1_Pos   /* Capture Channel 1 Enable: enabled */
	                           | 0 << TC_CTRLA_ALOCK_Pos     /* Auto Lock: disabled */
	                           | 1 << TC_CTRLA_PRESCSYNC_Pos /* Prescaler and Counter Synchronization: 0 */
	                           | 0 << TC_CTRLA_ONDEMAND_Pos  /* Clock On Demand: disabled */
	                           | 0 << TC_CTRLA_RUNSTDBY_Pos  /* Run in Standby: disabled */
	                           | 4 << TC_CTRLA_PRESCALER_Pos /* Setting: 4 */
	                           | 0x0 << TC_CTRLA_MODE_Pos);  /* Operating Mode: 0x0 */

	hri_tc_write_CTRLB_reg(TC1,
	                       0 << TC_CTRLBSET_CMD_Pos           /* Command: 0 */
	                           | 0 << TC_CTRLBSET_ONESHOT_Pos /* One-Shot: disabled */
	                           | 0 << TC_CTRLBCLR_LUPD_Pos    /* Setting: disabled */
	                           | 0 << TC_CTRLBSET_DIR_Pos);   /* Counter Direction: disabled */

	hri_tc_write_WAVE_reg(TC1,3); /* Waveform Generation Mode: 3 */

	// hri_tc_write_DRVCTRL_reg(TC1,0 << TC_DRVCTRL_INVEN1_Pos /* Output Waveform 1 Invert Enable: disabled */
	//		 | 0 << TC_DRVCTRL_INVEN0_Pos); /* Output Waveform 0 Invert Enable: disabled */

	// hri_tc_write_DBGCTRL_reg(TC1,0); /* Run in debug: 0 */

	hri_tccount16_write_CC_reg(TC1, 0, (uint16_t)2273); /* Compare/Capture Value: 375 */

	hri_tccount16_write_CC_reg(TC1, 1, (uint16_t)375); /* Compare/Capture Value: 1874 */

	hri_tccount16_write_COUNT_reg(TC1, (uint16_t)2273); /* Counter Value: 2273 */
	

	// hri_tc_write_EVCTRL_reg(TC1,0 << TC_EVCTRL_MCEO0_Pos /* Match or Capture Channel 0 Event Output Enable: disabled
	// */
	//		 | 0 << TC_EVCTRL_MCEO1_Pos /* Match or Capture Channel 1 Event Output Enable: disabled */
	//		 | 0 << TC_EVCTRL_OVFEO_Pos /* Overflow/Underflow Event Output Enable: disabled */
	//		 | 0 << TC_EVCTRL_TCEI_Pos /* TC Event Input: disabled */
	//		 | 0 << TC_EVCTRL_TCINV_Pos /* TC Inverted Event Input: disabled */
	//		 | 0); /* Event Action: 0 */

	// hri_tc_write_INTEN_reg(TC1,0 << TC_INTENSET_MC0_Pos /* Match or Capture Channel 0 Interrupt Enable: disabled */
	//		 | 0 << TC_INTENSET_MC1_Pos /* Match or Capture Channel 1 Interrupt Enable: disabled */
	//		 | 0 << TC_INTENSET_ERR_Pos /* Error Interrupt Enable: disabled */
	//		 | 0 << TC_INTENSET_OVF_Pos); /* Overflow Interrupt enable: disabled */

	hri_tc_write_CTRLA_ENABLE_bit(TC1, 1 << TC_CTRLA_ENABLE_Pos); /* Enable: enabled */
}

void SERVO_set_angle(int angle)
{
	double angle_ratio = angle / 180.0;
	uint16_t pwm_spread = TC1_PWM_MAXIMUM_us - TC1_PWM_MINIMUM_us;
	double angle_us = TC1_PWM_MINIMUM_us + (double)(angle_ratio * pwm_spread);
	uint16_t angle_val = (angle_us * TC1_COUNT) / TC1_PWM_PERIOD_us;
	
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
					printf("\r\nERROR: Invalid angle. Angle must be between 0 and 180\r\n", 
						   servo_angle);
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


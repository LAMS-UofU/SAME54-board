#include "common.h"
#include "start.h"
#include "eeprom.h"
#include "scan.h"
#include "drivers.h"
#include "servo/servo.h"
#include "lidar/lidar.h"
#include "lams_sd.h"
#include "sd_mmc.h"

volatile uint32_t systick_count;
volatile uint32_t timer;
volatile uint8_t  status;

uint8_t menu_txt[] = "\r\n******** Enter choice ******** \r\n \
1. Reset device\r\n \
2. EEPROM\r\n \
3. Servo\r\n \
4. LiDAR\r\n \
5. LAMS SD\r\n \
6. 360-degree scan\r\n";

/** 
  * Application entry point
  */ 
int main(void)
{
	/* Initializes MCU, drivers and middleware */
	start_init();
	//delay_init(0);
	SysTick_Config(12000UL); /* 12M ticks/seconds / 1k ticks/second = 120000 */
	
	status = STATUS_IDLE;
	
	printf("\r\nLAMS_DEBUG = %u\r\n", LAMS_DEBUG);
	
	gpio_set_pin_level(LED_STATUS, 0);
	SERVO_set_angle(0);

	if (LAMS_DEBUG) {
		printf("\r\n\r\n========LiDAR Automated Mapping System (LAMS)========\r\n");

		while (1) {
			uint32_t user_selection = 0;

			printf("%s", menu_txt);
			
			if (scanf("%"PRIu32"", &user_selection) == 0) {
				/* If its not a number, flush stdin */
				fflush(stdin);
				continue;
			}
			
			printf("\r\nSelected option is %d\r\n", (int)user_selection);
			
			switch (user_selection) {
				case 1:
					NVIC_SystemReset();
					break;
				
				case 2:
					EEPROM_menu();
					break;

				case 3:
					SERVO_menu();
					break;
				
				case 4:
					LIDAR_menu();
					break;
				
				case 5:
					SD_menu();
					break;
				
				case 6:
					scan();
					break;
				
				default:
					printf("\r\nInvalid option \r\n");
					break;
			}
		} 
	} 
    /* LAMS_DEBUG = 0 */
    else {
		while (1) {
			scan();
		}
	}
	
	/* Execution should not come here during normal operation */
    return (EXIT_FAILURE);
}

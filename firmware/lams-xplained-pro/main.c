#include "common.h"
#include "eeprom.h"
#include "scan.h"
#include "servo/servo.h"
#include "lidar/lidar.h"
#include "lams_sd.h"
#include "sd_mmc.h"

volatile uint8_t status;

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
	
	if (SYSTICK_EN)
		SysTick_Config(12000UL); /* 12M ticks/second / 1k ticks/second = 12000 */
	
	status = STATUS_IDLE;
	
	SERVO_set_angle(0);

	if (DEBUG) {
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

	/* Execution should not come here during normal operation */
    return (EXIT_FAILURE);
}

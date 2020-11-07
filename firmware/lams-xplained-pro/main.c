#include "common.h"
#include "smart_eeprom.h"
#include "servo/servo.h"
#include "lidar/lidar.h"

uint8_t menu_txt[] = "\r\n******** Enter choice ******** \r\n \
1. Reset device\r\n \
2. EEPROM\r\n \
3. Servo\r\n \
4. LiDAR\r\n";

/**
  * HardFault Handler
  */
void HardFault_Handler(void)
{
	if (DEBUG)
		printf("\r\n!!!!!!!! In HardFault_Handler !!!!!!!!\r\n");
	while (1);
}

/** 
  * Application entry point
  */ 
int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	EEPROM_init();
	SERVO_set_angle(0);

	if (DEBUG) {
		printf("\r\n\r\n========LiDAR Automated Mapping System (LAMS)========\r\n");

		while (1) {
			uint32_t user_selection = 0;

			printf("%s", menu_txt);
			
			if (scanf("%d", &user_selection) == 0) {
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
				
				default:
					printf("\r\nInvalid option \r\n");
					break;
			}
		}
	}

	/* Execution should not come here during normal operation */
    return (EXIT_FAILURE);
}

#include "common.h"
#include "eeprom.h"
#include "servo/servo.h"
#include "lidar/lidar.h"

uint32_t systick_count = 0;
extern volatile uint8_t lidar_timer;

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
  *	SysTick Handler
  */
void SysTick_Handler(void)
{
	systick_count++;
	if (lidar_timer > 0) {
		if (DEBUG)
			printf("lidar_timer = %u\r\n", lidar_timer);
		lidar_timer--;
	}
}

/** 
  * Application entry point
  */ 
int main(void)
{
	/* Initializes MCU, drivers and middleware */
	start_init();
	if (SYSTICK_EN)
		SysTick_Config(12000UL); /* 12M ticks/second / 1k ticks/second = 12000 */
	EEPROM_init();
	
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
				
				default:
					printf("\r\nInvalid option \r\n");
					break;
			}
		}
	}

	/* Execution should not come here during normal operation */
    return (EXIT_FAILURE);
}

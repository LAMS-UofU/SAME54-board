#include "servo.h"

uint8_t servo_menu_txt[] = "\r\n******** Enter choice ******** \r\n \
1. Back to main menu\r\n \
2. Set servo angle\r\n \
3. Set servo angle (linear transition)\r\n";

/**
  * Menu for servo command options in order to test angle adjustments with
  * servo
  */ 
void SERVO_menu(void)
{
	uint32_t user_selection = 0;
	uint32_t servo_angle	= 0;
	
	while (1) {
		printf("%s", servo_menu_txt);
		
		if (scanf("%"PRIu32"", &user_selection) == 0) {
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
				scanf("%"PRIu32"", &servo_angle);
				printf("%"PRIu32"", servo_angle);
			
				if (servo_angle < 0 || servo_angle > 180) {
					printf("\r\nERROR: Invalid angle. Angle must be between 0 and 180\r\n");
					break;
				}
			
				printf("\r\nSetting servo angle to %0"PRIu32"\r\n", servo_angle);
				SERVO_set_angle(servo_angle);
				break;
			
			case 3:
				printf("\r\nEnter angle >> ");
				scanf("%"PRIu32"", &servo_angle);
				printf("%"PRIu32"", servo_angle);
				
				if (servo_angle < 0 || servo_angle > 180) {
					printf("\r\nERROR: Invalid angle. Angle must be between 0 and 180\r\n");
					break;
				}
				
				printf("\r\nSetting servo angle to %0"PRIu32" with linear transition\r\n", servo_angle);
				SERVO_linear_transition_angle(servo_angle);
				break;
			
			default:
				printf("\r\nInvalid option\r\n");
				break;
		}
	}
}
#include "common.h"
#include "lams_sd.h"
#include "fatfs.h"
#include "ffconf.h"
#include "ff.h"

uint8_t sd_menu_txt[] = "\r\n ******** Enter choice ******** \r\n \
1. Back to main menu\r\n \
2. Write file\r\n";

void SD_menu(void)
{
	uint16_t user_selection = 0;
	TCHAR filename[FILENAME_MAX] = {0};
	char data[512] = {0};
	
	while (1) {
		printf("%s", sd_menu_txt);
		
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
				printf("\r\nEnter filename >> ");
				scanf("%s", filename);
				printf("%s\r\n", filename);
				
				printf("Enter data >> ");
				scanf("%s", data);
				printf("%s\r\n", data);
				
				FATFS_write_file(filename, data, 512);
				break;
		}
	}
}
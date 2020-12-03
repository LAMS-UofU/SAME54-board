#include "common.h"
#include "lams_sd.h"
#include "fatfs.h"
#include "ffconf.h"
#include "ff.h"

extern FRESULT fresult;

uint8_t sd_menu_txt[] = "\r\n ******** Enter choice ******** \r\n \
1. Back to main menu\r\n \
2. Write file\r\n \
3. Get SD status\r\n \
4. Remove file\r\n \
5. Rename file\r\n \
6. Remove empty files\r\n \
7. Fix numbering\r\n \
8. Print file directory\r\n \
9. +100 to file numbering\r\n";

void SD_menu(void)
{
	uint16_t user_selection = 0;
	TCHAR filename[FILENAME_MAX] = {0}, new_filename[FILENAME_MAX] = {0};
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
				
				printf("Enter data (no whitespace) >> ");
				scanf("%s", data);
				printf("%s\r\n", data);
				
				FATFS_write_file(filename, data, 512);
				break;
			
			case 3:
				printf("\r\nRetrieving SD status\r\n");
				FATFS_sd_status();
				break;
			
			case 4:
				printf("\r\nSD directory:\r\n\r\n");
				FATFS_print_files("/");
				if (fresult != FR_OK)
					break;
				
				printf("\r\nEnter filename >> ");
				scanf("%s", filename);
				printf("%s\r\n", filename);
				
				FATFS_remove_file(filename);
				break;
			
			case 5:
				printf("\r\nSD directory:\r\n\r\n");
				FATFS_print_files("/");
				if (fresult != FR_OK)
					break;
				
				printf("\r\nEnter filename >> ");
				scanf("%s", filename);
				printf("%s\r\n", filename);
				
				printf("Enter new filename >> ");
				scanf("%s", new_filename);
				printf("%s\r\n", new_filename);
				
				FATFS_rename_file(filename, new_filename);
				break;
			
			case 6:
				printf("\r\nRemoving empty files:\r\n");
				FATFS_remove_empty_files();
				break;
			
			case 7:
				printf("\r\nFixing numbering of directory:\r\n");
				FATFS_fix_numbering();
				break;
			
			case 8:
				printf("\r\nSD Directory:\r\n\r\n");
				FATFS_print_files("/");
				break;
			
			case 9:
				printf("\r\nAdding 100 to all filenames\r\n");
				FATFS_add_hundred();
				break;
			
			default:
				printf("r\nInvalid option\r\n");
				break;
		}
	}
}
#include "common.h"
#include "fatfs.h"

struct calendar_descriptor CALENDER_INTERFACE;

FATFS fatfs;
FIL	  fptr;

void FATFS_CALENDAR_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_RTC_bit(MCLK);
}

void FATFS_CALENDAR_init(void)
{
	FATFS_CALENDAR_CLOCK_init();
	calendar_init(&CALENDER_INTERFACE, RTC);
}

void FATFS_write_file(TCHAR* filename, char* data, uint32_t data_length)
{
	UINT bytes_written;
	FRESULT file_result;
	
	/* mount SD card */
	f_mount(&fatfs, "/", 0);
	
	/* create file */
	file_result = f_open(&fptr, filename, FA_WRITE | FA_CREATE_ALWAYS);
	
	if (file_result == FR_OK) {
		f_write(&fptr, data, data_length, &bytes_written);
		file_result = f_close(&fptr);
		if (file_result == FR_OK && bytes_written == data_length)
			printf("SUCCESS!\r\n");
	} else {
		printf("Failed to make file. file_result = %u\r\n", file_result);
	}
}

/**
 * Current time returned is packed into a DWORD value.
 *
 * The bit field is as follows:
 *		[31:25]  Year from 1980 (0..127)
 *		[24:21]  Month (1..12)
 *		[20:16]  Day in month(1..31)
 *		[15:11]  Hour (0..23)
 *		[10:5]   Minute (0..59)
 *		[4:0]    Second (0..59)
 */
DWORD get_fattime(void)
{
	uint32_t ul_time;
	struct calendar_date_time datetime;
	
	calendar_get_date_time(&CALENDER_INTERFACE, &datetime);

	ul_time = ((datetime.date.year - 1980) << 25) | (datetime.date.month << 21) | (datetime.date.day << 16)
			 | (datetime.time.hour << 11) | (datetime.time.min << 5) | (datetime.time.sec << 0);
	return ul_time;
}




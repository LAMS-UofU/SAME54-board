#include "common.h"
#include "fatfs.h"
#include "diskio.h"

static char* file_result_error(void);

struct calendar_descriptor CALENDER_INTERFACE;

static FATFS   fatfs;
static FIL	   fptr;
static FILINFO finfo;
static FRESULT fresult;
static DIR	   fdir;

void FATFS_CALENDAR_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_RTC_bit(MCLK);
}

void FATFS_CALENDAR_init(void)
{
	FATFS_CALENDAR_CLOCK_init();
	calendar_init(&CALENDER_INTERFACE, RTC);
}

/**
  * result:
  *		(0) Succeeded 
  *		(1) A hard error occurred in the low level disk I/O layer
  *		(2) Assertion failed
  *		(3) The physical drive cannot work 
  *		(4) Could not find the file
  *		(5) Could not find the path
  *		(6) The path name format is invalid 
  *		(7) Access denied due to prohibited access or directory full 
  *		(8) Access denied due to prohibited access 
  *		(9) The file/directory object is invalid 
  *		(10) The physical drive is write protected
  *		(11) The logical drive number is invalid 
  *		(12) The volume has no work area 
  *		(13) There is no valid FAT volume 
  *		(14) The f_mkfs() aborted due to any parameter error 
  *		(15) Could not get a grant to access the volume within defined period 
  *		(16) The operation is rejected according to the file sharing policy 
  *		(17) LFN working buffer could not be allocated 
  *		(18) Number of open files > _FS_LOCK 
  *		(19) Given parameter is invalid
  * status:
  *		(0) Good
  *		(1) Drive not initialized
  *		(2) No medium in the drive
  *		(3) Write protected
  */
void FATFS_sd_status(void)
{
	DSTATUS status;
	FRESULT result;
	
	status = disk_initialize(0);
	printf("\r\nInit result = %u\r\n", status);
	
	result = f_mount(&fatfs, "", 0);
	printf("Mount result = %u\r\n", result);
	
	status = disk_status(0);
	printf("Disk status = %u\r\n", status);
}

void FATFS_write_file(TCHAR* filename, char* data, uint32_t data_length)
{
	UINT bytes_written;
	FRESULT result;
	DSTATUS status;
	
	/* initialize SD card connection */
	status = disk_initialize(0);
	printf("\r\nInit result = %u\r\n", status);
	
	/* mount SD card */
	result = f_mount(&fatfs, "", 0);
	printf("Mount result = %u\r\n", result);
	
	/* create file */
	result = f_open(&fptr, filename, FA_WRITE | FA_CREATE_ALWAYS);
	
	if (result == FR_OK) {
		f_write(&fptr, data, data_length, &bytes_written);
		result = f_close(&fptr);
		if (result == FR_OK && bytes_written == data_length)
			printf("SUCCESS!\r\n");
	} else {
		printf("Failed to make file. file_result = %u\r\n", result);
	}
}

void FATFS_print_files(char* path)
{
	long p1;
	UINT s1, s2;
	FATFS *fs = &fatfs;
	
	fresult = f_opendir(&fdir, path);
	if (fresult) {
		printf("%s\r\n", file_result_error()); 
		return; 
	}
	
	p1 = s1 = s2 = 0;
	
	while(1) {
		fresult = f_readdir(&fdir, &finfo);
		if ((fresult != FR_OK) || !finfo.fname[0]) 
			break;
		
		if (finfo.fattrib & AM_DIR)
			s2++;
		else { 
			s1++; 
			p1 += finfo.fsize;
		}
		
		printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s\r\n",
			  (finfo.fattrib & AM_DIR) ? 'D' : '-',
			  (finfo.fattrib & AM_RDO) ? 'R' : '-',
			  (finfo.fattrib & AM_HID) ? 'H' : '-',
			  (finfo.fattrib & AM_SYS) ? 'S' : '-',
			  (finfo.fattrib & AM_ARC) ? 'A' : '-',
			  (finfo.fdate >> 9) + 1980, 
			  (finfo.fdate >> 5) & 15, 
			  finfo.fdate & 31,
			  (finfo.ftime >> 11), 
			  (finfo.ftime >> 5) & 63,
			  finfo.fsize, 
			  finfo.fname
		);
	}
	
	printf("%4u File(s),%10lu bytes total\n%4u Dir(s)", s1, p1, s2);
	fresult = f_getfree(path, (DWORD*)&p1, &fs);
	if (fresult == FR_OK)
		printf(", %10lu bytes free\r\n", p1 * fs->csize * 512);
	else
		printf("%s\r\n", file_result_error());
}

/**
  *	Return string based on file result
  */
char* file_result_error(void)
{
	switch (fresult) {
		case FR_DISK_ERR:			 return "Hard error occurred in the low level disk I/O layer";
		case FR_INT_ERR:			 return "Assertion failed";
		case FR_NOT_READY:			 return "Physical drive cannot work";
		case FR_NO_FILE:			 return "Could not find file";
		case FR_NO_PATH:			 return "Could not find path";
		case FR_INVALID_NAME:		 return "Path name format invalid";
		case FR_DENIED:				 return "Access denied due to prohibited access or directory full";
		case FR_EXIST:				 return "Access denied due to prohibited access";
		case FR_INVALID_OBJECT:		 return "The file/directory object is invalid";
		case FR_WRITE_PROTECTED:	 return "The physical drive is write protected";
		case FR_INVALID_DRIVE:		 return "The logical drive number is invalid";
		case FR_NOT_ENABLED:		 return "The volume has no work area";
		case FR_NO_FILESYSTEM:		 return "There is no valid FAT volume";
		case FR_MKFS_ABORTED:		 return "The f_mkfs() aborted due to any parameter error";
		case FR_TIMEOUT:			 return "Could not get a grant to access the volume within defined period";
		case FR_LOCKED:				 return "The operation is rejected according to the file sharing policy";
		case FR_NOT_ENOUGH_CORE:	 return "LFN working buffer could not be allocated";
		case FR_TOO_MANY_OPEN_FILES: return "Number of open files > _FS_LOCK";
		case FR_INVALID_PARAMETER:	 return "Given parameter is invalid";
		default:					 return "Disk write ok...should not have reached here";
	}
	return 0;
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




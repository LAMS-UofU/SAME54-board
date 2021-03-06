#include "common.h"
#include "fatfs.h"
#include "diskio.h"

struct calendar_descriptor CALENDER_INTERFACE;

FATFS   fatfs;
FIL     fptr;
DSTATUS dstatus;
FRESULT fresult;
UINT    bwritten;
FILINFO finfo;
DIR		fdir;

/**
  *	Initializes calendar clock for FATFS
  */ 
void FATFS_CALENDAR_CLOCK_init(void)
{
	hri_mclk_set_APBAMASK_RTC_bit(MCLK);
}

/**
  *	Initializes calendar for FATFS
  */ 
void FATFS_CALENDAR_init(void)
{
	FATFS_CALENDAR_CLOCK_init();
	calendar_init(&CALENDER_INTERFACE, RTC);
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

/**
  *	Get SD status (disk 0)
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

/**
  *	Write file given data
  */ 
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
	
	/* close file */
	fresult = f_close(&fptr);
	
	/* unmount SD */
	f_mount(0, "", 0);
}

/**
  *	Print files and directories from path
  */ 
void FATFS_print_files(char* path)
{
	long p1;
	UINT s1, s2;
	FATFS *fs = &fatfs;
	
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus != RES_OK) {
		printf("%s\r\n", DISKIO_dstatus_desc(dstatus));
		return;
	}
	
	/* mount SD card */
	fresult = f_mount(&fatfs, path, 0);
	if (fresult != FR_OK) {
		printf("%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	fresult = f_opendir(&fdir, path);
	if (fresult) {
		printf("%s\r\n", FATFS_fresult_desc(fresult)); 
		return; 
	}
	
	printf("\tD - Directory\r\n");
	printf("\tR - Read-Only\r\n");
	printf("\tH - Hidden\r\n");
	printf("\tS - System\r\n");
	printf("\tA - Archive\r\n");
	
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
		printf("%s\r\n", FATFS_fresult_desc(fresult));
	
	/* unmount SD */
	f_mount(0, "", 0);
}

void FATFS_remove_file(TCHAR* fname)
{
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus != RES_OK) {
		printf("\r\n%s\r\n", DISKIO_dstatus_desc(dstatus));
		return;
	}
	
	/* mount SD card */
	fresult = f_mount(&fatfs, "/", 0);
	if (fresult != FR_OK) {
		printf("\r\n%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	fresult = f_unlink(fname);
	if (fresult == FR_OK)
		printf("Successfully removed %s\r\n", fname);
	else
		printf("\r\nCould not removed %s: %s\r\n", fname, FATFS_fresult_desc(fresult));
	
	/* unmount SD */
	f_mount(0, "", 0);
}

void FATFS_rename_file(TCHAR* fname, TCHAR* new_fname)
{
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus != RES_OK) {
		printf("\r\n%s\r\n", DISKIO_dstatus_desc(dstatus));
		return;
	}
	
	/* mount SD card */
	fresult = f_mount(&fatfs, "/", 0);
	if (fresult != FR_OK) {
		printf("%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	fresult = f_rename(fname, new_fname);
	if (fresult == FR_OK)
		printf("Successfully renamed %s to %s\r\n", fname, new_fname);
	else
		printf("\r\nCould not rename %s: %s\r\n", fname, FATFS_fresult_desc(fresult));
	
	/* unmount SD */
	f_mount(0, "", 0);
}

/**
  *	Delete empty files -- up to 100 before breaks
  */
void FATFS_remove_empty_files(void)
{
	TCHAR empty_files[100][13];
	int i, empty_num = 0;
	
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus != RES_OK) {
		printf("\r\n%s\r\n", DISKIO_dstatus_desc(dstatus));
		return;
	}
	
	/* mount SD card */
	fresult = f_mount(&fatfs, "/", 0);
	if (fresult != FR_OK) {
		printf("\r\n%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	fresult = f_opendir(&fdir, "/");
	if (fresult) {
		printf("%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	while(1) {
		fresult = f_readdir(&fdir, &finfo);
		if ((fresult != FR_OK) || !finfo.fname[0])
			break;
		
		if ( (finfo.fsize == 0) && (!(finfo.fattrib & AM_SYS)) ) {
			for (i = 0; i < 12; i++)
				empty_files[empty_num][i] = finfo.fname[i];
			empty_num++;
		}
	}
	
	for(i = 0; i < empty_num; i++)
		FATFS_remove_file(empty_files[i]);
	
	/* unmount SD */
	f_mount(0, "", 0);
}

/**
  * fix numbering for 100 files
  */
void FATFS_fix_numbering(void)
{
	int i, filenum = 0;
	TCHAR filename[12];
	TCHAR files[100][13];
	
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus != RES_OK) {
		printf("\r\n%s\r\n", DISKIO_dstatus_desc(dstatus));
		return;
	}
	
	/* mount SD card */
	fresult = f_mount(&fatfs, "/", 0);
	if (fresult != FR_OK) {
		printf("\r\n%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	fresult = f_opendir(&fdir, "/");
	if (fresult) {
		printf("%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	while(1) {
		fresult = f_readdir(&fdir, &finfo);
		if ((fresult != FR_OK) || !finfo.fname[0])
			break;
		
		/* get filenames */
		if (!(finfo.fattrib & AM_SYS)) { 
			for (i = 0; i < 13; i++)
				files[filenum][i] = finfo.fname[i];
			filenum++;
		}
	}
	
	/* rename file to +1 of previous filenum */
	for (i = 0; i < filenum; i++) {
		sprintf(filename, "SCAN%03u.LAM", i + 1);
		FATFS_rename_file(files[i], filename);	
	}
	
	/* unmount SD */
	f_mount(0, "", 0);
}

void FATFS_add_hundred(void)
{
	int i, filenum = 0;
	TCHAR filename[12];
	TCHAR files[100][13];
	
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus != RES_OK) {
		printf("\r\n%s\r\n", DISKIO_dstatus_desc(dstatus));
		return;
	}
	
	/* mount SD card */
	fresult = f_mount(&fatfs, "/", 0);
	if (fresult != FR_OK) {
		printf("\r\n%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	fresult = f_opendir(&fdir, "/");
	if (fresult) {
		printf("%s\r\n", FATFS_fresult_desc(fresult));
		return;
	}
	
	while(1) {
		fresult = f_readdir(&fdir, &finfo);
		if ((fresult != FR_OK) || !finfo.fname[0])
		break;
		
		/* get filenames */
		if (!(finfo.fattrib & AM_SYS)) {
			for (i = 0; i < 13; i++)
			files[filenum][i] = finfo.fname[i];
			filenum++;
		}
	}
	
	/* rename file to +1 of previous filenum */
	for (i = 0; i < filenum; i++) {
		sprintf(filename, "SCAN%03u.LAM", i + 101);
		FATFS_rename_file(files[i], filename);
	}
	
	/* unmount SD */
	f_mount(0, "", 0);
}


/**
  *	Return string based on file result
  */
char* FATFS_fresult_desc(FRESULT res)
{
	switch (res) {
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
		default:					 return "Success";
	}
	return 0;
}

/**
  *	Return string based on disk status
  */
char* DISKIO_dstatus_desc(DSTATUS status)
{
	switch (status) {
		case STA_NOINIT:  return "Disk not initialized";
		case STA_NODISK:  return "No medium in the drive";
		case STA_PROTECT: return "Disk write protected";
	}
	return 0;
}

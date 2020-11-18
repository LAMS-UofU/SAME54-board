#include "scan.h"
#include "common.h"
#include "servo/servo.h"
#include "lidar/lidar.h"
#include "fatfs.h"
#include "ff.h"
#include "diskio.h"
#include <string.h>
#include <stdio.h>

#define PROCESSING 0
#define COMPLETED  1

#define SCAN_ERR_TIMEOUT		0
#define SCAN_ERR_OUT_OF_BOUNDS	1
#define SCAN_ERR_DISK_INIT		2
#define SCAN_ERR_DISK_MOUNT		3
#define SCAN_ERR_FILE_CREATE	4
#define SCAN_ERR_FILE_WRITE		5
#define SCAN_ERR_FILE_CLOSE		6
#define SCAN_ERR_NEW_FILENAME	7

static void scan_error(uint16_t);
static uint8_t process(void);
static void process_cabins(void);
static char* disk_status_error(void);
static char* file_result_error(void);
static char* get_new_filename(void);

extern volatile uint8_t  status;
extern volatile uint32_t start_time;

/* LIDAR variables */
extern response_descriptor resp_desc;
extern cabin_data		   cabins[MAX_SCANS];
extern uint32_t			   scan_count;
extern uint8_t			   lidar_request;
extern volatile uint32_t   byte_count;
extern volatile uint8_t	   lidar_timing;
extern volatile char	   DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

/* Servo variables */
static int angle;

/* Disk variables */
static FATFS   fatfs;
static FIL	   fptr;
static DSTATUS dstatus;
static FRESULT fresult;
static UINT	   bwritten;
static FILINFO finfo;

static char filename[12];

void scan(void)
{
	uint16_t error_code;
	char* info;
	uint8_t count = 0;
	uint32_t data_length = 0;
	TCHAR *path = "";
	
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus)
		scan_error(SCAN_ERR_DISK_INIT);
	
	/* mount SD card */
	fresult = f_mount(&fatfs, path, 0);
	if (fresult)
		scan_error(SCAN_ERR_DISK_MOUNT);
	
	/* Wait until button pressed */
	if (DEBUG)
		printf("\r\nPress button to start\r\n");
	while (!gpio_get_pin_level(START_BTN));
	
	/* create file */
	fresult = f_open(&fptr, get_new_filename(), FA_WRITE | FA_CREATE_ALWAYS);
	if (fresult)
		scan_error(SCAN_ERR_FILE_CREATE);
	
	/* start scan */
	if (DEBUG)
		printf("\r\nStarting scan\r\n");
	
	/* reset lidar */
	LIDAR_REQ_reset();
	while (!process()) {
		LIDAR_RES_reset();
	}
	
	/* get lidar health */
	LIDAR_REQ_get_health();
	while (!process());
	error_code = LIDAR_RES_get_health();
	if (error_code)
		scan_error(error_code);
	
	/* get lidar info */
	LIDAR_REQ_get_info();
	while (!process());
	info = LIDAR_RES_get_info();
	if (DEBUG)
		printf(info);
	while (count != 4) 		/* 4 '\n' in info */
		if (info[data_length++] == '\n')
			count++;
	fresult = f_write(&fptr, info, data_length, &bwritten);
	if (!fresult && bwritten != data_length)
		scan_error(SCAN_ERR_FILE_WRITE);
	
	/* start scans */
	LIDAR_PWM_start();
	LIDAR_REQ_express_scan();
	for (angle = 0; angle <= 20; angle++) {
		SERVO_set_angle(angle);
		while (scan_count < MAX_SCANS) {
			while (!process());
			LIDAR_RES_express_scan();
		}
		process_cabins();
	}
	
	/* all scans complete, stop processes */
	LIDAR_PWM_stop();
	LIDAR_REQ_stop();
	while (!process()) {
		LIDAR_RES_stop();
	}
	
	/* close file */
	fresult = f_close(&fptr);
	if (fresult)
		scan_error(SCAN_ERR_FILE_CLOSE);
	
	/* unmount SD */
	f_mount(0, "", 0);
	
	status = STATUS_IDLE;
}

/**
  *
  */
uint8_t process(void)
{
	unsigned data_idx;
	
	status = STATUS_PROCESSING; 
	
	/* STOP and RESET requests */
	if (lidar_request == LIDAR_STOP || 
		lidar_request == LIDAR_RESET) {
		if (lidar_timing) 
			return PROCESSING;
		else
			return COMPLETED;
	}
	
	/* Check for timeout -- wait 1 second */
	//if (systick_count >= start_time + 1000) {
		//scan_error(0);
	//}
	
	if (!usart_sync_is_rx_not_empty(&LIDAR_USART))
		return PROCESSING;
	
	/* Process response descriptor */
	switch (byte_count) {
		case 0:
			resp_desc.start1 = LIDAR_USART_read_byte();
			/* check sync -- 0xA5 */
			if (resp_desc.start1 == 0xA5)
				byte_count++;
			return PROCESSING;
		
		case 1:
			resp_desc.start2 = LIDAR_USART_read_byte();
			/* check sync -- 0x5A */
			if (resp_desc.start2 != 0x5A)
				byte_count--;
			else
				byte_count++;
			return PROCESSING;
		
		case 2:
			resp_desc.response_info = LIDAR_USART_read_byte();
			byte_count++;
			return PROCESSING;
		
		case 3:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 8);
			byte_count++;
			return PROCESSING;
		
		case 4:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 16);
			byte_count++;
			return PROCESSING;
		
		case 5:
			resp_desc.response_info |= (LIDAR_USART_read_byte() << 24);
			byte_count++;
			return PROCESSING;
		
		case 6:
			resp_desc.data_type = LIDAR_USART_read_byte();
			byte_count++;
			return PROCESSING;
		
		/* Process response data packets */
		default:
			data_idx = byte_count - LIDAR_RESP_DESC_SIZE;
			DATA_RESPONSE[data_idx] = LIDAR_USART_read_byte();
			
			if (lidar_request == LIDAR_EXPRESS_SCAN) {
				/* check sync -- 0xA */
				if (data_idx == 0) {
					if ((DATA_RESPONSE[data_idx] >> 4) != 0x0A )
						return PROCESSING;
				}
				/* check sync -- 0x5 */
				else if (data_idx == 1) {
					if ((DATA_RESPONSE[data_idx] >> 4) != 0x05) {
						/* decrement and wait for another 0xA to come back in sync */
						byte_count--;
						return PROCESSING;
					}
				}
			}
			byte_count++;
	};
	
	if (byte_count == (resp_desc.response_info & 0x3FFFFFFF) + LIDAR_RESP_DESC_SIZE) {
		return COMPLETED;
	} else if (byte_count > (resp_desc.response_info & 0x3FFFFFFF) + LIDAR_RESP_DESC_SIZE) {
		scan_error(1);
	}
	
	return PROCESSING;
}

/**
  *	Process cabin data and send to SD card
  */
void process_cabins(void)
{
	int i, data_length;
	char data[68];
	
	for (i=0; i<MAX_SCANS; i++) {
		sprintf(data, 
				"%u,%u,%u,%u,%u,%u,%u\r\n",
				angle,					/* Max 3 ASCII	*/
				cabins[i].S,			/* Max 1		*/
				cabins[i].start_angle,	/* Max 15		*/
				cabins[i].angle_value1,	/* Max 5		*/
				cabins[i].angle_value2,	/* Max 5		*/
				cabins[i].distance1,	/* Max 15		*/
				cabins[i].distance2		/* Max 15		*/
				);						/* single_scan max 67 ASCII */
		data_length = 0;
		while(data[data_length++] != '\n');
		fresult = f_write(&fptr, data, data_length, &bwritten);
		if (!fresult && bwritten != data_length)
			scan_error(SCAN_ERR_FILE_WRITE);
	}
}

/**
  *
  */
void scan_error(uint16_t error_code)
{
	printf("\r\n[Scan Error]\r\n");

	LIDAR_PWM_stop();

	status = STATUS_ERROR;

	switch (error_code) {
		
		case SCAN_ERR_TIMEOUT: 
			printf(" | Timeout Error\r\n");
			break;
		
		case SCAN_ERR_OUT_OF_BOUNDS:
			printf(" | Out-of-Bounds Error\r\n");
			printf(" | | byte count (%"PRIu32") went past the response descriptor limit (%lu)\r\n",
					byte_count, (resp_desc.response_info & 0x3FFFFFFF) + LIDAR_RESP_DESC_SIZE);
			break;
		
		case SCAN_ERR_DISK_INIT:
			printf(" | Disk Initialization Error\r\n");
			printf(" | | %s\r\n", disk_status_error());
			break;
		
		case SCAN_ERR_DISK_MOUNT:
			printf(" | Disk Mount Error\r\n");
			printf(" | | %s\r\n", file_result_error());
			break;
		
		case SCAN_ERR_FILE_CREATE:
			printf(" | File Create Error\r\n");
			printf(" | | %s\r\n", file_result_error());
			break;
		
		case SCAN_ERR_FILE_WRITE:
			printf(" | File Write Error\r\n");
			printf(" | | %s\r\n", file_result_error());
			break;
			
		case SCAN_ERR_FILE_CLOSE:
			printf(" | File Close Error\r\n");
			printf(" | | %s\r\n", file_result_error());
			break;
			
		case SCAN_ERR_NEW_FILENAME:
			printf(" | Error Generating New Filename\r\n");
			printf(" | | %s\r\n", file_result_error());
			break;
		
		default:
			printf(" | Error code %u\r\n", error_code);
	};

	while (1);
}

/**
  *	Return string based on disk status
  */
char* disk_status_error(void)
{
	switch (dstatus) {
		case STA_NOINIT:  return "Disk not initialized";
		case STA_NODISK:  return "No medium in the drive";
		case STA_PROTECT: return "Disk write protected";
	}
	return 0;
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
  *	Generates new filename for scans. Format of scan filename is:
  *
  *		scan###.lam
  * 
  * Note: This assumes that the SD is already mounted and initialized as it
  *		  is called within the scan function
  */
TCHAR* get_new_filename(void)
{
	char *path = "";
	TCHAR lfilenum[3];
	DIR fdir;
	int i, filenum, highfilenum = 0;
	
	fresult = f_opendir(&fdir, path);
	if (fresult)
		scan_error(SCAN_ERR_NEW_FILENAME);
	
	while (1) {
		fresult = f_readdir(&fdir, &finfo);
		if (fresult || !finfo.fname[0])
			break;
			
		/* shift right 4 to cut of "scan" */
		for (i=0; i<3; i++)
			lfilenum[i] = finfo.fname[i+4];
		filenum = atoi((char*)&lfilenum);
		
		/* check if higher for highest scan number */
		if (filenum > highfilenum) 
			highfilenum = filenum;	
	}
	
	sprintf(filename, "scan%03u.lam", highfilenum + 1);
	if (DEBUG) printf("\r\nWriting to %s\r\n", filename);
	
	return (TCHAR*)filename;
}
#include "scan.h"
#include "common.h"
#include "servo/servo.h"
#include "lidar/lidar.h"
#include "fatfs.h"
#include "lams_sd.h"
#include "drivers.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static uint8_t process(void);
static void write_averaged_cabins(void);
static void average_cabins(uint8_t);
static void scan_error(uint16_t);
static char* get_new_filename(void);
static int set_data_format(express_mode_t);
static void format_header_file_data(void);
static void write_print_buffer(UINT);
static void reset_print_buffer(void);

#define PROCESSING 0
#define COMPLETED  1

#define SCAN_ERR_TIMEOUT	        0
#define SCAN_ERR_OUT_OF_BOUNDS	    1
#define SCAN_ERR_DISK_INIT		    2
#define SCAN_ERR_DISK_MOUNT		    3
#define SCAN_ERR_FILE_CREATE	    4
#define SCAN_ERR_FILE_WRITE		    5
#define SCAN_ERR_FILE_CLOSE		    6
#define SCAN_ERR_NEW_FILENAME	    7
#define SCAN_ERR_FILE_FORMATTING    8

/* define scan mode */
#define SCAN_MODE           (scan_mode_t)    SCAN_MODE_EXPRESS
#define EXPRESS_SCAN_MODE   (express_mode_t) EXPRESS_SCAN_STANDARD

static char filename[12];   // filename format SCAN###.lam
static char print_buffer[MAX_PRINT_BUFFER_SIZE];

extern volatile uint8_t status;

/* LIDAR variables - see lidar.c for init */
extern write_data_s		sd_scan_data[MAX_SCANS];
extern resp_desc_s		resp_desc;
extern conf_data_t		conf_data;
extern uint32_t			scan_count;
extern uint32_t			invalid_exp_scans;
extern uint32_t			byte_count;
extern uint8_t			lidar_request;
extern volatile char	DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

/* LIDAR responses - see responses.c for init */
extern legacy_cabin_data_s  legacy_cabins[LEGACY_CABIN_COUNT];
extern ultra_cabin_data_s   ultra_cabins[EXT_CABIN_COUNT];
extern dense_cabin_data_s   dense_cabins[DENSE_CABIN_COUNT];

/* local servo variables */
static uint16_t angle;

/* local disk variables */
static FATFS   fatfs;
static FIL     fptr;
static DSTATUS dstatus;
static FRESULT fresult;
static UINT    bwritten;
static FILINFO finfo;

/**
  * Scanning method for main application
  */ 
 void scan(void)
 {
    uint16_t error_code;
	char* info;
	TCHAR *path = "";
	conf_data_t conf_count;
	int i;
	
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus)
		scan_error(SCAN_ERR_DISK_INIT);
	
	/* mount SD card */
	fresult = f_mount(&fatfs, path, 0);
	if (fresult)
		scan_error(SCAN_ERR_DISK_MOUNT);
	
	/* Wait until button pressed */
	//if (DEBUG)
		//printf("\r\nPress button to start\r\n");
	//while (!gpio_get_pin_level(START_BTN));
	
	/* create file */
	fresult = f_open(&fptr, get_new_filename(), FA_READ | FA_WRITE | FA_CREATE_NEW);
	if (fresult)
		scan_error(SCAN_ERR_FILE_CREATE);
	
	/* start scan */
	if (DEBUG)
		printf("\r\nStarting scan\r\n");
	
	/* reset lidar */
	LIDAR_REQ_reset();
	delay_ms(500);
	
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
	write_print_buffer(sprintf(print_buffer, "%s", info));
	
	/* place scan mode in file */
	LIDAR_REQ_get_lidar_conf(SCAN_MODE, CONF_SCAN_MODE_NAME);
	while (!process());
	LIDAR_RES_get_lidar_conf();
	write_print_buffer(sprintf(print_buffer, "# Scan mode is \"%s\"\r\n", conf_data.resp3));
	
	/* get system available scan count */
	LIDAR_REQ_get_lidar_conf(SCAN_MODE, CONF_SCAN_MODE_COUNT);
	while (!process());
	LIDAR_RES_get_lidar_conf();
	conf_count.resp1 = conf_data.resp1;
	
	/* place alternative scan mode names in file */
	write_print_buffer(sprintf(print_buffer, "# Alternative modes: ["));
	for (i = 0; i < conf_count.resp1; i++) {
		LIDAR_REQ_get_lidar_conf(i, CONF_SCAN_MODE_NAME);
		while (!process());
		LIDAR_RES_get_lidar_conf();
		write_print_buffer(sprintf(print_buffer, "\"%s\"", conf_data.resp3));
		if (i < (conf_count.resp1 - 1))
			write_print_buffer(sprintf(print_buffer, ","));
	}
	write_print_buffer(sprintf(print_buffer, "]\r\n"));
	
	/* place scan speed in file */
	LIDAR_REQ_get_lidar_conf(SCAN_MODE, CONF_SCAN_MODE_US_PER_SAMPLE);
	while (!process());
	LIDAR_RES_get_lidar_conf();
	write_print_buffer(sprintf(print_buffer, "# Scan costs %"PRIu32"us per sample\r\n", conf_data.resp2));
	
	/* place scan max distance in file */
	LIDAR_REQ_get_lidar_conf(SCAN_MODE, CONF_SCAN_MODE_MAX_DISTANCE);
	while (!process());
	LIDAR_RES_get_lidar_conf();
	write_print_buffer(sprintf(print_buffer, "# Max measuring distance is %"PRIu32"m\r\n", conf_data.resp2));
	
	/* place data format in file */
	write_print_buffer(set_data_format(EXPRESS_SCAN_MODE));
	
	/* format header data to look better */
	format_header_file_data();
	
	/* start scans */
	LIDAR_PWM_start();
	for (angle = 0; angle <= 180; angle+=2) {
		SERVO_set_angle(angle);
		delay_ms(100);
		if (DEBUG)
			printf("\r\nServo angle %u\r\n", angle);
		LIDAR_REQ_express_scan(EXPRESS_SCAN_MODE);
		while (scan_count < MAX_SCANS) {
			while (!process());
			if (LIDAR_RES_express_scan()) {
				average_cabins(angle);
			} 
		}
		write_averaged_cabins();
		scan_count = 0;
	}
	
	/* all scans complete, stop processes */
	LIDAR_PWM_stop();
	LIDAR_REQ_stop();
	delay_ms(50);
	
	/* close file */
	fresult = f_close(&fptr);
	if (fresult)
		scan_error(SCAN_ERR_FILE_CLOSE);
	
	/* unmount SD */
	f_mount(0, "", 0);
	
	status = STATUS_IDLE;
 }

/**
  * Process bytes coming from lidar in order to read valid responses
  */ 
 uint8_t process(void)
 {
     unsigned data_idx;

     status = STATUS_PROCESSING;

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

void write_averaged_cabins(void)
{
	int i, j;

	printf("\r\nAveraged Scan Data:\r\n");
	
	for (i = 0; i < MAX_SCANS; i++)
	{
		if (DEBUG) {
			printf("\t%u,%s,%s\t\t(repeated %u times)\r\n",
					sd_scan_data[i].servo_angle,
					sd_scan_data[i].distance,
					sd_scan_data[i].lidar_angle,
					sd_scan_data[i].repeated_n);
		}
		write_print_buffer (
			sprintf(print_buffer,
					"%u,%s,%s\r\n",
					sd_scan_data[i].servo_angle,
					sd_scan_data[i].distance,
					sd_scan_data[i].lidar_angle
			)
		);
	}
	
	/* clear written scans */
	for (i = 0; i < MAX_SCANS; i++) {
		sd_scan_data[i].repeated_n = 0;
		sd_scan_data[i].lidar_angle_int = 0;
		sd_scan_data[i].lidar_angle_precision = 0;
		sd_scan_data[i].servo_angle = 0;
		for (j = 0; j < 32; j++) {
			sd_scan_data[i].distance[j] = '\0';		
			sd_scan_data[i].lidar_angle[j] = '\0';
		}
	}
}

/**
  * Average processed scan cabins and write to SD
  */
void average_cabins(uint8_t servo_angle)
{
    int i, repeated_idx = scan_count;
	double distance, angle;
	char tmp_char[32];
	uint16_t distance_int, distance_precision;
	uint16_t angle_int, angle_precision;
	uint16_t precision = 6 + 1; // 6 decimal places (+1 for decimal)
	
	switch (resp_desc.data_type) {
		
		/* Legacy Version */
		case EXPRESS_SCAN_LEGACY_VER:
			sd_scan_data[scan_count].servo_angle = servo_angle;
			
			angle = legacy_cabins[0].start_angle / 64.0;
			angle_int = (uint16_t) angle;
			angle_precision = (uint16_t)((angle - angle_int) * pow(10, precision));
			
			sprintf(sd_scan_data[scan_count].lidar_angle,
					"%u.%u",
					angle_int,
					angle_precision
			);
			sd_scan_data[scan_count].lidar_angle_int = angle_int;
			sd_scan_data[scan_count].lidar_angle_precision = angle_precision;
			
			/* check for repeated lidar angles */
			for (i = 0; i < scan_count; i++) {
				sprintf(tmp_char, "%u.%u", sd_scan_data[i].lidar_angle_int, sd_scan_data[i].lidar_angle_precision);
				if (!strcmp(tmp_char, sd_scan_data[scan_count].lidar_angle)) {
					sd_scan_data[i].repeated_n++;
					repeated_idx = i;
					break;
				}
			}
			
			/* average scans */
			distance = 0;
			for (i = 0; i < LEGACY_CABIN_COUNT; i++) {
				/* sample 1 */
				angle = (legacy_cabins[i].angle_value1 & 0x0F) / 8.0;
				if (legacy_cabins[i].distance1 > 0)
					distance += sqrt(pow((double)legacy_cabins[i].distance1, 2.0) + 
									 pow(angle, 2.0) );
				/* sample 2 */
				angle = (legacy_cabins[i].angle_value2 & 0x0F) / 8;
				if (legacy_cabins[i].distance2 > 0)
					distance += sqrt(pow((double)legacy_cabins[i].distance2, 2.0) +
									 pow(angle, 2.0) );
			}
			
			distance /= (LEGACY_CABIN_COUNT * 2);
			
			/* account for repeated angles */
			if (sd_scan_data[repeated_idx].repeated_n > 0) {
				distance += strtod(sd_scan_data[scan_count].distance, NULL);
				distance /= 2.0;
				
				distance_int = (uint16_t)distance;
				distance_precision = (uint16_t)((distance - distance_int) * pow(10, precision));
				
				sprintf(sd_scan_data[repeated_idx].distance,
						"%u.%u",
						distance_int,
						distance_precision
				);
				break;
			}
			
			distance_int = (uint16_t)distance;
			distance_precision = (uint16_t)((distance - distance_int) * pow(10, precision));
			
			sprintf(sd_scan_data[scan_count].distance, 
					"%u.%u",
					distance_int, 
					distance_precision
			);
			
			scan_count++;
			break;
		
		/* Extended Version */
		//case EXPRESS_SCAN_EXTENDED_VER:
		//
		///* Denses Version */
		//case EXPRESS_SCAN_DENSE_VER:
	}
}

/**
  * Print any info based on the type of scan error and keep in infinite loop
  * while blinking light to symbolize error
  */ 
void scan_error(uint16_t error_code)
{
    if (DEBUG)
		printf("\r\n[Scan Error]\r\n");

	LIDAR_PWM_stop();

	status = STATUS_ERROR;
	
	if (DEBUG) {
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
				printf(" | | %s\r\n", FATFS_dstatus_desc(dstatus));
				break;
			case SCAN_ERR_DISK_MOUNT:
				printf(" | Disk Mount Error\r\n");
				printf(" | | %s\r\n", FATFS_fresult_desc(fresult));
				break;
			case SCAN_ERR_FILE_CREATE:
				printf(" | File Create Error\r\n");
				printf(" | | %s\r\n", FATFS_fresult_desc(fresult));
				break;
			case SCAN_ERR_FILE_WRITE:
				printf(" | File Write Error\r\n");
				printf(" | | %s\r\n", FATFS_fresult_desc(fresult));
				break;
			case SCAN_ERR_FILE_CLOSE:
				printf(" | File Close Error\r\n");
				printf(" | | %s\r\n", FATFS_fresult_desc(fresult));
				break;
			case SCAN_ERR_NEW_FILENAME:
				printf(" | Error Generating New Filename\r\n");
				printf(" | | %s\r\n", FATFS_fresult_desc(fresult));
				break;
			case SCAN_ERR_FILE_FORMATTING:
				printf(" | Error Formatting Header of File\r\n");
				printf(" | | %s\r\n", FATFS_fresult_desc(fresult));
				break;
			default:
				printf(" | Error code %u\r\n", error_code);
		};
	}
	
	/* Error loop */
	while (1);
}

/**
  *	Generates new filename for scans. Format of scan filename is:
  *
  *		scan###.lam
  * 
  * Note: This assumes that the SD is already mounted and initialized as it
  *		  is called within the scan function
  */
char* get_new_filename(void)
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

/**
  * Sets file format description in print buffer to be placed in header
  * and returns size of buffer
  */ 
int set_data_format(express_mode_t mode)
{
    return sprintf(print_buffer,
                   "# File format: "
                   "[SERVO ANGLE],[DISTANCE],[LIDAR ANGLE]\r\n");
}

/**
  * Formats header of file to look better by adding '#' square around all 
  * data
  */ 
void format_header_file_data(void)
{
    char read_characters[512] = {0};
	UINT i, j, bytes_read = 0, longest_length = 0, line_length = 0, tmp_longest_line = 0;
	
	/* go to start of file */
	fresult = f_rewind(&fptr);
	if (fresult)
		scan_error(SCAN_ERR_FILE_FORMATTING);
	
	/* read characters into buffer */
	fresult = f_read(&fptr, &read_characters, 512, &bytes_read);
	if (fresult) 
		scan_error(SCAN_ERR_FILE_FORMATTING);
	
    if (DEBUG) 
		printf("\r\nPre-formatted header:\r\n\r\n%s\r\n", read_characters);
	
	/* rewind file pointer to beginning to rewrite */
	fresult = f_rewind(&fptr);
	if (fresult)
		scan_error(fresult);
		
	/* reset print buffer for clean rewrite */
	//reset_print_buffer();
	
	/* get longest line */
	for (i = 0; i < bytes_read; i++) {
		if (read_characters[i] == '\n') {
			if (tmp_longest_line > longest_length)
				longest_length = tmp_longest_line;
			tmp_longest_line = 0;
			continue;
		}
		tmp_longest_line++;
	}
	
	if (DEBUG) {
		printf("bytes read = %u\r\n", bytes_read);
		printf("longest length = %u\r\n", longest_length);
	}
	
	/* create formatted lines */
	if (DEBUG) 
		printf("\r\nFormatted header:\r\n\r\n");

	for (i = 0; i < longest_length + 2; i++)
		strncat(print_buffer, "#", 1);
	strncat(print_buffer, "\r\n", 2);
	
    i = 0;
	while (i != bytes_read) {
		if (read_characters[i] == '\r') {
			for (j = 0; j < (longest_length - line_length - 1); j++) 
				strncat(print_buffer, " " , 1);
			strncat(print_buffer, "##", 2);
		}
		else if (read_characters[i] == '#') {
			strncat(print_buffer, "#", 1);
			line_length = 0;
			
			/* write formatted lines */
			if (DEBUG) printf("%s", print_buffer);
			write_print_buffer((UINT)strlen(print_buffer));
		}
		strncat(print_buffer, &read_characters[i++], 1);
		line_length++;
	}
	
	/* write last lines */
	for (i = 0; i < longest_length + 2; i++)
		strncat(print_buffer, "#", 1);
	strncat(print_buffer, "\r\n", 2);
	
	if (DEBUG) printf("%s", print_buffer);
	write_print_buffer((UINT)strlen(print_buffer));
}

/** 
  * Write contents of print buffer to SD and reset the print buffer
  */ 
void write_print_buffer(UINT data_length)
{
    fresult = f_write(&fptr, print_buffer, data_length, &bwritten);
    if (!fresult && bwritten != data_length)
        scan_error(SCAN_ERR_FILE_WRITE);
    reset_print_buffer();
}

/** 
  * Resets local print buffer.
  */
void reset_print_buffer(void)
{
	int i;
	for (i=0; i<MAX_PRINT_BUFFER_SIZE; i++)
		print_buffer[i] = '\0';
}
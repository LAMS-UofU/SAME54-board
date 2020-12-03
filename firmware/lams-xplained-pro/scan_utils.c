#include "scan_utils.h"
#include "common.h"
#include "lams_sd.h"
#include "drivers.h"
#include <string.h>
#include <math.h>

static int check_repeated_angles(volatile char*, double);
static void recover(void);
static void clear_scan_data(void);

extern char filename[12];   // filename format SCAN###.lam
extern char scan_print_buffer[MAX_PRINT_BUFFER_SIZE];

/* LIDAR variables - see lidar.c for init */
extern volatile write_data_s	sd_scan_data[MAX_SCANS];
extern resp_desc_s				resp_desc;
extern conf_data_t				conf_data;
extern uint32_t					scan_count;
extern uint32_t					invalid_exp_scans;
extern uint32_t					byte_count;
extern uint8_t					lidar_request;
extern uint8_t					lidar_config;
extern volatile char			DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

/* LIDAR responses - see responses.c for init */
extern legacy_cabin_data_s  legacy_cabins[LEGACY_CABIN_COUNT];
extern ultra_cabin_data_s   ultra_cabins[EXT_CABIN_COUNT];
extern dense_cabin_data_s   dense_cabins[DENSE_CABIN_COUNT];

/* servo variables */
extern uint16_t angle;

/* disk variables */
extern FATFS   fatfs;
extern FIL     fptr;
extern DSTATUS dstatus;
extern FRESULT fresult;
extern UINT    bwritten;
extern FILINFO finfo;

/* see main.c for init */
extern volatile uint32_t timer;
extern volatile uint8_t status;

static uint8_t restart_count;

/**
  *	Write scans to SD
  */
void write_scans(void)
{
	int i, length;

	//printf("\r\nAveraged Scan Data:\r\n");
	
	for (i = 0; i < MAX_SCANS; i++)
	{
		//if (LAMS_DEBUG) {
			//length = sprintf(scan_print_buffer,
							 //"\t%s,%s,%s",
							 //sd_scan_data[i].servo_angle_str,
							 //sd_scan_data[i].distance,
							 //sd_scan_data[i].lidar_angle);
			//printf(scan_print_buffer);
			//for (j = 0; j < (32 - length); j++)
				//printf(" ");
			//printf("(repeated %u times)\r\n", sd_scan_data[i].repeated_n);
		//}
		write_print_buffer (
			sprintf(scan_print_buffer,
					"%s,%s,%s\r\n",
					sd_scan_data[i].distance,
					sd_scan_data[i].lidar_angle,
					sd_scan_data[i].servo_angle_str
			)
		);
	}
	
	clear_scan_data();
	
	restart_count = 0;
}

/**
  * Average processed scan cabins and write to SD
  */
void average_cabins(uint8_t servo_angle)
{
    int i, repeated_idx;
	double distance, angle;
	
	switch (resp_desc.data_type) {
		
		/* Legacy Version */
		case EXPRESS_SCAN_LEGACY_VER:
			sd_scan_data[scan_count].servo_angle = servo_angle;
			
			set_double_repres((char*)sd_scan_data[scan_count].lidar_angle, 
							  legacy_cabins[0].start_angle / 64.0, 25);
			
			repeated_idx = check_repeated_angles(sd_scan_data[scan_count].lidar_angle, servo_angle);
			
			/* average scans */
			distance = 0;
			for (i = 0; i < LEGACY_CABIN_COUNT; i++) {
				/* sample 1 */
				angle = (legacy_cabins[i].angle_value1 & 0x0F) / 8.0;
				if (legacy_cabins[i].angle_value1 >> 4) angle = -angle;
				distance += ( legacy_cabins[i].distance1 / cos(angle) );
				/* sample 2 */
				angle = (legacy_cabins[i].angle_value2 & 0x0F) / 8;
				if (legacy_cabins[i].angle_value2 >> 4) angle = -angle;
				distance += ( legacy_cabins[i].distance2 / cos(angle) );
			}
			
			distance /= (LEGACY_CABIN_COUNT * 2);
			
			/* if distance ~ 0, don't add */
			if (distance < 0.001)
				return;
			
			/* account for repeated angles */
			if (repeated_idx != -1) {
				distance += strtod((char*)sd_scan_data[repeated_idx].distance, NULL);
				distance /= 2.0;
				
				set_double_repres((char*)sd_scan_data[repeated_idx].distance, 
								  distance, 6);
				break;
			}
			set_double_repres((char*)sd_scan_data[scan_count].distance, 
							  distance, 6);
			
			scan_count++;
			break;
		
		/* Extended Version */
		//case EXPRESS_SCAN_EXTENDED_VER:
		//
		///* Dense Version */
		//case EXPRESS_SCAN_DENSE_VER:
	}
}

/**
  * 
  */
void average_samples(double servo_angle)
{
	int i, repeated_idx;
	double distance, angle, start_angle;
	
	switch (resp_desc.data_type) {
		
		/* Legacy Version */
		case EXPRESS_SCAN_LEGACY_VER:

			start_angle = legacy_cabins[0].start_angle / 64.0;
			
			/* average scans */
			for (i = 0; i < LEGACY_CABIN_COUNT; i++) {
				
				/* sample 1 */
				sd_scan_data[scan_count].servo_angle = servo_angle;
				set_double_repres((char*)sd_scan_data[scan_count].servo_angle_str, servo_angle, 2);
				angle = (legacy_cabins[i].angle_value1 & 0x0F) / 8.0;
				if (legacy_cabins[i].angle_value1 >> 4) angle = -angle;
				angle += start_angle;
				
				if (angle > 360)
					angle -= 360;
				
				set_double_repres((char*)sd_scan_data[scan_count].lidar_angle,
								  angle, 25);
				repeated_idx = check_repeated_angles(sd_scan_data[scan_count].lidar_angle, servo_angle);
				
				distance = legacy_cabins[i].distance1;
				if (distance > 0.001) {
					if (repeated_idx != scan_count) {
						distance += strtod((char*)sd_scan_data[repeated_idx].distance, NULL);
						distance /= 2.0;
					
						set_double_repres((char*)sd_scan_data[repeated_idx].distance,
										  distance, 6);
					} else {
						set_double_repres((char*)sd_scan_data[scan_count].distance,
										  distance, 6);
						scan_count++;	
					}
				}
				
				/* sample 2 */
				sd_scan_data[scan_count].servo_angle = servo_angle;
				set_double_repres((char*)sd_scan_data[scan_count].servo_angle_str, servo_angle, 2);
				angle = (legacy_cabins[i].angle_value2 & 0x0F) / 8;
				if (legacy_cabins[i].angle_value2 >> 4) angle = -angle;
				angle += start_angle;
				
				set_double_repres((char*)sd_scan_data[scan_count].lidar_angle,
								  angle, 25);
				repeated_idx = check_repeated_angles(sd_scan_data[scan_count].lidar_angle, servo_angle);
				
				distance = legacy_cabins[i].distance2;
				if (distance > 0.001) {
					if (repeated_idx != scan_count) {
						distance += strtod((char*)sd_scan_data[repeated_idx].distance, NULL);
						distance /= 2.0;
						
						set_double_repres((char*)sd_scan_data[repeated_idx].distance,
										  distance, 6);
					} else {
						set_double_repres((char*)sd_scan_data[scan_count].distance,
										  distance, 6);
						scan_count++;
					}
				}
			}
			break;
		
		/* Extended Version */
		//case EXPRESS_SCAN_EXTENDED_VER:
		//
		///* Dense Version */
		//case EXPRESS_SCAN_DENSE_VER:
	}
}

/**
  * Print any info based on the type of scan error and keep in infinite loop
  * while blinking light to symbolize error
  */ 
void scan_error(uint16_t error_code)
{
    //if (LAMS_DEBUG)
		printf("\r\n[Scan Error]\r\n");
		
	status = STATUS_ERROR;
		
	LIDAR_REQ_stop();
	
	//if (LAMS_DEBUG) {
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
				printf(" | | %s\r\n", DISKIO_dstatus_desc(dstatus));
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
	//}
	
	/* try to restart request */
	if ((error_code == 1) && (restart_count <= 10)) {
		recover();
		return;
	}
	else {
		/* stop motor */
		LIDAR_PWM_stop();
		/* delete file as it is now invalid */
		fresult = f_unlink(filename);
		f_mount(0, "", 0);
		/* Error loop */
		while (1);
	}
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
			
		/* shift right 4 to cut off "scan" */
		for (i=0; i<3; i++)
			lfilenum[i] = finfo.fname[i+4];
		filenum = atoi((char*)&lfilenum);
		
		/* check if higher for highest scan number */
		if (filenum > highfilenum) 
			highfilenum = filenum;	
	}
	
	sprintf(filename, "SCAN%03u.LAM", highfilenum + 1);
	if (LAMS_DEBUG) 
		printf("\r\nWriting to %s\r\n", filename);
	
	return (TCHAR*)filename;
}

/**
  * Sets file format description in print buffer to be placed in header
  * and returns size of buffer
  */ 
int set_data_format(express_mode_t mode)
{
    return sprintf(scan_print_buffer,
                   "# File format: "
                   "[DISTANCE],[LIDAR ANGLE],[SERVO ANGLE]\r\n");
}

void set_double_repres(char* buf, double num, uint16_t precision)
{
	uint32_t num_int, num_precision;
	
	num_int = (uint32_t) num;
	num_precision = (uint32_t)((num - num_int) * pow(10, precision));
	
	sprintf((char*)buf, "%"PRIu32".%.*"PRIu32"", num_int, precision, num_precision);
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
	
    if (LAMS_DEBUG) 
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
	
	if (LAMS_DEBUG) {
		printf("bytes read = %u\r\n", bytes_read);
		printf("longest length = %u\r\n", longest_length);
	}
	
	/* create formatted lines */
	if (LAMS_DEBUG) 
		printf("\r\nFormatted header:\r\n\r\n");

	for (i = 0; i < longest_length + 3; i++)
		strncat(scan_print_buffer, "#", 1);
	strncat(scan_print_buffer, "\r\n", 2);
	
    i = 0;
	while (i != bytes_read) {
		if (read_characters[i] == '\r') {
			for (j = 0; j < (longest_length - line_length); j++) 
				strncat(scan_print_buffer, " " , 1);
			strncat(scan_print_buffer, "##", 2);
		}
		else if (read_characters[i] == '#') {
			strncat(scan_print_buffer, "#", 1);
			line_length = 0;
			
			/* write formatted lines */
			if (LAMS_DEBUG) 
				printf("%s", scan_print_buffer);
			write_print_buffer((UINT)strlen(scan_print_buffer));
		}
		strncat(scan_print_buffer, &read_characters[i++], 1);
		line_length++;
	}
	
	/* write last lines */
	for (i = 0; i < longest_length + 3; i++)
		strncat(scan_print_buffer, "#", 1);
	strncat(scan_print_buffer, "\r\n", 2);
	
	if (LAMS_DEBUG) 
		printf("%s\r\n", scan_print_buffer);
	write_print_buffer((UINT)strlen(scan_print_buffer));
}

/** 
  * Write contents of print buffer to SD and reset the print buffer
  */ 
void write_print_buffer(UINT data_length)
{
    fresult = f_write(&fptr, scan_print_buffer, data_length, &bwritten);
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
		scan_print_buffer[i] = '\0';
}

/**
  *	Returns true if repeated angle
  */
int check_repeated_angles(volatile char* angle, double servo_angle)
{
	int i;
	for (i = 0; i <= scan_count; i++) {
		if (sd_scan_data[i].servo_angle == servo_angle) {
			/* check if angle strings match */
			if (!strcmp((char*)angle, (char*)sd_scan_data[i].lidar_angle)) {
				//printf("repeated index = %u, scan count = %"PRIu32"\r\n", i, scan_count);
				sd_scan_data[i].repeated_n++;
				return i;
			}
		}
	}
	return -1;
}

/**
  *	Recover from failure by requesting same request
  */
void recover(void)
{
	if (LAMS_DEBUG)
		printf("Attempting recovery...resending %s request", request_str_repr(lidar_request));
		
	clear_scan_data();
	
	status = STATUS_PROCESSING;
	
	switch (lidar_request) {
		case LIDAR_STOP:				LIDAR_REQ_stop(); return;
		case LIDAR_RESET:				LIDAR_REQ_reset(); return;
		case LIDAR_SCAN:				LIDAR_REQ_scan(); return;
		case LIDAR_EXPRESS_SCAN:		LIDAR_REQ_express_scan(EXPRESS_SCAN_MODE); return;
		case LIDAR_FORCE_SCAN:			LIDAR_REQ_force_scan(); return;
		case LIDAR_GET_INFO:			LIDAR_REQ_get_info(); return;
		case LIDAR_GET_HEALTH:			LIDAR_REQ_get_health(); return;
		case LIDAR_GET_SAMPLERATE:		LIDAR_REQ_get_samplerate(); return;
		case LIDAR_GET_LIDAR_CONF:		LIDAR_REQ_get_lidar_conf(SCAN_MODE, lidar_config); return;
		//case LIDAR_MOTOR_SPEED_CTRL:	LIDAR_REQ_motor_speed_ctrl(); return;
	}
}

void clear_scan_data(void)
{
	int i, j;
	/* clear written scans */
	for (i = 0; i < MAX_SCANS; i++) {
		sd_scan_data[i].repeated_n = 0;
		sd_scan_data[i].servo_angle = 0;
		for (j = 0; j < 32; j++) {
			sd_scan_data[i].distance[j] = '\0';
			sd_scan_data[i].lidar_angle[j] = '\0';
			sd_scan_data[i].servo_angle_str[j] = '\0';
		}
	}
}
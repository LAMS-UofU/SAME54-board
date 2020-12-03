#include "scan.h"
#include "scan_utils.h"
#include "common.h"
#include "servo/servo.h"
#include "lidar/lidar.h"
#include "fatfs.h"
#include "drivers.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static uint8_t process(void);

char filename[12];   // filename format SCAN###.lam
char scan_print_buffer[MAX_PRINT_BUFFER_SIZE];

/* LIDAR variables - see lidar.c for init */
extern volatile write_data_s	sd_scan_data[MAX_SCANS];
extern resp_desc_s				resp_desc;
extern conf_data_t				conf_data;
extern uint32_t					scan_count;
extern uint32_t					invalid_exp_scans;
extern uint32_t					byte_count;
extern uint8_t					lidar_request;
extern volatile char			DATA_RESPONSE[LIDAR_RESP_MAX_SIZE];

/* LIDAR responses - see responses.c for init */
extern legacy_cabin_data_s  legacy_cabins[LEGACY_CABIN_COUNT];
extern ultra_cabin_data_s   ultra_cabins[EXT_CABIN_COUNT];
extern dense_cabin_data_s   dense_cabins[DENSE_CABIN_COUNT];

/* local servo variables */
double angle;

/* disk variables - see fatfs.c for init */
extern FATFS   fatfs;
extern FIL     fptr;
extern DSTATUS dstatus;
extern FRESULT fresult;
extern UINT    bwritten;
extern FILINFO finfo;

/* see main.c for init */
extern volatile uint32_t timer;
extern volatile uint8_t  status;

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
	
	/* move servo to 0 */
	SERVO_set_angle(0);
	
	/* initialize SD card connection */
	dstatus = disk_initialize(0);
	if (dstatus)
		scan_error(SCAN_ERR_DISK_INIT);
	
	/* mount SD card */
	fresult = f_mount(&fatfs, path, 0);
	if (fresult)
		scan_error(SCAN_ERR_DISK_MOUNT);
	
	/* Wait until button pressed */
	if (LAMS_DEBUG)
		printf("\r\nPress button to start\r\n");
	while (!gpio_get_pin_level(START_BTN));
	
	/* give 10 seconds to move away from tripod */
	status = STATUS_PROCESSING;
	if (LAMS_DEBUG) 
		printf("\r\nCounting down from 10 to move away from tripod...");
	timer = 10000;
	while (timer);
	printf("0\r\n");
	
	/* create file */
	fresult = f_open(&fptr, get_new_filename(), FA_READ | FA_WRITE | FA_CREATE_NEW);
	if (fresult)
		scan_error(SCAN_ERR_FILE_CREATE);
	
	/* place filename in file */
	write_print_buffer(sprintf(scan_print_buffer, "# filename: %s\r\n", filename));
	
	/* start scan */
	if (LAMS_DEBUG)
		printf("\r\nStarting scan\r\n");
	
	/* reset lidar */
	LIDAR_REQ_reset();
	
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
	if (LAMS_DEBUG)
		printf(info);
	write_print_buffer(sprintf(scan_print_buffer, "%s", info));
	
	/* place scan mode in file */
	LIDAR_REQ_get_lidar_conf(SCAN_MODE, CONF_SCAN_MODE_NAME);
	while (!process());
	LIDAR_RES_get_lidar_conf();
	write_print_buffer(sprintf(scan_print_buffer, "# Scan mode is \"%s\"\r\n", conf_data.resp3));
	
	/* get system available scan count */
	LIDAR_REQ_get_lidar_conf(SCAN_MODE, CONF_SCAN_MODE_COUNT);
	while (!process());
	LIDAR_RES_get_lidar_conf();
	conf_count.resp1 = conf_data.resp1;
	
	/* place alternative scan mode names in file */
	write_print_buffer(sprintf(scan_print_buffer, "# Alternative modes: ["));
	for (i = 0; i < conf_count.resp1; i++) {
		LIDAR_REQ_get_lidar_conf(i, CONF_SCAN_MODE_NAME);
		while (!process());
		LIDAR_RES_get_lidar_conf();
		write_print_buffer(sprintf(scan_print_buffer, "\"%s\"", conf_data.resp3));
		if (i < (conf_count.resp1 - 1))
			write_print_buffer(sprintf(scan_print_buffer, ","));
	}
	write_print_buffer(sprintf(scan_print_buffer, "]\r\n"));
	
	/* place scan speed in file */
	LIDAR_REQ_get_lidar_conf(SCAN_MODE, CONF_SCAN_MODE_US_PER_SAMPLE);
	while (!process());
	LIDAR_RES_get_lidar_conf();
	write_print_buffer(sprintf(scan_print_buffer, "# Scan costs %"PRIu32"us per sample\r\n", conf_data.resp2));
	
	/* place scan max distance in file */
	LIDAR_REQ_get_lidar_conf(SCAN_MODE, CONF_SCAN_MODE_MAX_DISTANCE);
	while (!process());
	LIDAR_RES_get_lidar_conf();
	write_print_buffer(sprintf(scan_print_buffer, "# Max measuring distance is %"PRIu32"m\r\n", conf_data.resp2));
	
	/* place data format in file */
	write_print_buffer(set_data_format(EXPRESS_SCAN_MODE));
	
	/* format header data to look better */
	format_header_file_data();
	
	/* start scans */
	LIDAR_PWM_start();
	for (angle = 0.0; angle <= 180.0; angle+=0.25) {
		SERVO_set_angle(angle);
		timer = 50;
		while (timer);
		if (LAMS_DEBUG) {
			set_double_repres(scan_print_buffer, angle, 2);
			printf("Servo angle %s\r\n", scan_print_buffer);
			reset_print_buffer();
		}
		LIDAR_REQ_express_scan(EXPRESS_SCAN_MODE);
		while (scan_count < MAX_SCANS) {
			while (!process());
			if (LIDAR_RES_express_scan()) {
				average_samples(angle);
			} 
		}
		write_scans();
		scan_count = 0;
	}
	
	/* all scans complete, stop processes */
	LIDAR_PWM_stop();
	LIDAR_REQ_stop();
	
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
		scan_error(SCAN_ERR_OUT_OF_BOUNDS);
	}
	
	return PROCESSING;
}

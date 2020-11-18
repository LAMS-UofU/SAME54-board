#ifndef LIDAR_H_
#define LIDAR_H_

#include "common.h"

extern struct usart_sync_descriptor LIDAR_USART;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/***************************************************************************/
/*  DEFINITIONS                                                            */
/***************************************************************************/

/* LIDAR PWM Frequency 25kHz, 60% duty cycle */
#define LIDAR_PWM_COUNT				60
#define LIDAR_PWM_CC1				36

#define LIDAR_START_BYTE			0xA5
#define LIDAR_RES_BYTE				0x5A

#define LIDAR_STOP 					0x25
#define LIDAR_RESET 				0x40
#define LIDAR_SCAN					0x20
#define LIDAR_EXPRESS_SCAN 			0x82
#define LIDAR_FORCE_SCAN			0x21
#define LIDAR_GET_INFO				0x50
#define LIDAR_GET_HEALTH			0x52
#define LIDAR_GET_SAMPLERATE		0x59

#define LIDAR_SEND_MODE_SINGLE_RES	0x0
#define LIDAR_SEND_MODE_MULTI_RES	0x1

#define MAX_PRINT_BUFFER_SIZE		256
#define LIDAR_RESP_DESC_SIZE		7
#define LIDAR_RESP_MAX_SIZE			128

/***************************************************************************/
/*  DATA STRUCTURES					                                       */
/***************************************************************************/

/*  start1        - 1 byte (0xA5)
	start2        - 1 byte (0x5A)
	response_info - 32 bits ([32:30] send_mode, [29:0] data_length)
	data_type 	  - 1 byte	*/
typedef struct response_descriptor {
	uint8_t start1;
	uint8_t start2;
	uint32_t response_info;
	uint8_t data_type;
} response_descriptor;

/*	distance1    - 15 bits 
	distance2    - 15 bits
	angle_value1 - 5 bits
	angle_value2 - 5 bits 
	S			 - 1 bit */
typedef struct cabin_data {
	uint8_t S;
	uint8_t angle_value1;
	uint8_t angle_value2;
	uint16_t distance1;
	uint16_t distance2;
	uint16_t start_angle;
} cabin_data;

/*  quality - 6 bits
	angle   - 15 bits
	distance - 16 bits */
typedef struct scan_data {
	uint8_t quality;
	uint16_t angle;
	uint16_t distance;	
} scan_data;

/***************************************************************************/
/*  METHODS                                                                */
/***************************************************************************/

/* Used for debugging */
void LIDAR_menu(void);

void LIDAR_REQ_stop(void);
void LIDAR_REQ_reset(void);
void LIDAR_REQ_scan(void);
void LIDAR_REQ_express_scan(void);
void LIDAR_REQ_force_scan(void);
void LIDAR_REQ_get_info(void);
void LIDAR_REQ_get_health(void);
void LIDAR_REQ_get_samplerate(void);

void LIDAR_RES_stop(void);
void LIDAR_RES_reset(void);
void LIDAR_RES_scan(void);
void LIDAR_RES_express_scan(void);
char* LIDAR_RES_get_info(void);
uint16_t LIDAR_RES_get_health(void);
void LIDAR_RES_get_samplerate(void);

void LIDAR_PWM_init(void);
void LIDAR_PWM_start(void);
void LIDAR_PWM_stop(void);

void LIDAR_USART_init(void);
void LIDAR_USART_send(uint8_t *, uint16_t);
uint8_t LIDAR_USART_read_byte(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_H_ */
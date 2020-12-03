#ifndef LIDAR_H_
#define LIDAR_H_

#include <inttypes.h>

extern struct usart_sync_descriptor LIDAR_USART;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/***************************************************************************/
/*  DEFINITIONS                                                            */
/***************************************************************************/

/* LIDAR PWM Frequency 25kHz, 60% duty cycle */
#define LIDAR_PWM_COUNT					60
#define LIDAR_PWM_CC1					36

#define LIDAR_START_BYTE				0xA5
#define LIDAR_RES_BYTE					0x5A

#define LIDAR_STOP 						0x25
#define LIDAR_RESET 					0x40
#define LIDAR_SCAN						0x20
#define LIDAR_EXPRESS_SCAN 				0x82
#define LIDAR_FORCE_SCAN				0x21
#define LIDAR_GET_INFO					0x50
#define LIDAR_GET_HEALTH				0x52
#define LIDAR_GET_SAMPLERATE			0x59
#define LIDAR_GET_LIDAR_CONF			0x84
#define LIDAR_MOTOR_SPEED_CTRL			0xA8

#define LIDAR_SEND_MODE_SINGLE_RES		0x0
#define LIDAR_SEND_MODE_MULTI_RES		0x1

#define MAX_PRINT_BUFFER_SIZE			256
#define LIDAR_RESP_DESC_SIZE			7
#define LIDAR_RESP_MAX_SIZE				136

typedef uint16_t scan_mode_t;
#define SCAN_MODE_STANDARD				0
#define SCAN_MODE_EXPRESS				1
#define SCAN_MODE_BOOST					2
#define SCAN_MODE_STABILITY				3

typedef uint8_t express_mode_t;
#define EXPRESS_SCAN_STANDARD			(SCAN_MODE_EXPRESS - 1)
#define EXPRESS_SCAN_BOOST				(SCAN_MODE_BOOST - 1)
#define EXPRESS_SCAN_STABILITY			(SCAN_MODE_STABILITY - 1)

#define EXPRESS_SCAN_LEGACY_VER			0x82
#define EXPRESS_SCAN_EXTENDED_VER		0x84
#define EXPRESS_SCAN_DENSE_VER			0x85

typedef uint32_t conf_type_t;
#define CONF_SCAN_MODE_COUNT			0x70
#define CONF_SCAN_MODE_US_PER_SAMPLE	0x71
#define CONF_SCAN_MODE_MAX_DISTANCE		0x74
#define CONF_SCAN_MODE_ANS_TYPE			0x75
#define CONF_SCAN_MODE_TYPICAL			0x7C
#define CONF_SCAN_MODE_NAME				0x7F 

#define ANS_TYPE_STANDARD				0x81
#define ANS_TYPE_EXPRESS				0x82
/* Boost, stability, sensitivity */
#define ANS_TYPE_CONF					0x83

#define LEGACY_CABIN_COUNT				16
#define LEGACY_CABIN_BYTE_COUNT			5

#define DENSE_CABIN_COUNT				40
#define DENSE_CABIN_BYTE_COUNT			2

#define EXT_CABIN_COUNT					32
#define EXT_CABIN_BYTE_COUNT			4

typedef union {
	uint8_t  resp0;
	uint16_t resp1;
	uint32_t resp2;
	char	 resp3[16];
} conf_data_t;

/***************************************************************************/
/*  DATA STRUCTURES					                                       */
/***************************************************************************/

/*  start1        - 1 byte (0xA5)
	start2        - 1 byte (0x5A)
	response_info - 32 bits ([32:30] send_mode, [29:0] data_length)
	data_type 	  - 1 byte	*/
typedef struct {
	uint8_t start1;
	uint8_t start2;
	uint32_t response_info;
	uint8_t data_type;
} resp_desc_s;

/*	distance	- 16 bits
	S			- 1 bit
	start_angle - 15 bits */
typedef struct {
	uint8_t S;
	uint16_t start_angle;
	uint16_t distance;
} dense_cabin_data_s;

/*	distance1    - 14 bits 
	distance2    - 14 bits
	angle_value1 - 5 bits
	angle_value2 - 5 bits 
	S			 - 1 bit
	start_angle	 - 15 bits */
typedef struct {
	uint8_t S;
	uint8_t angle_value1;
	uint8_t angle_value2;
	uint16_t distance1;
	uint16_t distance2;
	uint16_t start_angle;
} legacy_cabin_data_s;

/*	predict2 - 10 bits
	predict1 - 10 bits
	major	 - 12 bits
	S		 - 1 bit
	start_angle - 15 bits */
typedef struct {
	uint8_t S;
	uint16_t start_angle;
	uint16_t predict1;
	uint16_t predict2;
	uint16_t major;
} ultra_cabin_data_s;

/*  quality - 6 bits
	angle   - 15 bits
	distance - 16 bits */
typedef struct {
	uint8_t quality;
	uint16_t angle;
	uint16_t distance;	
} scan_data_s;

/*	repeated_n - keeps track of repeated lidar angles

 
	servo angle (deg)	- 3 bits
	distance	(mm)	- 16 bits
	lidar angle	(deg)	- 32 bits */
typedef struct write_data {
	uint8_t repeated_n;
	double servo_angle;
	char servo_angle_str[32];
	char distance[32];
	char lidar_angle[32];
} write_data_s;

/***************************************************************************/
/*  METHODS                                                                */
/***************************************************************************/

/* Used for debugging */
void LIDAR_menu(void);
void LIDAR_conf_menu(void);
scan_mode_t LIDAR_conf_scan_menu(void);
express_mode_t LIDAR_express_scan_menu(void);

void LIDAR_REQ_stop(void);
void LIDAR_REQ_reset(void);
void LIDAR_REQ_scan(void);
void LIDAR_REQ_express_scan(express_mode_t);
void LIDAR_REQ_force_scan(void);
void LIDAR_REQ_get_info(void);
void LIDAR_REQ_get_health(void);
void LIDAR_REQ_get_samplerate(void);
void LIDAR_REQ_get_lidar_conf(scan_mode_t, conf_type_t);
//void LIDAR_REQ_motor_speed_ctrl(void);	RPLIDAR S1 only

// void LIDAR_RES_stop(void);   just need to wait 1ms
// void LIDAR_RES_reset(void);  just need to wait 2ms
void LIDAR_RES_scan(void);
uint8_t LIDAR_RES_express_scan(void);
char* LIDAR_RES_get_info(void);
uint16_t LIDAR_RES_get_health(void);
void LIDAR_RES_get_samplerate(void);
void LIDAR_RES_get_lidar_conf(void);
//void LIDAR_RES_motor_speed_ctrl(void);	RPLIDAR S1 only

void LIDAR_PWM_init(void);
void LIDAR_PWM_start(void);
void LIDAR_PWM_stop(void);

void LIDAR_USART_init(void);
void LIDAR_USART_send(uint8_t *, uint16_t);
uint8_t LIDAR_USART_read_byte(void);

char* request_str_repr(uint8_t);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_H_ */
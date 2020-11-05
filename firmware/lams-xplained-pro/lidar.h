#ifndef LIDAR_H_
#define LIDAR_H_

#include "atmel_start_pins.h"
#include <hal_usart_sync.h>

extern struct usart_sync_descriptor LIDAR_USART;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void LIDAR_menu(void);
void LIDAR_process(void);
void LIDAR_reset_print_buffer(void);
void LIDAR_reset_response_descriptor(void);

void LIDAR_print_scans(void);
void LIDAR_print_cabins(void);

// SERCOM0_1_Handler
void LIDAR_NewData_Handler(void);
void LIDAR_NewData_Config(void);

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
void LIDAR_RES_get_info(void);
void LIDAR_RES_get_health(void);
void LIDAR_RES_get_samplerate(void);

void LIDAR_PWM_PORT_init(void);
void LIDAR_PWM_CLOCK_init(void);
void LIDAR_PWM_init(void);
void LIDAR_PWM_start(void);
void LIDAR_PWM_stop(void);

void LIDAR_USART_PORT_init(void);
void LIDAR_USART_CLOCK_init(void);
void LIDAR_USART_init(void);
void LIDAR_USART_send(uint8_t *, uint16_t);
uint8_t LIDAR_USART_read_byte(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_H_ */
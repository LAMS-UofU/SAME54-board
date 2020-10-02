#ifndef LIDAR_H_
#define LIDAR_H_

#include "atmel_start_pins.h"
#include <hal_usart_sync.h>

extern struct usart_sync_descriptor LIDAR_USART;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void LIDAR_USART_PORT_init(void);
void LIDAR_USART_CLOCK_init(void);
void LIDAR_USART_init(void);

void LIDAR_PWM_PORT_init(void);
void LIDAR_PWM_CLOCK_init(void);
void LIDAR_PWM_init(void);
void LIDAR_PWM_start(void);
void LIDAR_PWM_stop(void);

void LIDAR_menu(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_H_ */
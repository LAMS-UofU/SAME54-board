/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_calendar.h>

#include <hal_usart_sync.h>

#include <hal_usart_sync.h>

#include <hal_mci_sync.h>
#include <tc_lite.h>
#include <tc_lite.h>

extern struct calendar_descriptor CALENDER_INTERFACE;

extern struct usart_sync_descriptor LIDAR_USART;

extern struct usart_sync_descriptor STDIO_IO;

extern struct mci_sync_desc SDHC_IO_BUS;

void CALENDER_INTERFACE_CLOCK_init(void);
void CALENDER_INTERFACE_init(void);

void LIDAR_USART_PORT_init(void);
void LIDAR_USART_CLOCK_init(void);
void LIDAR_USART_init(void);

void STDIO_IO_PORT_init(void);
void STDIO_IO_CLOCK_init(void);
void STDIO_IO_init(void);

void SDHC_IO_BUS_PORT_init(void);
void SDHC_IO_BUS_CLOCK_init(void);
void SDHC_IO_BUS_init(void);

void SERVO_PWM_CLOCK_init(void);

void SERVO_PWM_PORT_init(void);

int8_t SERVO_PWM_init(void);

void LIDAR_PWM_CLOCK_init(void);

void LIDAR_PWM_PORT_init(void);

int8_t LIDAR_PWM_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED

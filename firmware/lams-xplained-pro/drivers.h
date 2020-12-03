#ifndef DRIVERS_H_
#define DRIVERS_H_

#include "start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_usart_sync.h>
#include <hal_mci_sync.h>

#include "servo/servo.h"
#include "lidar/lidar.h"
#include "fatfs.h"

extern struct usart_sync_descriptor STDIO_IO;
extern struct mci_sync_desc SDHC_IO_BUS;
extern struct calendar_descriptor CALENDER_INTERFACE;

void STDIO_IO_PORT_init(void);
void STDIO_IO_CLOCK_init(void);
void STDIO_IO_init(void);

void SDHC_IO_BUS_PORT_init(void);
void SDHC_IO_BUS_CLOCK_init(void);
void SDHC_IO_BUS_init(void);

void GPIO_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* DRIVERS_H_ */

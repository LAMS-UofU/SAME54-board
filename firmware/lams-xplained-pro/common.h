#ifndef COMMON_H_
#define COMMON_H_

#include "start.h"
#include <peripheral_clk_config.h>
#include <hal_init.h>
#include <hal_usart_sync.h>
#include <stdlib.h>
#include <stdio.h>
#include <utils.h>
#include <inttypes.h>
#include "drivers.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAX_SCANS 64

/* Blink speeds in terms of ms */
#define BLINK_ERROR      100
#define BLINK_PROCESSING 500

/* Status flag */
#define STATUS_IDLE       0
#define STATUS_PROCESSING 1
#define STATUS_ERROR	  2

extern volatile uint32_t systick_count;

/* Remove for production */
#define DEBUG 1

/* Want to make sure scans still run at full speed regardless of SysTick...
   after debugging, this can be removed */
#define SYSTICK_EN 1 

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_H_ */
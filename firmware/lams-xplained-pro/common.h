#ifndef COMMON_H_
#define COMMON_H_

#include <hal_usart_sync.h>
#include <stdlib.h>
#include <stdio.h>
#include <utils.h>
#include <inttypes.h>
#include <hal_delay.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAX_SCANS 10

/* Blink speeds in terms of ms */
#define BLINK_ERROR       100
#define BLINK_PROCESSING  500

/* Remove for production */
#ifndef DEBUG
#define DEBUG 1
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_H_ */
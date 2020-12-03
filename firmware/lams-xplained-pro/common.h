#ifndef COMMON_H_
#define COMMON_H_

#include <hal_usart_sync.h>
#include <stdlib.h>
#include <stdio.h>
#include <utils.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAX_SCANS 128

/* Blink speeds in terms of ms */
#define BLINK_ERROR       100
#define BLINK_PROCESSING  500

/* Status flag */
#define STATUS_IDLE       0
#define STATUS_PROCESSING 1
#define STATUS_ERROR	  2

/* Remove for production */
#define LAMS_DEBUG 0

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_H_ */
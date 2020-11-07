#ifndef START_H_
#define START_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "drivers.h"
#include "stdio_redirect.h"
#include "sd_mmc_start.h"

/**
 * Initializes MCU, drivers and middleware in the project
 **/
void start_init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* START_H_ */

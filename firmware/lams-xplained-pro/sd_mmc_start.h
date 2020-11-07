#ifndef SD_MMC_START_H_
#define SD_MMC_START_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdio.h>

void SDMMC_ACCESS_example(void);

/**
 * \brief Initialize SD MMC Stack
 */
void sd_mmc_stack_init(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SD_MMC_START_H_ */

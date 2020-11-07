#include "start.h"
#include "sd_mmc.h"

#include "conf_sd_mmc.h"

/* Card Detect (CD) pin settings */
static sd_mmc_detect_t SDMMC_ACCESS_cd[CONF_SD_MMC_MEM_CNT] = {

    {-1, CONF_SD_MMC_0_CD_DETECT_VALUE},
};

/* Write Protect (WP) pin settings */
static sd_mmc_detect_t SDMMC_ACCESS_wp[CONF_SD_MMC_MEM_CNT] = {

    {-1, CONF_SD_MMC_0_WP_DETECT_VALUE},
};

static uint8_t sd_mmc_block[512];

/*
 * Example
 */
void SDMMC_ACCESS_example(void)
{
	while (SD_MMC_OK != sd_mmc_check(0)) {
		/* Wait card ready. */
	}
	if (sd_mmc_get_type(0) & (CARD_TYPE_SD | CARD_TYPE_MMC)) {
		/* Read card block 0 */
		sd_mmc_init_read_blocks(0, 0, 1);
		sd_mmc_start_read_blocks(sd_mmc_block, 1);
		sd_mmc_wait_end_of_read_blocks(false);
	}
#if (CONF_SDIO_SUPPORT == 1)
	if (sd_mmc_get_type(0) & CARD_TYPE_SDIO) {
		/* Read 22 bytes from SDIO Function 0 (CIA) */
		sdio_read_extended(0, 0, 0, 1, sd_mmc_block, 22);
	}
#endif
}

void sd_mmc_stack_init(void)
{

	sd_mmc_init(&SDHC_IO_BUS, SDMMC_ACCESS_cd, SDMMC_ACCESS_wp);
}

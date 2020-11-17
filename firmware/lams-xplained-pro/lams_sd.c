#include "common.h"
#include "sd_mmc.h"

/* Card Detect (CD) pin settings */
static sd_mmc_detect_t SDMMC_ACCESS_cd[CONF_SD_MMC_MEM_CNT] = {
	{-1, CONF_SD_MMC_0_CD_DETECT_VALUE},
};

/* Write Protect (WP) pin settings */
static sd_mmc_detect_t SDMMC_ACCESS_wp[CONF_SD_MMC_MEM_CNT] = {

	{-1, CONF_SD_MMC_0_WP_DETECT_VALUE},
};

/**
  *	Initializes SD MMC stack
  */
void SDMMC_init(void)
{
	sd_mmc_init(&SDHC_IO_BUS, SDMMC_ACCESS_cd, SDMMC_ACCESS_wp);
}


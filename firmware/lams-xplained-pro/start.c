#include <start.h>
#include "drivers.h"
#include "stdio_redirect.h"
#include "lams_sd.h"
#include "eeprom.h"

/**
 * Initializes MCU, drivers and middleware in the project
 **/
void start_init(void)
{
	system_init();
	stdio_redirect_init();
	SDMMC_init();
	EEPROM_init();
}

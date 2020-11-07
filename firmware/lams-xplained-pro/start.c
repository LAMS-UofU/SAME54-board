#include <start.h>

/**
 * Initializes MCU, drivers and middleware in the project
 **/
void start_init(void)
{
	system_init();
	stdio_redirect_init();
	sd_mmc_stack_init();
}

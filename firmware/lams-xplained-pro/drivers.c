#include "drivers.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>

struct usart_sync_descriptor STDIO_IO;
struct mci_sync_desc SDHC_IO_BUS;

void STDIO_IO_PORT_init(void)
{
	gpio_set_pin_function(PB25, PINMUX_PB25D_SERCOM2_PAD0);
	gpio_set_pin_function(PB24, PINMUX_PB24D_SERCOM2_PAD1);
}

void STDIO_IO_CLOCK_init(void)
{
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	hri_mclk_set_APBBMASK_SERCOM2_bit(MCLK);
}

void STDIO_IO_init(void)
{
	STDIO_IO_CLOCK_init();
	usart_sync_init(&STDIO_IO, SERCOM2, (void *)NULL);
	STDIO_IO_PORT_init();
}

void SDHC_IO_BUS_PORT_init(void)
{
	gpio_set_pin_direction(PA21, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PA21, false);
	gpio_set_pin_pull_mode(PA21, GPIO_PULL_OFF);
	gpio_set_pin_function(PA21, PINMUX_PA21I_SDHC1_SDCK);
	
	gpio_set_pin_direction(PA20, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PA20, false);
	gpio_set_pin_pull_mode(PA20, GPIO_PULL_OFF);
	gpio_set_pin_function(PA20, PINMUX_PA20I_SDHC1_SDCMD);

	gpio_set_pin_direction(PB18, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PB18, false);
	gpio_set_pin_pull_mode(PB18, GPIO_PULL_OFF);
	gpio_set_pin_function(PB18, PINMUX_PB18I_SDHC1_SDDAT0);

	gpio_set_pin_direction(PB19, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PB19, false);
	gpio_set_pin_pull_mode(PB19, GPIO_PULL_OFF);
	gpio_set_pin_function(PB19, PINMUX_PB19I_SDHC1_SDDAT1);

	gpio_set_pin_direction(PB20, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PB20, false);
	gpio_set_pin_pull_mode(PB20, GPIO_PULL_OFF);
	gpio_set_pin_function(PB20, PINMUX_PB20I_SDHC1_SDDAT2);

	gpio_set_pin_direction(PB21, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PB21, false);
	gpio_set_pin_pull_mode(PB21, GPIO_PULL_OFF);
	gpio_set_pin_function(PB21, PINMUX_PB21I_SDHC1_SDDAT3);
	
	gpio_set_pin_direction(CARD_DETECT_0, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(CARD_DETECT_0, GPIO_PULL_OFF);
	gpio_set_pin_function(CARD_DETECT_0, GPIO_PIN_FUNCTION_OFF);
	
	gpio_set_pin_direction(WRITE_PROTECT_0, GPIO_DIRECTION_IN);
	gpio_set_pin_pull_mode(WRITE_PROTECT_0, GPIO_PULL_OFF);
	gpio_set_pin_function(WRITE_PROTECT_0, GPIO_PIN_FUNCTION_OFF);
}

void SDHC_IO_BUS_CLOCK_init(void)
{
	hri_mclk_set_AHBMASK_SDHC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC1_GCLK_ID, CONF_GCLK_SDHC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC1_GCLK_ID_SLOW, CONF_GCLK_SDHC1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}

void SDHC_IO_BUS_init(void)
{
	SDHC_IO_BUS_CLOCK_init();
	mci_sync_init(&SDHC_IO_BUS, SDHC1);
	SDHC_IO_BUS_PORT_init();
}

void GPIO_init(void)
{
	gpio_set_pin_level(LED0, false);
	gpio_set_pin_direction(LED0, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(LED0, GPIO_PIN_FUNCTION_OFF);
	
	gpio_set_pin_level(START_BTN, false);
	gpio_set_pin_direction(START_BTN, GPIO_DIRECTION_IN);
	gpio_set_pin_function(START_BTN, GPIO_PIN_FUNCTION_OFF);
	
	gpio_set_pin_level(LED_STATUS, false);
	gpio_set_pin_direction(LED_STATUS, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(LED_STATUS, GPIO_PIN_FUNCTION_OFF);
}

void system_init(void)
{
	init_mcu();

	GPIO_init();
	FATFS_CALENDAR_init();
	LIDAR_USART_init();
	STDIO_IO_init();
	SDHC_IO_BUS_init();
	SERVO_PWM_init();
	LIDAR_PWM_init();
}

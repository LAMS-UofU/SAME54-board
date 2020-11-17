#include "start.h"
#include "stdio_redirect.h"
#include "drivers.h"

void STDIO_REDIRECT_example(void)
{
	/* Print welcome message */
	printf("\r\nHello ATMEL World!\r\n");
}

void stdio_redirect_init(void)
{
	usart_sync_enable(&STDIO_IO);
	stdio_io_init(&STDIO_IO.io);
}

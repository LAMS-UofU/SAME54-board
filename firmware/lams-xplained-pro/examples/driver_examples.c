/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

/**
 * Example of using LIDAR_USART to write "Hello World" using the IO abstraction.
 */
void LIDAR_USART_example(void)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&LIDAR_USART, &io);
	usart_sync_enable(&LIDAR_USART);

	io_write(io, (uint8_t *)"Hello World!", 12);
}

/**
 * Example of using STDIO_IO to write "Hello World" using the IO abstraction.
 */
void STDIO_IO_example(void)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&STDIO_IO, &io);
	usart_sync_enable(&STDIO_IO);

	io_write(io, (uint8_t *)"Hello World!", 12);
}

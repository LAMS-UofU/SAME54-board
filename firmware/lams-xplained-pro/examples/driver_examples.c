/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "drivers.h"
#include "utils.h"

/**
 * Example of using CALENDER_INTERFACE.
 */
static struct calendar_alarm alarm;

static void alarm_cb(struct calendar_descriptor *const descr)
{
	/* alarm expired */
}

void CALENDER_INTERFACE_example(void)
{
	struct calendar_date date;
	struct calendar_time time;

	calendar_enable(&CALENDER_INTERFACE);

	date.year  = 2000;
	date.month = 12;
	date.day   = 31;

	time.hour = 12;
	time.min  = 59;
	time.sec  = 59;

	calendar_set_date(&CALENDER_INTERFACE, &date);
	calendar_set_time(&CALENDER_INTERFACE, &time);

	alarm.cal_alarm.datetime.time.sec = 4;
	alarm.cal_alarm.option            = CALENDAR_ALARM_MATCH_SEC;
	alarm.cal_alarm.mode              = REPEAT;

	calendar_set_alarm(&CALENDER_INTERFACE, &alarm, alarm_cb);
}

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

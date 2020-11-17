#ifndef FATFS_H_
#define FATFS_H_

#ifdef __cplusplus
{
#endif /* __cplusplus */

#include <string.h>	
#include <hal_calendar.h>
#include "ff.h"

extern struct calendar_descriptor CALENDER_INTERFACE;

void FATFS_CALENDAR_CLOCK_init(void);
void FATFS_CALENDAR_init(void);

DWORD get_fattime(void);

void FATFS_write_file(TCHAR* filename, char* data, uint32_t data_length);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* FATFS_H_ */
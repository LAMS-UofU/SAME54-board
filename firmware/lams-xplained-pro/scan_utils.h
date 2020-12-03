#ifndef SCAN_UTILS_H_
#define SCAN_UTILS_H_

#include "scan.h"
#include "lidar/lidar.h"
#include "fatfs.h"
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void write_scans(void);
void average_cabins(uint8_t);
void average_samples(double);

void scan_error(uint16_t);

char* get_new_filename(void);
int set_data_format(express_mode_t);
void format_header_file_data(void);

void set_double_repres(char*, double, uint16_t);
void write_print_buffer(UINT length);
void reset_print_buffer(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SCAN_UTILS_H_ */
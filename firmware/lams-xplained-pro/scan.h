#ifndef SCAN_H_
#define SCAN_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define PROCESSING 0
#define COMPLETED  1

#define SCAN_ERR_TIMEOUT	        0
#define SCAN_ERR_OUT_OF_BOUNDS	    1
#define SCAN_ERR_DISK_INIT		    2
#define SCAN_ERR_DISK_MOUNT		    3
#define SCAN_ERR_FILE_CREATE	    4
#define SCAN_ERR_FILE_WRITE		    5
#define SCAN_ERR_FILE_CLOSE		    6
#define SCAN_ERR_NEW_FILENAME	    7
#define SCAN_ERR_FILE_FORMATTING    8

/* define scan mode */
#define SCAN_MODE           (scan_mode_t)    SCAN_MODE_EXPRESS
#define EXPRESS_SCAN_MODE   (express_mode_t) EXPRESS_SCAN_STANDARD

void scan(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SCAN_H_ */
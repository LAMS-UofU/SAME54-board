#ifndef LIDAR_REQ_RES_H_
#define LIDAR_REQ_RES_H_

#include "lidar.h"

#ifndef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define LIDAR_START_BYTE			0xA5
#define LIDAR_RES_BYTE				0x5A
#define LIDAR_REQ_STOP 				0x25
#define LIDAR_REQ_RESET 			0x40
#define LIDAR_REQ_SCAN				0x20
#define LIDAR_REQ_EXPRESS_SCAN 		0x82
#define LIDAR_REQ_FORCE_SCAN		0x21
#define LIDAR_REQ_GET_INFO			0x50
#define LIDAR_REQ_GET_HEALTH		0x52
#define LIDAR_REQ_GET_SAMPLERATE	0x59

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_REQ_RES_H_ */
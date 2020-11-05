#ifndef LIDAR_REQ_RES_H
#define LIDAR_REQ_RES_H

#ifndef __cplusplus
extern "C" {
#endif /* __cplusplus */

extern volatile uint8_t lidar_timer;
extern volatile uint8_t lidar_timing;

void LIDAR_reset_response_descriptor(void)

void LIDAR_REQ_stop(void);
void LIDAR_REQ_reset(void);
void LIDAR_REQ_scan(void);
void LIDAR_REQ_express_scan(void);
void LIDAR_REQ_force_scan(void);
void LIDAR_REQ_get_info(void);
void LIDAR_REQ_get_health(void);
void LIDAR_REQ_get_samplerate(void);

void LIDAR_RES_stop(void);
void LIDAR_RES_reset(void);
void LIDAR_RES_scan(void);
void LIDAR_RES_express_scan(void);
void LIDAR_RES_get_info(void);
void LIDAR_RES_get_health(void);
void LIDAR_RES_get_samplerate(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIDAR_REQ_RES_H */
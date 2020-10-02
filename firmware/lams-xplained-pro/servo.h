#ifndef SERVO_H_
#define SERVO_H_

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void SERVO_PWM_PORT_init(void);
void SERVO_PWM_CLOCK_init(void);
void SERVO_PWM_init(void);

void SERVO_set_angle(int angle);
void SERVO_menu(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SERVO_H_ */
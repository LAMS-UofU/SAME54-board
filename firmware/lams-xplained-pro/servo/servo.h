#ifndef SERVO_H_
#define SERVO_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void SERVO_PWM_init(void);
void SERVO_set_angle(double);
void SERVO_linear_transition_angle(double);

/* Debug method */
void SERVO_menu(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SERVO_H_ */
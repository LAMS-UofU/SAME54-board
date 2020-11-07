#ifndef SERVO_H_
#define SERVO_H_

#include "../common.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void SERVO_PWM_init(void);
void SERVO_set_angle(int angle);

/* Debug method */
void SERVO_menu(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SERVO_H_ */
#ifndef SERVO_H_
#define SERVO_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void SERVO_PWM_init(void);
void SERVO_set_angle(int angle);
void SERVO_menu(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SERVO_H_ */
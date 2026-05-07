#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVE_HPP


#define RIGHT_MOTOR_RPWM 17
#define RIGHT_MOTOR_LPWM 16

#define LEFT_MOTOR_RPWM 19
#define LEFT_MOTOR_LPWM 18

#define FORWARD  1
#define BACKWORD 0


bool activate_right_motor     (bool enable);
bool activate_left_motor      (bool enable);
void set_velocity_right_motor (float pwm);
void set_velocity_left_motor  (float pwm);


#endif
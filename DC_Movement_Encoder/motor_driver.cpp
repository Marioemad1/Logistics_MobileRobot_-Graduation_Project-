#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#include "motor_driver.hpp"
#include <Arduino.h>



bool activate_right_motor(bool enable)
{
    if (enable == true)
    {
      /* pinMode(RIGHT_MOTOR_EN , OUTPUT); */
      pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
      pinMode(RIGHT_MOTOR_LPWM, OUTPUT);
      
      /* digitalWrite(RIGHT_MOTOR_EN, HIGH); */
      analogWrite(RIGHT_MOTOR_RPWM, 0);
      analogWrite(RIGHT_MOTOR_LPWM, 0);
    }
    else
    {
      analogWrite(RIGHT_MOTOR_RPWM, 0);
      analogWrite(RIGHT_MOTOR_LPWM, 0);
     /*  digitalWrite(RIGHT_MOTOR_EN, LOW); */
     return false;
    }
    

    return true;
  

}

bool activate_left_motor(bool enable)
{

    if (enable == true)
    {
      /* pinMode(LEFT_MOTOR_EN , OUTPUT); */
      pinMode(LEFT_MOTOR_RPWM, OUTPUT);
      pinMode(LEFT_MOTOR_LPWM, OUTPUT);
        
      /* digitalWrite(LEFT_MOTOR_EN, HIGH); */
      analogWrite(LEFT_MOTOR_RPWM, 0);
      analogWrite(LEFT_MOTOR_LPWM, 0);
    }
    else
    {
      analogWrite(LEFT_MOTOR_RPWM, 0);
      analogWrite(LEFT_MOTOR_LPWM, 0);
      /* digitalWrite(LEFT_MOTOR_EN, LOW); */
      return false;
    }

    return true;
}

void set_velocity_right_motor(float pwm)
{
  if (pwm > 0) // Forward
  {
    analogWrite(RIGHT_MOTOR_RPWM, 0);
    analogWrite(RIGHT_MOTOR_LPWM, pwm);
  }
  else if (pwm < 0) // Reverse
  {
    analogWrite(RIGHT_MOTOR_RPWM, -pwm); // Make the PWM positive!
    analogWrite(RIGHT_MOTOR_LPWM, 0);
  }
  else // Stop
  {
    analogWrite(RIGHT_MOTOR_RPWM, 0);
    analogWrite(RIGHT_MOTOR_LPWM, 0);
  }
}

void set_velocity_left_motor(float pwm)
{
  if (pwm > 0) // Forward
  {
    analogWrite(LEFT_MOTOR_RPWM, 0);
    analogWrite(LEFT_MOTOR_LPWM, pwm);
  }
  else if(pwm < 0) // Reverse
  {
    analogWrite(LEFT_MOTOR_RPWM, -pwm); // Make the PWM positive!
    analogWrite(LEFT_MOTOR_LPWM, 0);
  }
  else // Stop
  {
    analogWrite(LEFT_MOTOR_RPWM, 0);
    analogWrite(LEFT_MOTOR_LPWM, 0);
  }
}
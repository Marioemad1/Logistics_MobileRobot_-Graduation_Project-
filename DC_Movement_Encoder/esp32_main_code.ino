#include "motor_driver.hpp"
#include "DRI_interface.hpp"

#define LED_PIN 2
/* 
#include <Arduino> */

volatile float global_PWM_right    = 0.0 ;
volatile float global_PWM_left     = 0.0 ;
volatile float global_lift_command = 0.0 ;

float previous_lift_command = 0.0;

void NemaMotorTask(void * paramter)
{
  DRI_init();

  for(;;)
  {
    // REPLACE THIS ENTIRE TASK WITH THIS CODE
    // Safe float math (> 0.5) AND checking if it's a NEW command
    if(global_lift_command > 0.5 && previous_lift_command <= 0.5) 
    {
      DRI_riseup(); 
      previous_lift_command = 1.0; // Remember that we moved UP
    }
    else if (global_lift_command < -0.5 && previous_lift_command >= -0.5)
    {
      DRI_falldown(); 
      previous_lift_command = -1.0; // Remember that we moved DOWN
    }
    else if (global_lift_command >= -0.5 && global_lift_command <= 0.5)
    {
      // ROS 2 is sending 0.0. Reset our memory so we are ready for the next move!
      previous_lift_command = 0.0;
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
  // put your setup code here, to run once:

  bool state_right = activate_right_motor(true);
  
  bool state_left = activate_left_motor(true);


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    NemaMotorTask,
    "NemaTask",
    10000,
    NULL,
    1,
    NULL,
    0
    );
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0)
  {
    String income_order_ = Serial.readStringUntil('\n');

    income_order_.trim();

    if(income_order_.startsWith("R:"))//<--this means the commend is a dc command 
    {
      int comma_index_ = income_order_.indexOf(','); //the index of the comma 


      if (comma_index_ != -1) //check if the comma exists 
      {

        String right_wheel_string = income_order_.substring(0,comma_index_); //"R:VEL"

        String left_wheel_string = income_order_.substring(comma_index_+1); //"L:VEL"


        if(right_wheel_string.startsWith("R:") && left_wheel_string.startsWith("L:") )
        {

          String right_number = right_wheel_string.substring(2);

          String left_number = left_wheel_string.substring(2);

          float right_wheel_velocity_ = right_number.toFloat();
          float left_wheel_velocity_ = left_number.toFloat();

          float right_pwm = right_wheel_velocity_ * 60.0;
          float left_pwm  = left_wheel_velocity_ * 60.0;

          global_PWM_right = constrain(right_pwm, -255.0, 255.0);
          global_PWM_left  = constrain(left_pwm, -255.0, 255.0);



          set_velocity_right_motor(global_PWM_right);
          set_velocity_left_motor(global_PWM_left);

          Serial.println("ACK");
        }
      }
    }
    else if (income_order_.startsWith("N:"))
    {
      String lift_command = income_order_.substring(2);
      global_lift_command = lift_command.toFloat();

      // REPLACE THE LED LOGIC WITH THIS
      // FIXED LED: Only blink if the command is actually telling it to move!
      if (global_lift_command > 0.5 || global_lift_command < -0.5) {
          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms delay makes the blink visible to humans!
          digitalWrite(LED_PIN, LOW);
      }

      Serial.println("ACK");
    }
  }

  vTaskDelay(1 / portTICK_PERIOD_MS);

}

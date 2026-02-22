/**************************** DC_Movement.cpp (ESP32 Core v3.0+ Version) ****************************
************************************************** Date     :  11/15/2025
************************************************** Name     :  Omar Mohamed Hamdy (Updated)
************************************************** Version  :  2.0 (v3.0 API)
****************************************************************************************************/

#include <Arduino.h>
#include "DC_Movement.hpp"

// --- Define PWM Properties ---
// Increased to 20kHz to eliminate high-pitched motor noise
const int PWM_FREQ = 20000;    
const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)

// --- Constructor ---
// Stores the pins. Note: Channels are handled automatically in ESP32 Core v3.0
RobotDrivetrain::RobotDrivetrain(int L_lpwm, int L_rpwm, int R_lpwm, int R_rpwm)
{
    _L_LPWM_PIN = L_lpwm;
    _L_RPWM_PIN = L_rpwm;
    _R_LPWM_PIN = R_lpwm;
    _R_RPWM_PIN = R_rpwm;
    
  
}

// --- init() ---
// Sets up the ESP32's ledc pins using the new v3.0 syntax
void RobotDrivetrain::init()
{
    // In v3.0, ledcAttach combines setup and attach.
    // Syntax: ledcAttach(pin, frequency, resolution)
    
    // Setup Left Motor Pins
    ledcAttach(_L_LPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(_L_RPWM_PIN, PWM_FREQ, PWM_RESOLUTION);

    // Setup Right Motor Pins
    ledcAttach(_R_LPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(_R_RPWM_PIN, PWM_FREQ, PWM_RESOLUTION);

    // Ensure motors are stopped at boot
    stop();
}

// --- Private Helper Functions ---

// Controls the Left Motor
void RobotDrivetrain::setLeftMotor(int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {                                   
        // Forward: Write directly to the PIN, not the channel
        ledcWrite(_L_LPWM_PIN, speed); 
        ledcWrite(_L_RPWM_PIN, 0);     
    }
    else if (speed < 0)
    {                                        
        // Reverse
        ledcWrite(_L_LPWM_PIN, 0);          
        ledcWrite(_L_RPWM_PIN, abs(speed)); 
    }
    else
    { 
        // Stop
        ledcWrite(_L_LPWM_PIN, 0);
        ledcWrite(_L_RPWM_PIN, 0);
    }
}

// Controls the Right Motor
void RobotDrivetrain::setRightMotor(int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {                                   
        // Forward
        ledcWrite(_R_LPWM_PIN, speed); 
        ledcWrite(_R_RPWM_PIN, 0);     
    }
    else if (speed < 0)
    {                                        
        // Reverse
        ledcWrite(_R_LPWM_PIN, 0);          
        ledcWrite(_R_RPWM_PIN, abs(speed)); 
    }
    else
    { 
        // Stop
        ledcWrite(_R_LPWM_PIN, 0);
        ledcWrite(_R_RPWM_PIN, 0);
    }
}

// --- Public Movement Functions ---
// These remain exactly the same as your original logic

void RobotDrivetrain::moveForward(int speed)
{
    setLeftMotor(speed);
    setRightMotor(speed);
}

void RobotDrivetrain::moveBackward(int speed)
{
    setLeftMotor(-speed);
    setRightMotor(-speed);
}

void RobotDrivetrain::turnLeft(int speed)
{
    setLeftMotor(-speed);
    setRightMotor(speed);
}

void RobotDrivetrain::turnRight(int speed)
{
    setLeftMotor(speed);
    setRightMotor(-speed);
}

void RobotDrivetrain::forwardLeft(int speed)
{
    setLeftMotor(speed / 2); 
    setRightMotor(speed);
}

void RobotDrivetrain::forwardRight(int speed)
{
    setLeftMotor(speed);
    setRightMotor(speed / 2);
}

void RobotDrivetrain::backwardLeft(int speed)
{
    setLeftMotor(-speed);
    setRightMotor(-speed / 2);
}

void RobotDrivetrain::backwardRight(int speed)
{
    setLeftMotor(-speed / 2); 
    setRightMotor(-speed);
}

void RobotDrivetrain::stop()
{
    setLeftMotor(0);
    setRightMotor(0);
}
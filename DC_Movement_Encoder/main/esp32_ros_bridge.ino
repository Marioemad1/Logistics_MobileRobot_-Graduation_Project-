/*********************************************************************************
 * ESP32 ROS2 BRIDGE
 * Receives Twist messages from Raspberry Pi via Serial (UART)
 * Converts to motor commands using differential drive kinematics
 * 
 * Hardware:
 *   - ESP32 connected to RPi via UART (GPIO 16=RX, GPIO 17=TX)
 *   - BTS7960 motor drivers
 *   - Encoders on both motors
 * 
 * Communication:
 *   - Baud rate: 115200
 *   - Format: JSON
 *   - Message: {"linear":0.5,"angular":0.2}
 *********************************************************************************/

#include <ArduinoJson.h>  // IMPORTANT:WE need to Install this library 
#include "DC_Movement_Encoder.hpp"

// ========== CONFIGURATION ==========

// Robot physical parameters (ADJUST TO YOUR ROBOT!)
const float WHEELBASE = 0.3;           // Distance between wheels (meters)
const float MAX_LINEAR_SPEED = 1.0;    // Maximum speed your robot can achieve (m/s)
                                       // Measure this by running motors at full speed

// Motor driver pins (BTS7960)
const int LEFT_LPWM  = 16;  // Left motor forward
const int LEFT_RPWM  = 17;  // Left motor reverse
const int RIGHT_LPWM = 18;  // Right motor forward
const int RIGHT_RPWM = 19;  // Right motor reverse

// Encoder pins
const int LEFT_ENC_A  = 32;  // Left encoder channel A
const int LEFT_ENC_B  = 33;  // Left encoder channel B
const int RIGHT_ENC_A = 25;  // Right encoder channel A
const int RIGHT_ENC_B = 26;  // Right encoder channel B

// Communication settings
const unsigned long SERIAL_BAUD = 115200;
const unsigned long TIMEOUT_MS = 1000;  // Stop motors if no command for 1 second

// ========== GLOBAL OBJECTS ==========

// Create robot drivetrain object
RobotDrivetrain robot(LEFT_LPWM, LEFT_RPWM, RIGHT_LPWM, RIGHT_RPWM,
                     LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B);

// Timing variables
unsigned long lastCommandTime = 0;

// ========== SETUP ==========

void setup() {
    // Initialize serial communication with Raspberry Pi
    Serial.begin(SERIAL_BAUD);
    
    // Wait for serial connection
    while (!Serial) {
        delay(10);
    }
    
    // Initialize robot hardware
    robot.init();
    
    // Send startup message
    Serial.println("{\"status\":\"ESP32 Ready\"}");
    
    delay(1000);
}

// ========== MAIN LOOP ==========

void loop() {
    // === CHECK FOR INCOMING COMMANDS ===
    if (Serial.available() > 0) {
        // Read JSON string until newline
        String jsonString = Serial.readStringUntil('\n');
        
        // Parse the JSON
        StaticJsonDocument<200> doc;  // Allocate memory for JSON document
        DeserializationError error = deserializeJson(doc, jsonString);
        
        // === CHECK FOR PARSING ERRORS ===
        if (error) {
            // Send error message back to RPi
            Serial.print("{\"error\":\"JSON parse failed: ");
            Serial.print(error.c_str());
            Serial.println("\"}");
            return;
        }
        
        // === EXTRACT VALUES FROM JSON ===
        // Get linear velocity (forward/backward)
        float linear_x = doc["linear"] | 0.0;  // Default to 0 if missing
        
        // Get angular velocity (rotation)
        float angular_z = doc["angular"] | 0.0;  // Default to 0 if missing
        
        // === CALCULATE MOTOR PWM VALUES ===
        int pwmValues[2];  // Array to hold [left_pwm, right_pwm]
        
        robot.twistToPWM(
            linear_x,           // Linear velocity (m/s)
            angular_z,          // Angular velocity (rad/s)
            WHEELBASE,          // Distance between wheels
            MAX_LINEAR_SPEED,   // Maximum linear speed
            pwmValues           // Output array
        );
        
        // === APPLY TO MOTORS ===
        robot.setLeftMotor(pwmValues[0]);   // Left motor PWM
        robot.setRightMotor(pwmValues[1]);  // Right motor PWM
        
        // === UPDATE LAST COMMAND TIME ===
        lastCommandTime = millis();
        
        // === SEND FEEDBACK TO RPI (OPTIONAL) ===
        // This helps with debugging
        Serial.print("{\"left_pwm\":");
        Serial.print(pwmValues[0]);
        Serial.print(",\"right_pwm\":");
        Serial.print(pwmValues[1]);
        Serial.print(",\"left_enc\":");
        Serial.print(robot.getLeftPosition());
        Serial.print(",\"right_enc\":");
        Serial.print(robot.getRightPosition());
        Serial.println("}");
    }
    
    // === SAFETY TIMEOUT ===
    // Stop motors if no command received for TIMEOUT_MS
    if (millis() - lastCommandTime > TIMEOUT_MS) {
        robot.stop();
        lastCommandTime = millis();  // Reset timer to avoid spamming stop commands
    }
    
    // Small delay for stability
    delay(20);  // 50Hz update rate
}
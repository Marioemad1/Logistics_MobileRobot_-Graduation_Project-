/*********************************************************************************
**************************************************    Date     :  12/21/2025
**************************************************    Name     :   Omar Mohamed Hamdy
**************************************************    Version  :   2.1 (with Encoders)
**************************************************    SWC      :   DC_Movement_Encoder.hpp
**************************************************    Purpose  :   Header file defining robot control with encoder feedback
*********************************************************************************/

#ifndef DC_MOVEMENT_ENCODER_HPP  // Header guard - prevents multiple inclusion
#define DC_MOVEMENT_ENCODER_HPP  // If not defined, define it now

#include <Arduino.h>  // Include Arduino core functions (pinMode, digitalWrite, etc.)

/*********************************************************************************
 * ENCODER DATA STRUCTURE
 * This structure holds all information related to one motor's encoder
 *********************************************************************************/
struct MotorEncoder {
    // === Hardware Pin Configuration ===
    int pinA;                    // GPIO pin connected to encoder Channel A (interrupt pin)
    int pinB;                    // GPIO pin connected to encoder Channel B (direction detection)
    
    // === Position Tracking ===
    volatile long position;      // Current encoder count (pulses accumulated)
                                 // 'volatile' tells compiler this can change in interrupt
                                 // Can be positive (forward) or negative (backward)
    
    volatile long targetPosition; // Desired position for closed-loop control
                                  // When position == targetPosition, motor should stop
    
    // === Encoder Specifications ===
    int ppr;                     // Pulses Per Revolution of encoder (7 for MY-775)
    float gearRatio;             // Motor gear ratio (125:1 for SG775125000)
                                 // Total resolution = ppr × gearRatio = 875 pulses/wheel revolution
    
    // === Speed Measurement ===
    volatile unsigned long lastPulseTime; // Timestamp (microseconds) of last encoder pulse
                                          // Used to calculate time between pulses → speed
    
    volatile float currentSpeed; // Current rotational speed in RPM (Revolutions Per Minute)
                                // Calculated from time between encoder pulses
};

/*********************************************************************************
 * ROBOT DRIVETRAIN CLASS
 * Main class that controls the entire robot's movement system
 *********************************************************************************/
class RobotDrivetrain {
public:
    /*********************************************************************************
     * CONSTRUCTOR
     * Called once when creating the robot object
     * 
     * Parameters explained:
     * @param L_lpwm : Left motor forward PWM pin (makes left motor spin forward)
     * @param L_rpwm : Left motor reverse PWM pin (makes left motor spin backward)
     * @param R_lpwm : Right motor forward PWM pin (makes right motor spin forward)
     * @param R_rpwm : Right motor reverse PWM pin (makes right motor spin backward)
     * @param L_encA : Left encoder Channel A (main pulse signal, generates interrupt)
     * @param L_encB : Left encoder Channel B (used to detect rotation direction)
     * @param R_encA : Right encoder Channel A (main pulse signal, generates interrupt)
     * @param R_encB : Right encoder Channel B (used to detect rotation direction)
     * @param ppr    : Pulses per revolution (default 7 for MY-775 encoder)
     * @param gearRatio : Motor gear ratio (default 125.0 for SG775125000)
     * 
     * Example usage:
     * RobotDrivetrain robot(16, 17, 18, 19, 32, 33, 25, 26, 7, 125.0);
     *********************************************************************************/
    RobotDrivetrain(int L_lpwm, int L_rpwm, int R_lpwm, int R_rpwm,
                    int L_encA, int L_encB, int R_encA, int R_encB,
                    int ppr = 7, float gearRatio = 125.0);

    /*********************************************************************************
     * INITIALIZATION FUNCTION
     * Must be called in Arduino setup() before using the robot
     * 
     * What it does:
     * 1. Configures PWM channels for motor control (20kHz frequency, 8-bit resolution)
     * 2. Sets up encoder GPIO pins as inputs with pull-up resistors
     * 3. Attaches interrupt handlers to encoder Channel A pins
     * 4. Stops motors (sets all PWM outputs to 0)
     * 
     * Example:
     * void setup() {
     *     robot.init();  // Initialize all hardware
     * }
     *********************************************************************************/
    void init();

    /*********************************************************************************
     * BASIC MOVEMENT FUNCTIONS (Open-Loop Control)
     * These functions control motors directly without using encoder feedback
     * Speed range: 0-255 (0 = stopped, 255 = full power)
     *********************************************************************************/
    
    /**
     * Move robot straight forward
     * Both motors spin forward at same speed
     * @param speed : PWM value 0-255
     */
    void moveForward(int speed);

    /**
     * Move robot straight backward
     * Both motors spin backward at same speed
     * @param speed : PWM value 0-255
     */
    void moveBackward(int speed);

    /**
     * Pivot turn left (rotate counter-clockwise in place)
     * Left motor: backward, Right motor: forward
     * Robot rotates around its center point
     * @param speed : PWM value 0-255
     */
    void turnLeft(int speed);

    /**
     * Pivot turn right (rotate clockwise in place)
     * Left motor: forward, Right motor: backward
     * Robot rotates around its center point
     * @param speed : PWM value 0-255
     */
    void turnRight(int speed);

    /**
     * Veer forward-left (curved path)
     * Left motor: half speed forward, Right motor: full speed forward
     * Robot moves forward while curving left
     * @param speed : PWM value 0-255
     */
    void forwardLeft(int speed);

    /**
     * Veer forward-right (curved path)
     * Left motor: full speed forward, Right motor: half speed forward
     * Robot moves forward while curving right
     * @param speed : PWM value 0-255
     */
    void forwardRight(int speed);

    /**
     * Veer backward-left (curved path)
     * Left motor: full speed backward, Right motor: half speed backward
     * Robot moves backward while curving left
     * @param speed : PWM value 0-255
     */
    void backwardLeft(int speed);

    /**
     * Veer backward-right (curved path)
     * Left motor: half speed backward, Right motor: full speed backward
     * Robot moves backward while curving right
     * @param speed : PWM value 0-255
     */
    void backwardRight(int speed);

    /**
     * Emergency stop
     * Sets all motor PWM outputs to 0
     * Disables position control mode
     */
    void stop();

    /*********************************************************************************
     * ENCODER READING FUNCTIONS
     * These functions provide access to encoder data
     *********************************************************************************/
    
    /**
     * Get left motor encoder position
     * @return Current pulse count (can be negative if moving backward)
     * 
     * Example: If robot moved forward, you might see 4375 pulses
     * This means: 4375 / 875 = 5 complete wheel rotations
     */
    long getLeftPosition();
    
    /**
     * Get right motor encoder position
     * @return Current pulse count (can be negative if moving backward)
     */
    long getRightPosition();
    
    /**
     * Get left motor position in degrees (output shaft)
     * Formula: (pulses / (PPR × gearRatio)) × 360°
     * @return Degrees rotated from last reset point
     * 
     * Example: 875 pulses = 360° = one complete wheel rotation
     */
    float getLeftDegrees();
    
    /**
     * Get right motor position in degrees (output shaft)
     * @return Degrees rotated from last reset point
     */
    float getRightDegrees();
    
    /**
     * Reset both encoder counters to zero
     * Also resets target positions
     * Use this before starting a new movement command
     * 
     * Example:
     * robot.resetEncoders();  // Start from zero
     * robot.moveDistance(1000, 180, 100);  // Move 1000mm
     */
    void resetEncoders();
    
    /**
     * Get left motor current speed
     * Calculated from time between encoder pulses
     * @return Speed in RPM (Revolutions Per Minute) of output shaft
     * 
     * Note: Speed becomes inaccurate at very low speeds (< 5 RPM)
     * because time between pulses is too long
     */
    float getLeftSpeed();
    
    /**
     * Get right motor current speed
     * @return Speed in RPM of output shaft
     */
    float getRightSpeed();

    /*********************************************************************************
     * POSITION CONTROL FUNCTIONS (Closed-Loop Control)
     * These functions use encoder feedback to move precise distances
     *********************************************************************************/
    
    /**
     * Move forward/backward a specific distance with encoder feedback
     * Robot will automatically stop when target distance is reached
     * 
     * @param distance : Distance to travel in millimeters
     *                   Positive = forward, Negative = backward
     * @param speed : Maximum motor speed (0-255)
     * @param wheelDiameter : Wheel diameter in millimeters
     * 
     * How it works:
     * 1. Calculates how many encoder pulses = desired distance
     * 2. Sets target position for both encoders
     * 3. Enables position control mode
     * 4. You must call updatePositionControl() in loop() repeatedly
     * 
     * Example:
     * robot.moveDistance(2000.0, 180, 100.0);  // Move 2 meters forward
     * while(!robot.updatePositionControl()) {  // Wait until complete
     *     delay(10);
     * }
     * 
     * Math:
     * Distance = (pulses / (PPR × gearRatio)) × (π × wheelDiameter)
     * Therefore: pulses = (distance × PPR × gearRatio) / (π × wheelDiameter)
     */
    void moveDistance(float distance, int speed, float wheelDiameter);
    
    /**
     * Rotate robot by specific angle with encoder feedback
     * Robot will pivot in place (one wheel forward, one backward)
     * 
     * @param angle : Rotation angle in degrees
     *                Positive = rotate right (clockwise)
     *                Negative = rotate left (counter-clockwise)
     * @param speed : Maximum motor speed (0-255)
     * @param wheelbase : Distance between left and right wheels (mm)
     * @param wheelDiameter : Wheel diameter in millimeters
     * 
     * How it works:
     * 1. Calculates arc length each wheel must travel: (angle/360) × π × wheelbase
     * 2. Converts arc length to encoder pulses
     * 3. Left wheel goes one direction, right wheel goes opposite
     * 4. You must call updatePositionControl() in loop() repeatedly
     * 
     * Example:
     * robot.rotateAngle(90.0, 150, 300.0, 100.0);  // Rotate 90° right
     * while(!robot.updatePositionControl()) {
     *     delay(10);
     * }
     */
    void rotateAngle(float angle, int speed, float wheelbase, float wheelDiameter);
    
    /**
     * PID Position Control Update Function
     * MUST be called repeatedly in loop() when position control is active
     * 
     * What it does:
     * 1. Reads current encoder positions
     * 2. Calculates error (how far from target)
     * 3. Uses PID algorithm to calculate motor speeds
     * 4. Adjusts motor speeds to reach target smoothly
     * 5. Stops motors when target is reached (within 5 pulses tolerance)
     * 
     * @return true if target reached, false if still moving
     * 
     * PID Algorithm:
     * - P (Proportional): Speed proportional to distance from target
     * - I (Integral): Corrects small steady-state errors
     * - D (Derivative): Dampens oscillations, prevents overshoot
     * 
     * Example usage:
     * void loop() {
     *     if (!robot.updatePositionControl()) {
     *         // Still moving toward target
     *     } else {
     *         // Target reached, do next action
     *     }
     *     delay(10);  // Update rate: 100Hz
     * }
     */
    bool updatePositionControl();
    
    /**
     * Set PID controller gains for position control
     * Tune these values to optimize robot performance
     * 
     * @param kp : Proportional gain (affects response speed)
     *             Higher = faster response, but can overshoot
     *             Typical range: 0.5 - 2.0
     * 
     * @param ki : Integral gain (eliminates steady-state error)
     *             Higher = faster error correction, but can cause windup
     *             Typical range: 0.001 - 0.05
     * 
     * @param kd : Derivative gain (reduces oscillation)
     *             Higher = more damping, smoother motion
     *             Typical range: 0.05 - 0.5
     * 
     * Default values (already set): Kp=0.8, Ki=0.01, Kd=0.1
     * 
     * Tuning tips:
     * 1. Start with Kp only (Ki=0, Kd=0)
     * 2. Increase Kp until robot responds quickly but oscillates
     * 3. Add Kd to stop oscillations
     * 4. Add small Ki if robot doesn't quite reach target
     */
    void setPIDGains(float kp, float ki, float kd);

    /*********************************************************************************
     * INTERRUPT SERVICE ROUTINES (ISR)
     * These functions are called automatically when encoder pulses occur
     * MUST be public so Arduino can attach them to interrupts
     * 
     * WARNING: Keep ISR code FAST - no Serial.print, no delay, no heavy computation
     *********************************************************************************/
    
    /**
     * Left encoder interrupt handler
     * Called every time left encoder Channel A goes from LOW to HIGH
     * Reads Channel B to determine direction, increments/decrements counter
     */
    void leftEncoderISR();
    
    /**
     * Right encoder interrupt handler
     * Called every time right encoder Channel A goes from LOW to HIGH
     * Reads Channel B to determine direction, increments/decrements counter
     */
    void rightEncoderISR();

    //  Differential drive kinematics
    void twistToPWM(float linear_x, float angular_z,
                    float wheelbase, float max_linear_speed,
                    int* pwmValues);


    void setLeftMotor(int speed);
    
    /**
     * Control right motor directly
     * @param speed : -255 to +255
     */
    void setRightMotor(int speed);

    
private:
    /*********************************************************************************
     * PRIVATE MOTOR CONTROL FUNCTIONS
     * These are internal functions, not accessible from outside the class
     *********************************************************************************/
    
    /**
     * Control left motor directly
     * @param speed : -255 to +255
     *                Positive = forward, Negative = backward, 0 = stop
     * 
     * Implementation:
     * - If speed > 0: Set LPWM = speed, RPWM = 0
     * - If speed < 0: Set LPWM = 0, RPWM = abs(speed)
     * - If speed = 0: Set LPWM = 0, RPWM = 0 (coast to stop)
     */

    /*********************************************************************************
     * PRIVATE MEMBER VARIABLES
     * These store the robot's internal state
     *********************************************************************************/
    
    // === Motor Driver Pin Numbers ===
    int _L_LPWM_PIN;  // Left motor forward PWM pin
    int _L_RPWM_PIN;  // Left motor reverse PWM pin
    int _R_LPWM_PIN;  // Right motor forward PWM pin
    int _R_RPWM_PIN;  // Right motor reverse PWM pin

    // === Encoder Data Structures ===
    MotorEncoder leftEncoder;   // All left encoder information
    MotorEncoder rightEncoder;  // All right encoder information
    
    // === PID Control Variables ===
    float _kp;  // Proportional gain
    float _ki;  // Integral gain
    float _kd;  // Derivative gain
    
    long _leftPrevError;   // Previous position error for left motor (used in derivative)
    long _rightPrevError;  // Previous position error for right motor
    
    long _leftIntegral;    // Accumulated error for left motor (used in integral)
    long _rightIntegral;   // Accumulated error for right motor
    
    // === Position Control State ===
    bool _positionControlActive;  // Flag: is robot in position control mode?
    int _maxControlSpeed;         // Maximum speed during position control (0-255)
};

/*********************************************************************************
 * GLOBAL POINTER FOR ISR ACCESS
 * 
 * Why needed:
 * Arduino interrupt functions cannot be class member functions directly
 * We need a global pointer to access the robot object from ISR
 * 
 * This is set in the constructor to point to the robot object
 *********************************************************************************/
extern RobotDrivetrain* g_drivetrain;

#endif // DC_MOVEMENT_ENCODER_HPP
// End of header guard - stops reading here
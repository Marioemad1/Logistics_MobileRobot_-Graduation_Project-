/*********************************************************************************
**************************************************    Date     :  12/21/2025
**************************************************    Name     :   Omar Mohamed Hamdy
**************************************************    Version  :   2.1 (with Encoders)
**************************************************    SWC      :   DC_Movement_Encoder.cpp
**************************************************    Purpose  :   Implementation of robot control with encoder feedback
*********************************************************************************/

#include "DC_Movement_Encoder.hpp"  // Include our header file with class definition

/*********************************************************************************
 * PWM CONFIGURATION CONSTANTS
 *********************************************************************************/
const int PWM_FREQ = 20000;      // PWM frequency: 20kHz (20,000 Hz)
                                 // Why 20kHz? Above human hearing range (20Hz-20kHz)
                                 // Eliminates annoying motor whine noise
                                 // Higher frequency = smoother motor operation

const int PWM_RESOLUTION = 8;    // PWM resolution: 8 bits
                                 // This gives us 2^8 = 256 possible values (0-255)
                                 // 0 = 0% duty cycle (motor off)
                                 // 255 = 100% duty cycle (motor full power)

/*********************************************************************************
 * GLOBAL POINTER FOR ISR ACCESS
 * Initialized to nullptr (null pointer) for safety
 *********************************************************************************/
RobotDrivetrain* g_drivetrain = nullptr;

/*********************************************************************************
 * INTERRUPT SERVICE ROUTINE WRAPPER FUNCTIONS
 * 
 * Why needed:
 * Arduino attachInterrupt() requires a plain C function, not a class member
 * These wrapper functions check if g_drivetrain is valid, then call member function
 * 
 * IRAM_ATTR attribute:
 * - Tells compiler to place this code in RAM instead of Flash memory
 * - Makes interrupt response FASTER (critical for high-speed encoders)
 * - Flash access on ESP32 can be slow and cause interrupt latency
 *********************************************************************************/

void IRAM_ATTR leftEncoderISR_Wrapper() {
    if (g_drivetrain) {              // Safety check: is pointer valid?
        g_drivetrain->leftEncoderISR();  // Call the actual ISR in the class
    }
}

void IRAM_ATTR rightEncoderISR_Wrapper() {
    if (g_drivetrain) {
        g_drivetrain->rightEncoderISR();
    }
}

/*********************************************************************************
 * CONSTRUCTOR IMPLEMENTATION
 * Called when you create robot object: RobotDrivetrain robot(16,17,18,19,32,33,25,26);
 *********************************************************************************/
RobotDrivetrain::RobotDrivetrain(int L_lpwm, int L_rpwm, int R_lpwm, int R_rpwm,
                                 int L_encA, int L_encB, int R_encA, int R_encB,
                                 int ppr, float gearRatio)
{
    // === Store Motor Driver Pin Numbers ===
    _L_LPWM_PIN = L_lpwm;  // Left motor forward control pin
    _L_RPWM_PIN = L_rpwm;  // Left motor reverse control pin
    _R_LPWM_PIN = R_lpwm;  // Right motor forward control pin
    _R_RPWM_PIN = R_rpwm;  // Right motor reverse control pin
    
    // === Initialize LEFT Encoder Structure ===
    leftEncoder.pinA = L_encA;           // Store Channel A pin number (GPIO 32)
    leftEncoder.pinB = L_encB;           // Store Channel B pin number (GPIO 33)
    leftEncoder.position = 0;            // Start position counter at zero
    leftEncoder.targetPosition = 0;      // No target initially
    leftEncoder.ppr = ppr;               // Pulses per revolution (7 for MY-775)
    leftEncoder.gearRatio = gearRatio;   // Gear ratio (125.0 for SG775125000)
                                         // Effective resolution = 7 × 125 = 875 pulses/wheel rev
    leftEncoder.lastPulseTime = 0;       // No pulses detected yet
    leftEncoder.currentSpeed = 0;        // Speed starts at zero
    
    // === Initialize RIGHT Encoder Structure ===
    rightEncoder.pinA = R_encA;          // GPIO 25
    rightEncoder.pinB = R_encB;          // GPIO 26
    rightEncoder.position = 0;           // Start at zero
    rightEncoder.targetPosition = 0;     // No target
    rightEncoder.ppr = ppr;              // Same 7 PPR
    rightEncoder.gearRatio = gearRatio;  // Same 125:1 ratio
    rightEncoder.lastPulseTime = 0;      // No pulses yet
    rightEncoder.currentSpeed = 0;       // Zero speed
    
    // === Initialize PID Controller Gains ===
    // These default values work reasonably well for most robots
    // You can change them later using setPIDGains()
    _kp = 0.8;   // Proportional: How aggressively to correct position errors
    _ki = 0.01;  // Integral: How quickly to eliminate steady-state errors
    _kd = 0.1;   // Derivative: How much to dampen oscillations
    
    // === Initialize PID State Variables ===
    _leftPrevError = 0;    // No previous error yet
    _rightPrevError = 0;   // No previous error yet
    _leftIntegral = 0;     // No accumulated error yet
    _rightIntegral = 0;    // No accumulated error yet
    
    // === Initialize Control Flags ===
    _positionControlActive = false;  // Start in open-loop (manual) mode
    _maxControlSpeed = 200;          // Default max speed for position control
    
    // === Set Global Pointer ===
    // This allows ISR wrapper functions to access this object
    g_drivetrain = this;  // 'this' is a pointer to the current object
}

/*********************************************************************************
 * INITIALIZATION FUNCTION
 * Call this ONCE in Arduino setup() before using robot
 *********************************************************************************/
void RobotDrivetrain::init()
{
    // === Setup PWM Channels for Motor Control ===
    // ESP32 Core v3.0 uses ledcAttach (combines old ledcSetup + ledcAttachPin)
    // Syntax: ledcAttach(pin, frequency, resolution)
    
    // Left motor PWM pins
    ledcAttach(_L_LPWM_PIN, PWM_FREQ, PWM_RESOLUTION);  // Pin 16: Forward PWM
    ledcAttach(_L_RPWM_PIN, PWM_FREQ, PWM_RESOLUTION);  // Pin 17: Reverse PWM
    
    // Right motor PWM pins
    ledcAttach(_R_LPWM_PIN, PWM_FREQ, PWM_RESOLUTION);  // Pin 18: Forward PWM
    ledcAttach(_R_RPWM_PIN, PWM_FREQ, PWM_RESOLUTION);  // Pin 19: Reverse PWM
    
    // After this, pins are ready to output PWM signals (0-255 values)
    
    // === Setup Encoder Input Pins ===
    // Configure as INPUT with internal pull-up resistors enabled
    // Pull-up resistors prevent floating inputs (noise) when encoder is idle
    
    pinMode(leftEncoder.pinA, INPUT_PULLUP);   // Pin 32: Left Channel A
    pinMode(leftEncoder.pinB, INPUT_PULLUP);   // Pin 33: Left Channel B
    pinMode(rightEncoder.pinA, INPUT_PULLUP);  // Pin 25: Right Channel A
    pinMode(rightEncoder.pinB, INPUT_PULLUP);  // Pin 26: Right Channel B
    
    // === Attach Interrupt Handlers to Encoder Channel A ===
    // When Channel A transitions from LOW to HIGH (RISING edge), call ISR
    // This happens once per encoder pulse
    
    attachInterrupt(
        digitalPinToInterrupt(leftEncoder.pinA),  // Convert pin number to interrupt number
        leftEncoderISR_Wrapper,                   // Function to call when interrupt occurs
        RISING                                    // Trigger on LOW→HIGH transition
    );
    
    attachInterrupt(
        digitalPinToInterrupt(rightEncoder.pinA),
        rightEncoderISR_Wrapper,
        RISING
    );
    
    // Note: We only attach to Channel A, not B
    // Channel B is read manually inside ISR to determine direction
    
    // === Ensure Motors Start Stopped ===
    stop();  // Set all PWM outputs to 0
}

/*********************************************************************************
 * LEFT ENCODER INTERRUPT SERVICE ROUTINE
 * Called automatically every time left encoder Channel A goes HIGH
 * 
 * CRITICAL: This code must execute FAST (< 10 microseconds)
 * - No Serial.print()
 * - No delay()
 * - No complex calculations
 *********************************************************************************/
void IRAM_ATTR RobotDrivetrain::leftEncoderISR()
{
    // === Read Current Time ===
    // micros() returns microseconds since ESP32 booted
    // We need this to calculate speed later
    unsigned long currentTime = micros();
    
    // === Determine Direction Using Quadrature Encoding ===
    // Quadrature encoding uses two channels (A and B) 90° out of phase
    // 
    // Forward rotation pattern:
    //   A: ↑ (we're here, in ISR)
    //   B: HIGH at this moment
    //   → Position should INCREASE
    //
    // Backward rotation pattern:
    //   A: ↑ (we're here, in ISR)
    //   B: LOW at this moment
    //   → Position should DECREASE
    
    if (digitalRead(leftEncoder.pinB) == HIGH) {
        // Channel B is HIGH when A rises → Forward rotation
        leftEncoder.position++;  // Increment counter
    } else {
        // Channel B is LOW when A rises → Backward rotation
        leftEncoder.position--;  // Decrement counter
    }
    
    // === Calculate Current Speed ===
    // Speed = 60,000,000 µs/min ÷ (time_between_pulses × pulses_per_revolution)
    // Result is in RPM (Revolutions Per Minute) of output shaft
    
    unsigned long timeDiff = currentTime - leftEncoder.lastPulseTime;
    // timeDiff = microseconds since last pulse
    
    if (timeDiff > 0) {  // Avoid division by zero
        // Formula derivation:
        // 1 minute = 60,000,000 microseconds
        // Time per revolution = timeDiff × (PPR × gearRatio)
        // RPM = 60,000,000 / (timeDiff × PPR × gearRatio)
        
        leftEncoder.currentSpeed = 60000000.0 / 
            (timeDiff * leftEncoder.ppr * leftEncoder.gearRatio);
        
        // Example calculation for SG775125000 (7 PPR, 125:1 gear):
        // If timeDiff = 1000 µs (1 millisecond between pulses)
        // Speed = 60,000,000 / (1000 × 7 × 125) = 68.6 RPM
    }
    
    // === Update Last Pulse Time ===
    leftEncoder.lastPulseTime = currentTime;  // Remember this time for next pulse
}

/*********************************************************************************
 * RIGHT ENCODER INTERRUPT SERVICE ROUTINE
 * Identical logic to left encoder ISR
 *********************************************************************************/
void IRAM_ATTR RobotDrivetrain::rightEncoderISR()
{
    unsigned long currentTime = micros();
    
    // Determine direction from Channel B state
    if (digitalRead(rightEncoder.pinB) == HIGH) {
        rightEncoder.position++;  // Forward
    } else {
        rightEncoder.position--;  // Backward
    }
    
    // Calculate speed from pulse timing
    unsigned long timeDiff = currentTime - rightEncoder.lastPulseTime;
    if (timeDiff > 0) {
        rightEncoder.currentSpeed = 60000000.0 / 
            (timeDiff * rightEncoder.ppr * rightEncoder.gearRatio);
    }
    rightEncoder.lastPulseTime = currentTime;
}

/*********************************************************************************
 * MOTOR CONTROL FUNCTIONS
 * These functions translate signed speed values (-255 to +255) into
 * PWM signals for the BTS7960 motor driver
 *********************************************************************************/

void RobotDrivetrain::setLeftMotor(int speed)
{
    // === Constrain Speed to Valid Range ===
    // Ensures speed is between -255 and +255
    // If speed > 255, sets it to 255
    // If speed < -255, sets it to -255
    speed = constrain(speed, -255, 255);
    
    // === Generate PWM Signals Based on Direction ===
    // BTS7960 driver uses dual PWM inputs:
    // - LPWM (Left PWM): Controls forward direction
    // - RPWM (Right PWM): Controls reverse direction
    // RULE: Never have both HIGH simultaneously (would cause short circuit)
    
    if (speed > 0) {
        // === FORWARD Direction ===
        ledcWrite(_L_LPWM_PIN, speed);  // Forward PWM = speed (1-255)
        ledcWrite(_L_RPWM_PIN, 0);      // Reverse PWM = 0 (disabled)
        
        // Example: speed = 200
        // LPWM = 200/255 = 78.4% duty cycle → motor spins forward
        // RPWM = 0/255 = 0% duty cycle → reverse is OFF
        
    } else if (speed < 0) {
        // === REVERSE Direction ===
        ledcWrite(_L_LPWM_PIN, 0);          // Forward PWM = 0 (disabled)
        ledcWrite(_L_RPWM_PIN, abs(speed)); // Reverse PWM = |speed|
        
        // Example: speed = -150
        // LPWM = 0 → forward is OFF
        // RPWM = 150/255 = 58.8% duty cycle → motor spins backward
        
    } else {
        // === STOP ===
        ledcWrite(_L_LPWM_PIN, 0);  // Forward PWM = 0
        ledcWrite(_L_RPWM_PIN, 0);  // Reverse PWM = 0
        
        // Both inputs LOW → motor coasts to stop (no braking)
        // BTS7960 has internal pull-downs, so this is safe
    }
}

void RobotDrivetrain::setRightMotor(int speed)
{
    // Identical logic to setLeftMotor()
    speed = constrain(speed, -255, 255);
    
    if (speed > 0) {
        ledcWrite(_R_LPWM_PIN, speed);  // Forward
        ledcWrite(_R_RPWM_PIN, 0);
    } else if (speed < 0) {
        ledcWrite(_R_LPWM_PIN, 0);
        ledcWrite(_R_RPWM_PIN, abs(speed));  // Reverse
    } else {
        ledcWrite(_R_LPWM_PIN, 0);  // Stop
        ledcWrite(_R_RPWM_PIN, 0);
    }
}

/*********************************************************************************
 * BASIC MOVEMENT FUNCTIONS (OPEN-LOOP CONTROL)
 * These functions command motors directly without encoder feedback
 *********************************************************************************/

void RobotDrivetrain::moveForward(int speed) {
    setLeftMotor(speed);   // Both motors forward
    setRightMotor(speed);  // Same speed → straight line
}

void RobotDrivetrain::moveBackward(int speed) {
    setLeftMotor(-speed);   // Both motors backward
    setRightMotor(-speed);  // Same speed → straight line (reverse)
}

void RobotDrivetrain::turnLeft(int speed) {
    setLeftMotor(-speed);  // Left motor backward
    setRightMotor(speed);  // Right motor forward
    // Robot pivots counter-clockwise around center point
}

void RobotDrivetrain::turnRight(int speed) {
    setLeftMotor(speed);   // Left motor forward
    setRightMotor(-speed); // Right motor backward
    // Robot pivots clockwise around center point
}

void RobotDrivetrain::forwardLeft(int speed) {
    setLeftMotor(speed / 2);  // Left motor half speed
    setRightMotor(speed);     // Right motor full speed
    // Robot curves left while moving forward
}

void RobotDrivetrain::forwardRight(int speed) {
    setLeftMotor(speed);      // Left motor full speed
    setRightMotor(speed / 2); // Right motor half speed
    // Robot curves right while moving forward
}

void RobotDrivetrain::backwardLeft(int speed) {
    setLeftMotor(-speed);      // Left motor full speed backward
    setRightMotor(-speed / 2); // Right motor half speed backward
    // Robot curves left while moving backward
}

void RobotDrivetrain::backwardRight(int speed) {
    setLeftMotor(-speed / 2);  // Left motor half speed backward
    setRightMotor(-speed);     // Right motor full speed backward
    // Robot curves right while moving backward
}

void RobotDrivetrain::stop() {
    setLeftMotor(0);   // Stop left motor
    setRightMotor(0);  // Stop right motor
    _positionControlActive = false;  // Disable position control mode
}

/*********************************************************************************
 * ENCODER READING FUNCTIONS
 * These provide access to encoder data for the user
 *********************************************************************************/

long RobotDrivetrain::getLeftPosition() {
    return leftEncoder.position;  // Return current pulse count
    // Can be negative if robot moved backward more than forward
}

long RobotDrivetrain::getRightPosition() {
    return rightEncoder.position;
}

float RobotDrivetrain::getLeftDegrees() {
    // Convert pulses to degrees of output shaft rotation
    // Formula: degrees = (pulses / total_pulses_per_rev) × 360°
    // total_pulses_per_rev = PPR × gearRatio = 7 × 125 = 875
    
    return (leftEncoder.position / (leftEncoder.ppr * leftEncoder.gearRatio)) * 360.0;
    
    // Example: position = 875 pulses
    // degrees = (875 / 875) × 360 = 360° = one complete wheel rotation
}

float RobotDrivetrain::getRightDegrees() {
    return (rightEncoder.position / (rightEncoder.ppr * rightEncoder.gearRatio)) * 360.0;
}

void RobotDrivetrain::resetEncoders() {
    // Reset all encoder counters to zero
    leftEncoder.position = 0;
    rightEncoder.position = 0;
    leftEncoder.targetPosition = 0;
    rightEncoder.targetPosition = 0;
    
    // Use this before starting a new movement:
    // robot.resetEncoders();
    // robot.moveDistance(1000, 180, 100);
}

float RobotDrivetrain::getLeftSpeed() {
    return leftEncoder.currentSpeed;  // Return calculated RPM
    // Note: Speed calculation happens in ISR, we just read it here
}

float RobotDrivetrain::getRightSpeed() {
    return rightEncoder.currentSpeed;
}

/*********************************************************************************
 * POSITION CONTROL FUNCTION: moveDistance()
 * Commands robot to move a specific distance using encoder feedback
 *********************************************************************************/
void RobotDrivetrain::moveDistance(float distance, int speed, float wheelDiameter)
{
    // === Calculate Required Encoder Pulses ===
    // 
    // Relationship between distance and encoder pulses:
    // distance = (pulses / (PPR × gearRatio)) × (π × wheelDiameter)
    // 
    // Solving for pulses:
    // pulses = (distance × PPR × gearRatio) / (π × wheelDiameter)
    
    float pulsesNeeded = (distance * leftEncoder.ppr * leftEncoder.gearRatio) / 
                         (PI * wheelDiameter);
    
    // Example calculation for your robot:
    // distance = 1000 mm
    // PPR = 7
    // gearRatio = 125
    // wheelDiameter = 100 mm
    // 
    // pulsesNeeded = (1000 × 7 × 125) / (3.14159 × 100)
    //              = 875,000 / 314.159
    //              = 2,784.8 pulses
    // 
    // Verification: 2784.8 / 875 = 3.18 wheel revolutions
    //               3.18 × (π × 100) = 999 mm ✓
    
    // === Prepare for Movement ===
    resetEncoders();  // Start from zero position
    
    // Set target for both wheels (same target = straight line)
    leftEncoder.targetPosition = (long)pulsesNeeded;
    rightEncoder.targetPosition = (long)pulsesNeeded;
    
    _maxControlSpeed = speed;  // Store maximum allowed speed
    _positionControlActive = true;  // Enable PID control mode
    
    // Important: You must now call updatePositionControl() repeatedly in loop()
}

/*********************************************************************************
 * POSITION CONTROL FUNCTION: rotateAngle()
 * Commands robot to rotate by specific angle using encoder feedback
 *********************************************************************************/
void RobotDrivetrain::rotateAngle(float angle, int speed, float wheelbase, float wheelDiameter)
{
    // === Calculate Arc Length Each Wheel Must Travel ===
    // When robot rotates in place, each wheel travels along an arc
    // Arc length = (angle/360°) × circumference of circle
    // Circle radius = wheelbase (distance between wheels)
    // Circumference = π × wheelbase
    
    float arcLength = (abs(angle) / 360.0) * PI * wheelbase;
    
    // Example calculation:
    // angle = 90°
    // wheelbase = 300 mm
    // 
    // arcLength = (90 / 360) × π × 300
    //           = 0.25 × 3.14159 × 300
    //           = 235.6 mm
    // 
    // Each wheel must travel 235.6 mm in opposite directions
    
    // === Convert Arc Length to Encoder Pulses ===
    float pulsesNeeded = (arcLength * leftEncoder.ppr * leftEncoder.gearRatio) / 
                         (PI * wheelDiameter);
    
    // Example continued:
    // pulsesNeeded = (235.6 × 7 × 125) / (π × 100)
    //              = 206,150 / 314.159
    //              = 656.2 pulses per wheel
    
    // === Prepare for Rotation ===
    resetEncoders();
    
    // Set opposite targets for left and right wheels
    if (angle > 0) {
        // Positive angle = Turn RIGHT (clockwise)
        leftEncoder.targetPosition = (long)pulsesNeeded;   // Left forward
        rightEncoder.targetPosition = -(long)pulsesNeeded; // Right backward
    } else {
        // Negative angle = Turn LEFT (counter-clockwise)
        leftEncoder.targetPosition = -(long)pulsesNeeded;  // Left backward
        rightEncoder.targetPosition = (long)pulsesNeeded;  // Right forward
    }
    
    _maxControlSpeed = speed;
    _positionControlActive = true;
    
    // Must call updatePositionControl() in loop()
}

/*********************************************************************************
 * PID CONTROL UPDATE FUNCTION
 * This implements the closed-loop control algorithm
 * MUST be called repeatedly (every 10-20ms) in Arduino loop()
 *********************************************************************************/
bool RobotDrivetrain::updatePositionControl()
{
    // === Check if Position Control is Active ===
    if (!_positionControlActive) {
        return true;  // Not in control mode, report "target reached"
    }
    
    // === Calculate Position Errors ===
    // Error = where we want to be - where we are
    long leftError = leftEncoder.targetPosition - leftEncoder.position;
    long rightError = rightEncoder.targetPosition - rightEncoder.position;
    
    // Example:
    // Target = 1000 pulses
    // Current position = 750 pulses
    // Error = 1000 - 750 = 250 pulses remaining
    
    // === Check if Target Reached ===
    // If both errors are small (< 5 pulses), consider target reached
    // 5 pulses = 5 / 875 = 0.0057 wheel revolutions
    //          = 0.0057 × (π × 100) = 1.8 mm tolerance
    if (abs(leftError) < 5 && abs(rightError) < 5) {
        stop();  // Stop motors and disable control mode
        return true;  // Signal that target is reached
    }
    
    // === PID CONTROL FOR LEFT MOTOR ===
    
    // 1. Integral Term (Accumulate Error)
    _leftIntegral += leftError;  // Add current error to sum
    
    // Anti-windup: Prevent integral from growing too large
    // Large integral can cause overshoot when error changes sign
    _leftIntegral = constrain(_leftIntegral, -10000, 10000);
    
    // 2. Derivative Term (Rate of Change of Error)
    long leftDerivative = leftError - _leftPrevError;
    // If error is decreasing, derivative is negative → dampens motion
    
    // 3. PID Formula
    // output = Kp×error + Ki×integral + Kd×derivative
    float leftOutput = (_kp * leftError) + 
                       (_ki * _leftIntegral) + 
                       (_kd * leftDerivative);
    
    // Example calculation:
    // Kp = 0.8, Ki = 0.01, Kd = 0.1
    // error = 250, integral = 1000, derivative = -10
    // output = (0.8 × 250) + (0.01 × 1000) + (0.1 × -10)
    //        = 200 + 10 - 1
    //        = 209
    
    // 4. Constrain Output to Valid Motor Speed Range
    int leftSpeed = constrain((int)leftOutput, -_maxControlSpeed, _maxControlSpeed);
    // Ensures speed stays within -200 to +200 (if _maxControlSpeed = 200)
    
    // === PID CONTROL FOR RIGHT MOTOR (Same Logic) ===
    _rightIntegral += rightError;
    _rightIntegral = constrain(_rightIntegral, -10000, 10000);
    
    long rightDerivative = rightError - _rightPrevError;
    
    float rightOutput = (_kp * rightError) + 
                        (_ki * _rightIntegral) + 
                        (_kd * rightDerivative);
    
    int rightSpeed = constrain((int)rightOutput, -_maxControlSpeed, _maxControlSpeed);
    
    // === Apply Calculated Speeds to Motors ===
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
    
    // === Update Previous Errors for Next Iteration ===
    _leftPrevError = leftError;
    _rightPrevError = rightError;
    
    // === Report Still Moving ===
    return false;  // Target not yet reached, keep calling updatePositionControl()
}

/*********************************************************************************
 * PID GAIN SETTER FUNCTION
 * Allows user to tune PID controller performance
 *********************************************************************************/
void RobotDrivetrain::setPIDGains(float kp, float ki, float kd)
{
    _kp = kp;  // Store new proportional gain
    _ki = ki;  // Store new integral gain
    _kd = kd;  // Store new derivative gain
    
    // Usage example:
    // robot.setPIDGains(1.2, 0.02, 0.15);  // More aggressive tuning
}


/*********************************************************************************
 * TWIST TO PWM CONVERSION
 * Converts ROS2 Twist message (linear + angular velocity) to motor PWM values
 * 
 * Parameters:
 *   linear_x         : Forward/backward velocity from Twist.linear.x (m/s)
 *   angular_z        : Rotation velocity from Twist.angular.z (rad/s)
 *   wheelbase        : Distance between wheels (meters)
 *   max_linear_speed : Maximum linear velocity your robot can achieve (m/s)
 *   pwmValues        : Output array [left_pwm, right_pwm]
 *********************************************************************************/
void RobotDrivetrain::twistToPWM(float linear_x, float angular_z,
                                  float wheelbase, float max_linear_speed,
                                  int* pwmValues)
{
    // === DIFFERENTIAL DRIVE KINEMATICS ===
    // For a two-wheeled robot:
    // v_left  = v - ω×L/2  (left wheel moves slower when turning right)
    // v_right = v + ω×L/2  (right wheel moves faster when turning right)
    
    float v_left  = linear_x - (angular_z * wheelbase / 2.0);
    float v_right = linear_x + (angular_z * wheelbase / 2.0);
    
    // Example:
    // linear_x = 0.5 m/s (moving forward)
    // angular_z = 0.2 rad/s (turning right)
    // wheelbase = 0.3 m
    // v_left  = 0.5 - (0.2 × 0.15) = 0.47 m/s
    // v_right = 0.5 + (0.2 × 0.15) = 0.53 m/s
    
    // === NORMALIZE TO PWM RANGE (-255 to +255) ===
    // Convert velocity (m/s) to PWM duty cycle
    // max_linear_speed represents 255 PWM
    
    int left_pwm  = (int)((v_left  / max_linear_speed) * 255.0);
    int right_pwm = (int)((v_right / max_linear_speed) * 255.0);
    
    // Example (continued):
    // If max_linear_speed = 1.0 m/s:
    // left_pwm  = (0.47 / 1.0) × 255 = 120
    // right_pwm = (0.53 / 1.0) × 255 = 135
    
    // === CONSTRAIN TO VALID PWM RANGE ===
    // Prevent values from exceeding motor driver limits
    left_pwm  = constrain(left_pwm,  -255, 255);
    right_pwm = constrain(right_pwm, -255, 255);
    
    // === STORE RESULTS IN OUTPUT ARRAY ===
    pwmValues[0] = left_pwm;   // Left motor PWM
    pwmValues[1] = right_pwm;  // Right motor PWM
}

// === END OF IMPLEMENTATION FILE ===
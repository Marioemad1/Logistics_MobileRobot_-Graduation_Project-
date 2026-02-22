// #include "DC_Movement_Encoder.hpp"

// // Pin Assignments for BTS7960 and Encoders
// const int L_LPWM = 16;
// const int L_RPWM = 17;
// const int R_LPWM = 18;
// const int R_RPWM = 19;
// const int L_ENCA = 32;
// const int L_ENCB = 33;
// const int R_ENCA = 25;
// const int R_ENCB = 26;

// // Robot Physical Constants
// const float WHEEL_DIAM = 100.0; // mm
// const float WHEEL_BASE = 300.0; // mm

// // Create Drivetrain instance
// RobotDrivetrain robot(L_LPWM, L_RPWM, R_LPWM, R_RPWM, L_ENCA, L_ENCB, R_ENCA,
//                       R_ENCB);

// void setup() {
//   Serial.begin(115200);

//   // Initialize PWM and interrupts via global pointer g_drivetrain
//   robot.init();

//   // Set PID gains for a stable 100kg chassis response
//   robot.setPIDGains(0.8, 0.01, 0.1);

//   Serial.println("--- AMR MOTION TEST STARTING ---");
//   delay(3000); // Safety delay to place robot on floor
// }

// void loop() {
//   // 1. Move Forward 500mm
//   Serial.println("Action: Forward 500mm");
//   robot.moveDistance(500.0, 150, WHEEL_DIAM);
//   waitForTarget();

//   // 2. Pivot Turn Right 90 Degrees
//   Serial.println("Action: Rotate Right 90°");
//   robot.rotateAngle(90.0, 150, WHEEL_BASE, WHEEL_DIAM);
//   waitForTarget();

//   // 3. Move Backward 500mm
//   Serial.println("Action: Backward 500mm");
//   robot.moveDistance(-500.0, 150, WHEEL_DIAM);
//   waitForTarget();

//   // 4. Pivot Turn Left 90 Degrees
//   Serial.println("Action: Rotate Left 90°");
//   robot.rotateAngle(-90.0, 150, WHEEL_BASE, WHEEL_DIAM);
//   waitForTarget();

//   Serial.println("--- Full Test Cycle Complete ---");
//   delay(5000); // Pause before repeating
// }

// /**
//  * Helper function to block execution until the robot reaches its target
//  * while printing real-time encoder feedback to the Serial Monitor.
//  */
// void waitForTarget() {
//   while (!robot.updatePositionControl()) {
//     static unsigned long lastLog = 0;
//     if (millis() - lastLog > 200) {
//       Serial.print("L_Pos: ");
//       Serial.print(robot.getLeftPosition());
//       Serial.print(" | R_Pos: ");
//       Serial.print(robot.getRightPosition());
//       Serial.print(" | Speed: ");
//       Serial.print(robot.getLeftSpeed());
//       Serial.println(" RPM");
//       lastLog = millis();
//     }
//     delay(10); // 100Hz update rate
//   }
//   Serial.println("Target Reached.");
//   delay(1000); // Short pause between movements
// }

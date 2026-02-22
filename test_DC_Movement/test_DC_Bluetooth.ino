
/****************************c:\Users\alz00\OneDrive\Desktop\final_rcCar\Final_Project\Autonomous-Mobile-Robot-for-Efficient-Factory-Logistics\test_DC_Movement\test_DC_Bluetooth.ino\test_DC_Bluetooth.ino
**************************************************    Date     :  11/15/2025			*******************************************************
**************************************************    Name     :   Omar Mohamed Hamdy 	*************************************************
**************************************************    Version  :   1.0	            	***************************************************
**************************************************    SWC      :  	test_DC_Bluetooth.ino        ****************************************
*****************************************************************************************************************************************
*****************************************************************************************************************************************
*/

#include <BluetoothSerial.h>
#include "DC_Movement.hpp"

// --- BLUETOOTH ---
BluetoothSerial SerialBT;
String device_name = "ESP32_Robot";
// --- MOTOR PINS ---
#define L_MOTOR_LPWM 25
#define L_MOTOR_RPWM 26
#define R_MOTOR_LPWM 27
#define R_MOTOR_RPWM 32

// --- Create Robot Object ---
RobotDrivetrain robot(L_MOTOR_LPWM, L_MOTOR_RPWM, R_MOTOR_LPWM, R_MOTOR_RPWM);

// --- Control Variables ---
int motorSpeed = 180;     // Default speed (0-255). 180 is safe
char lastDirection = 'S'; // Remember last movement command

void setup()
{
  Serial.begin(115200);

  // Start Bluetooth
  SerialBT.begin(device_name);
  Serial.println("=====================================");
  Serial.println("Bluetooth Robot Ready!");
  Serial.printf("Name: %s\n", device_name.c_str());
  Serial.println("Pair your phone and open a robot app");
  Serial.println("=====================================");

  // Initialize motors
  robot.init();
  robot.stop(); // Make sure motors are off at start
  delay(500);
}

void loop()
{
  if (SerialBT.available())
  {
    char cmd = SerialBT.read();

    // Ignore newline/carriage return
    if (cmd == '\n' || cmd == '\r')
      return;

    Serial.print("Received: ");
    Serial.println(cmd);

    // --- SPEED CONTROL (0-9) ---
    if (cmd >= '0' && cmd <= '9')
    {
      int level = cmd - '0';                 // '0' → 0, '9' → 9
      motorSpeed = map(level, 0, 9, 0, 255); // Convert to 0-255  //map(value, fromLow, fromHigh, toLow, toHigh)
      Serial.printf("Speed level %d → PWM %d\n", level, motorSpeed);

      // Re-apply last direction with new speed
      applyLastDirection();
      return;
    }

    // --- MOVEMENT COMMANDS ---
    switch (cmd)
    {
    case 'F':
    case 'f': // Forward
      lastDirection = 'F';
      robot.moveForward(motorSpeed);
      Serial.println("Forward");
      break;

    case 'B':
    case 'b': // Backward
      lastDirection = 'B';
      robot.moveBackward(motorSpeed);
      Serial.println("Backward");
      break;

    case 'L':
    case 'l': // Turn Left
      lastDirection = 'L';
      robot.turnLeft(motorSpeed);
      Serial.println("Turn Left");
      break;

    case 'R':
    case 'r': // Turn Right
      lastDirection = 'R';
      robot.turnRight(motorSpeed);
      Serial.println("Turn Right");
      break;

    case 'G':
    case 'g': // Forward Left
      lastDirection = 'G';
      robot.forwardLeft(motorSpeed);
      Serial.println("Forward Left");
      break;

    case 'I':
    case 'i': // Forward Right
      lastDirection = 'I';
      robot.forwardRight(motorSpeed);
      Serial.println("Forward Right");
      break;

    case 'H':
    case 'h': // Backward Left
      lastDirection = 'H';
      robot.backwardLeft(motorSpeed);
      Serial.println("Backward Left");
      break;

    case 'J':
    case 'j': // Backward Right
      lastDirection = 'J';
      robot.backwardRight(motorSpeed);
      Serial.println("Backward Right");
      break;

    // --- STOP COMMANDS (most apps send one of these) ---
    case 'S':
    case 's':
    case 'D':
    case 'd':
    case 'V':
    case 'v': // Some apps send 'V' when joystick released
    case 'X':
    case 'x':
      lastDirection = 'S';
      robot.stop();
      Serial.println("STOP");
      break;

    default:
      Serial.printf("Unknown command: %c \n", cmd);
      break;
    }
  }

  // Small delay to prevent watchdog issues & reduce CPU usage
  delay(10);
}

// --- Helper: Re-apply last movement when speed changes ---
void applyLastDirection()
{
  switch (lastDirection)
  {
  case 'F':
    robot.moveForward(motorSpeed);
    break;
  case 'B':
    robot.moveBackward(motorSpeed);
    break;
  case 'L':
    robot.turnLeft(motorSpeed);
    break;
  case 'R':
    robot.turnRight(motorSpeed);
    break;
  case 'G':
    robot.forwardLeft(motorSpeed);
    break;
  case 'I':
    robot.forwardRight(motorSpeed);
    break;
  case 'H':
    robot.backwardLeft(motorSpeed);
    break;
  case 'J':
    robot.backwardRight(motorSpeed);
    break;
  default:
    robot.stop();
    break;
  }
}
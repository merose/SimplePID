// AStarPIDTest - Test the PID control for an AStar-based robot.
//
// To gather PID tuning data for graphing, put the robot on blocks, then
// start this program and open a Serial Monitor. Press S4 to start both
// motors, let them come to a steady speed, then press S4 to stop the
// motors. Copy the Serial output to a spreadsheet and plot target speed and
// current speed versus time. You should see a classic PID controller graph
// with some overshoot.

#include <AStar32U4.h>
#include <EnableInterrupt.h>
#include <SimplePID.h>

// Motor parameters. These are based on a Pololu #2285 motor with 48cpr
// encoder.
const float GEAR_RATIO = 210.59;
const float ENCODER_CPR = 12;

// Number of encoder ticks per revolution of the wheel.
const float TICKS_PER_REV = GEAR_RATIO * ENCODER_CPR;

// Maximum desired motor speed. Pololu #2285 are rated for 130rpm, but we
// don't drive them with enough voltage to achieve that.
const float MAX_REVS_PER_SEC = 100.0 / 60.0;

// Use 60% of maximum speed for PID tuning.
const int PID_TUNING_TICKS_PER_SEC = 0.6 * MAX_REVS_PER_SEC * TICKS_PER_REV;

// DFRobot Romeo v2 and BLE PIN assignments.

const int ONBOARD_SWITCH_PIN = A7;

// Pins for the Pololu motor encoder outputs.
const int M1_A = 7;
const int M1_B = 11;
const int M2_A = 15;
const int M2_B = 16;

// Minimum motor control value. Motor output below this will stall.
const int MIN_MOTOR_CMD = 10;

// Maximum motor control value.
const int MAX_MOTOR_CMD = 400;

AStar32U4Motors motors;

// These objects provide access to the A-Star's on-board
// buttons.  We will only use buttonA.
AStar32U4ButtonA buttonA;
AStar32U4ButtonB buttonB;
AStar32U4ButtonC buttonC;

enum { NO_BUTTON, BUTTON_A, BUTTON_B, BUTTON_C };

int leftSpeedTarget = 0;
int rightSpeedTarget = 0;

volatile long leftTicks;
volatile long rightTicks;

long lastLeftTicks = 0;
long lastRightTicks = 0;

int leftMotorCmd = 0;
int rightMotorCmd = 0;

unsigned long lastLoopTime;

unsigned long lastSwitchTime;

float loopTime = 0.0;

// Ziegler-Nichols tuning. See this Wikipedia article for details:
//     https://en.wikipedia.org/wiki/PID_controller#Loop_tuning
// To tune the controller, set USE_KU_ONLY to 1 and increase Ku
// until oscillation occurs, and set Tu to the oscillation period
// in seconds. Then set USE_KU_ONLY to zero to use the
// Ziegler-Nichols tune. To get a less agressive tune, set LESS_AGGRESSIVE
// to 1.

// Set to 1 to use Ku only, to determine oscillation point.
#define USE_KU_ONLY 0

const float Ku = .15;
const float Tu = .1142857143;

#define MANUAL_TUNE 1
#define LESS_AGGRESSIVE 0

#if MANUAL_TUNE
// Found empirically to rapidly converge with little overshoot.
// There is no integral constant, but not needed when driving
// on flat ground, and when the heading is otherwise controlled
// by higher-level software. (I.e., there's no need to catch up
// for past errors.)
const float Kp = 0.07;
const float Ki = 0.0;
const float Kd = 0.001;
#elif !LESS_AGGRESSIVE
const float Kp = 0.6*Ku;
const float Ki = 2*Kp/Tu;
const float Kd = Kp*Tu/8;
#else
const float Kp = 0.2*Ku;
const float Ki = 2*Kp/Tu;
const float Kd = Kp*Tu/3;
#endif

#if USE_KU_ONLY
SimplePID leftController = SimplePID(Ku, 0, 0);
SimplePID rightController = SimplePID(Ku, 0, 0);
#else
SimplePID leftController = SimplePID(Kp, Ki, Kd);
SimplePID rightController = SimplePID(Kp, Ki, Kd);
#endif

// Sets up the serial output and the motor control pins, and attaches
// interrupt handlers to the pins on which we will read the encoder
// phototransistor values.
void setup() {
  Serial.begin(115200);
  delay(3000);

  enableInterrupt(M1_A, leftAChange, CHANGE);
  enableInterrupt(M1_B, leftBChange, CHANGE);
  enableInterrupt(M2_A, rightAChange, CHANGE);
  enableInterrupt(M2_B, rightBChange, CHANGE);

//  Serial.print("Time (s)\t");
  Serial.print("Left Target\tLeft Speed");
//  Serial.print("\tLeft Cum Error\tLeft Motor\t");
  Serial.print("\tRight Target\tRight Speed");
//  Serial.print("\tRight Cum Error\tRight Motor");
  Serial.println();

  lastLoopTime = micros();
  lastSwitchTime = millis();

  leftTicks = rightTicks = 0;
}

// Loops forever showing the motor speeds and errors since the last loop,
// and adjusts the target speeds depending on whether the user is pressing
// switches S2 through S5, as indicated in the code below.
void loop() {
  delay(15);

  unsigned long curLoopTime = micros();
  noInterrupts();
  long curLeftTicks = leftTicks;
  long curRightTicks = rightTicks;
  interrupts();

  float dt = (curLoopTime - lastLoopTime) / 1E6;
  
  float leftSpeed = (curLeftTicks - lastLeftTicks) / dt;
  float rightSpeed = (curRightTicks - lastRightTicks) / dt;

  int leftControl = leftController.getControlValue(leftSpeed, dt);
  leftMotorCmd += min(MAX_MOTOR_CMD, leftControl);
  leftMotorCmd = constrain(leftMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  if (leftMotorCmd > 0) {
    leftMotorCmd = max(leftMotorCmd, MIN_MOTOR_CMD);
  }
  
  int rightControl = rightController.getControlValue(rightSpeed, dt);
  rightMotorCmd += min(MAX_MOTOR_CMD, rightControl);
  rightMotorCmd = constrain(rightMotorCmd, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  if (rightMotorCmd > 0) {
    rightMotorCmd = max(rightMotorCmd, MIN_MOTOR_CMD);
  }

  // Coast to a stop if target is zero.
  if (leftSpeedTarget == 0) {
    leftMotorCmd = 0;
  }
  if (rightSpeedTarget == 0) {
    rightMotorCmd = 0;
  }
  
  setSpeed(leftMotorCmd, rightMotorCmd);

  if (leftSpeedTarget > 0 || leftSpeed > 0 || rightSpeedTarget > 0 || rightSpeed > 0) {
//    Serial.print(loopTime, 3);
//    Serial.print("\t");
    Serial.print(leftSpeedTarget);
    Serial.print("\t");
    Serial.print(leftSpeed);
    Serial.print("\t");
//    Serial.print(leftController.getCumulativeError());
//    Serial.print("\t");
//    Serial.print(leftMotorCmd);
//    Serial.print("\t");
  
    Serial.print(rightSpeedTarget);
    Serial.print("\t");
    Serial.print(rightSpeed);
    Serial.print("\t");
//    Serial.print(rightController.getCumulativeError());
//    Serial.print("\t");
//    Serial.print(rightMotorCmd);
//    Serial.print("\t");
  
    Serial.println();

    loopTime += dt;
}

  // Ignore switches for a short time after pressing, to avoid bounce.
  if (curLoopTime - lastSwitchTime > 0.5*1E6) {
    int switchValue = readSwitch();
    if (switchValue > 0) {
      lastSwitchTime = curLoopTime;

      switch (switchValue) {
        case BUTTON_A:
          // Left motor on/off.
          leftSpeedTarget = (leftSpeedTarget==0 ? PID_TUNING_TICKS_PER_SEC : 0);
          break;
        case BUTTON_B:
          // Right motor on/off.
          rightSpeedTarget = (rightSpeedTarget==0 ? PID_TUNING_TICKS_PER_SEC : 0);
          break;
        case BUTTON_C:
          // Both motors on/off.
          leftSpeedTarget = (leftSpeedTarget==0 ? PID_TUNING_TICKS_PER_SEC : 0);
          rightSpeedTarget = (rightSpeedTarget==0 ? PID_TUNING_TICKS_PER_SEC : 0);
          break;
        default:
          // no change in speed
          break;
      }

      leftController.setSetPoint(leftSpeedTarget);
      rightController.setSetPoint(rightSpeedTarget);
    }    
  }

  lastLeftTicks = curLeftTicks;
  lastRightTicks = curRightTicks;
  lastLoopTime = curLoopTime;
}

void waitForSwitch1() {
  while (readSwitch() != 1) {
    // do nothing
  }
}

void setSpeed(int leftSpeed, int rightSpeed) {
  motors.setSpeeds(leftSpeed, rightSpeed);
}

void leftAChange() {
  if (digitalRead(M1_A) == digitalRead(M1_B)) {
    ++leftTicks;
  } else {
    --leftTicks;
  }
}

void leftBChange() {
  if (digitalRead(M1_A) != digitalRead(M1_B)) {
    ++leftTicks;
  } else {
    --leftTicks;
  }
}

void rightAChange() {
  if (digitalRead(M2_A) != digitalRead(M2_B)) {
    ++rightTicks;
  } else {
    --rightTicks;
  }
}

void rightBChange() {
  if (digitalRead(M2_A) == digitalRead(M2_B)) {
    ++rightTicks;
  } else {
    --rightTicks;
  }
}

int readSwitch() {
  if (buttonA.getSingleDebouncedRelease()) {
    return BUTTON_A;
  } else if (buttonB.getSingleDebouncedRelease()) {
    return BUTTON_B;
  } else if (buttonC.getSingleDebouncedRelease()) {
    return BUTTON_C;
  } else {
    return 0;
  }
}

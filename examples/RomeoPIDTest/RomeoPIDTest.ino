// RomeoPIDTest - Test the PID control for a DFRobot Romeo-based robot.
//
// To gather PID tuning data for graphing, put the robot on blocks, then
// start this program and open a Serial Monitor. Press S4 to start both
// motors, let them come to a steady speed, then press S4 to stop the
// motors. Copy the Serial output to a spreadsheet and plot target speed and
// current speed versus time. You should see a classic PID controller graph
// with some overshoot.

#include <SimplePID.h>

// Motor parameters. These are based on a Pololu #2285 motor with 48cpr
// encoder.
const float GEAR_RATIO = 46.85;
const float ENCODER_CPR = 48;

// Only using A changes to get ticks. Remove the "/ 2.0" if using both A
// and B changes.
const float TICKS_PER_REV = GEAR_RATIO * ENCODER_CPR / 2.0;

// Maximum desired motor speed. Pololu #2285 are rated for 130rpm, but we
// don't drive them with enough voltage to achieve that.
const float MAX_REVS_PER_SEC = 100.0 / 60.0;

// Use 60% of maximum speed for PID tuning.
const int PID_TUNING_TICKS_PER_SEC = 0.6 * MAX_REVS_PER_SEC * TICKS_PER_REV;

// DFRobot Romeo v2 and BLE PIN assignments.

const int ONBOARD_SWITCH_PIN = A0;

const int LEFT_DIRECTION = 4;
const int LEFT_SPEED = 5;
const int RIGHT_SPEED = 6;
const int RIGHT_DIRECTION = 7;

// Minimum motor control value. Motor output below this will stall.
const int MIN_MOTOR_CMD = 60;

int leftSpeedTarget = 0;
int rightSpeedTarget = 0;

volatile unsigned long leftTicks = 0;
volatile unsigned long lastLeftTickTime = 0;
volatile unsigned long rightTicks = 0;
volatile unsigned long lastRightTickTime = 0;

int lastLeftTicks = 0;
int lastRightTicks = 0;

int leftMotorCmd = 0;
int rightMotorCmd = 0;

unsigned long lastLoopTime;

unsigned long lastSwitchTime;

float loopTime = 0.0;

// Ziegler-Nichols tuning. See this Wikipedia article for details:
//     https://en.wikipedia.org/wiki/PID_controller#Loop_tuning
// Ku and Tu were determined by setting Ki and Kd to zero, then increasing
// Kp until steady oscillation occurs. Tu is the oscillation wavelength.
const float Ku = .19;
const float Tu = .23;
const float Kp = 0.6*Ku;
const float Ki = 2*Kp/Tu;
const float Kd = Kp*Tu/8;

SimplePID leftController = SimplePID(Kp, Ki, Kd);
SimplePID rightController = SimplePID(Kp, Ki, Kd);

// Sets up the serial output and the motor control pins, and attaches
// interrupt handlers to the pins on which we will read the encoder
// phototransistor values.
void setup() {
  Serial.begin(115200);
  delay(3000);

  pinMode(LEFT_DIRECTION, OUTPUT);
  pinMode(LEFT_SPEED, OUTPUT);
  pinMode(RIGHT_DIRECTION, OUTPUT);
  pinMode(RIGHT_SPEED, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(2), handleLeftTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), handleRightTick, CHANGE);

  Serial.print("Time\tLeft Target\tLeft Speed\tLeft Cum Error\tLeft Motor");
  Serial.print("\tRight Target\tRightSpeed\tRight Cum Error\tRightMotor");
  Serial.println();

  lastLoopTime = micros();
  lastSwitchTime = millis();
}

// Loops forever showing the motor speeds and errors since the last loop,
// and adjusts the target speeds depending on whether the user is pressing
// switches S2 through S5, as indicated in the code below.
void loop() {
  delay(20);

  unsigned long curLoopTime = micros();
  noInterrupts();
  int curLeftTicks = leftTicks;
  int curRightTicks = rightTicks;
  interrupts();

  float dt = (curLoopTime - lastLoopTime) / 1E6;
  
  float leftSpeed = (curLeftTicks - lastLeftTicks) / dt;
  float rightSpeed = (curRightTicks - lastRightTicks) / dt;

  int leftControl = leftController.getControlValue(leftSpeed, dt);
  leftMotorCmd += min(255, leftControl);
  leftMotorCmd = constrain(leftMotorCmd, -255, 255);
  if (leftMotorCmd > 0) {
    leftMotorCmd = max(leftMotorCmd, MIN_MOTOR_CMD);
  }
  
  int rightControl = rightController.getControlValue(rightSpeed, dt);
  rightMotorCmd += min(255, rightControl);
  rightMotorCmd = constrain(rightMotorCmd, -255, 255);
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
    Serial.print(loopTime);
    Serial.print("\t");
    Serial.print(leftSpeedTarget);
    Serial.print("\t");
    Serial.print(leftSpeed);
    Serial.print("\t");
    Serial.print(leftController.getCumulativeError());
    Serial.print("\t");
    Serial.print(leftMotorCmd);
    Serial.print("\t");
  
    Serial.print(rightSpeedTarget);
    Serial.print("\t");
    Serial.print(rightSpeed);
    Serial.print("\t");
    Serial.print(rightController.getCumulativeError());
    Serial.print("\t");
    Serial.print(rightMotorCmd);
    Serial.print("\t");
  
    Serial.println();

    loopTime += dt;
}

  // Ignore switches for a short time after pressing, to avoid bounce.
  if (curLoopTime - lastSwitchTime > 0.5*1E6) {
    int switchValue = readSwitch();
    if (switchValue > 0) {
      lastSwitchTime = curLoopTime;

      switch (switchValue) {
        case 2:
          // Left motor on/off.
          leftSpeedTarget = (leftSpeedTarget==0 ? PID_TUNING_TICKS_PER_SEC : 0);
          break;
        case 3:
          // Right motor on/off.
          rightSpeedTarget = (rightSpeedTarget==0 ? PID_TUNING_TICKS_PER_SEC : 0);
          break;
        case 4:
          // Both motors on/off.
          leftSpeedTarget = (leftSpeedTarget==0 ? PID_TUNING_TICKS_PER_SEC : 0);
          rightSpeedTarget = (rightSpeedTarget==0 ? PID_TUNING_TICKS_PER_SEC : 0);
          break;
        case 5:
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
  digitalWrite(LEFT_DIRECTION, (leftSpeed >= 0 ? HIGH : LOW));
  analogWrite(LEFT_SPEED, abs(leftSpeed));
  digitalWrite(RIGHT_DIRECTION, (rightSpeed >= 0 ? HIGH : LOW));
  analogWrite(RIGHT_SPEED, abs(rightSpeed));
}

void handleLeftTick() {
  if (digitalRead(2) == digitalRead(8)) {
    ++leftTicks;
  } else {
    --leftTicks;
  }
}

void handleRightTick() {
  if (digitalRead(3) != digitalRead(9)) {
    ++rightTicks;
  } else {
    --rightTicks;
  }
}

// Reads the value of the built-in switches S1 through S5. The switches
// are connected in parallel with resistors of five different sizes to
// analog pin A0. The difference in voltage at A0 allows us to determine
// which switch is pressed.
int readSwitch() {
  int value = analogRead(ONBOARD_SWITCH_PIN);

  if (value > 800) {
    return 0;
  } else if (value > 600) {
    return 5;
  } else if (value > 400) {
    return 4;
  } else if (value > 250) {
    return 3;
  } else if (value > 75) {
    return 2;
  } else {
    return 1;
  }
}


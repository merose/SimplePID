#include <float.h>
#include "SimplePID.h"

SimplePID::SimplePID(float Kp, float Ki, float Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  minOutput = -FLT_MAX;
  maxOutput = FLT_MAX;
  
  sumError = 0.0;
  lastActual = 0.0;
}

void SimplePID::setConstants(float Kp, float Ki, float Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}
  
void SimplePID::setOutputRange(float minOutput, float maxOutput) {
  this->minOutput = minOutput;
  this->maxOutput = maxOutput;
}

void SimplePID::setSetPoint(float setPoint) {
  this->setPoint = setPoint;
  sumError = 0.0;
}

float SimplePID::getCumulativeError() {
  return sumError;
}

void SimplePID::clearCumulativeError() {
  sumError = 0.0;
}

float SimplePID::getControlValue(float actual, float dt) {
  float error = setPoint - actual;

  float newSum = sumError + error*dt;

  // The derivative is calculated by assuming the two setpoints are
  // equal. This works better when changing the setpoint because the
  // derivative error does not suddenly increase.
  float dErrorDt = (lastActual - actual) / dt;
  lastActual = actual;

  float output = Kp*error + Ki*sumError + Kd*dErrorDt;

  if (output >= maxOutput) {
    return maxOutput;
  } else if (output <= minOutput) {
    return minOutput;
  } else {
    sumError = newSum;
    return output;
  }
}

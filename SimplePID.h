// SimplePID.h - Definitions for the simple PID library.

#ifndef SIMPLE_PID_H
#define SIMPLE_PID_H

class SimplePID {
  public:

    SimplePID(float Kp, float Ki, float Kd);

    void setConstants(float Kp, float Ki, float Kd);
    
    void setOutputRange(float minOutput, float maxOutput);

    void setSetPoint(float setPoint);

    float getCumulativeError();

    void clearCumulativeError();

    float getControlValue(float actual, float dt);

  private:
  
    float Kp;
    float Ki;
    float Kd;
    float setPoint;
    float minOutput;
    float maxOutput;
    float lastActual;
    float sumError;

};

#endif SIMPLE_PID_H

#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float outputMin, float outputMax, bool degrees);
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setIntegralWindupLimit(float limit);
    void setIntegralZeroingLimit(float ZeroLimit);
    void reset();
    float compute(float setpoint, float input, float dt);
    float getDelta();


private:
    float _kp; // Proportional gain
    float _ki; // Integral gain
    float _kd; // Derivative gain
    float _integral; // Integral term
    float _zeroLimit;
    float _previousError; // Previous error
    float _outputMin; // Minimum output limit
    float _outputMax; // Maximum output limit
    float _integralWindupLimit; // Integral windup limit
    bool _degrees;
    double _delta;
    int lastMillis;
};

#endif // PID_H

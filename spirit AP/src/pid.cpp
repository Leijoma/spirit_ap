#include "PID.h"
#include "logger.h"

PID::PID(float kp, float ki, float kd, float outputMin, float outputMax, bool degrees)
    : _degrees(true) ,_kp(kp), _ki(ki), _kd(kd), _integral(0), _previousError(0), _outputMin(outputMin), _outputMax(outputMax), _integralWindupLimit(outputMax), _zeroLimit(0) {}

void PID::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setOutputLimits(float min, float max) {
    _outputMin = min;
    _outputMax = max;
    _integralWindupLimit = max;
}

void PID::setIntegralWindupLimit(float limit) {
    _integralWindupLimit = limit;
}


void PID::setIntegralZeroingLimit(float ZeroLimit) {
    _zeroLimit = ZeroLimit;
}
void PID::reset() {
    _integral = 0;
    _previousError = 0;
}

  float PID::getDelta() {
    return _delta;
  }

float PID::compute(float setpoint, float input, float dt) {
    // Update delta value
    _delta=0.9*_delta+0.1*(millis()-lastMillis);
    lastMillis=millis();
    
    double error = setpoint - input;
    if (_degrees) {
         if (error > 180) error -= 360;
         if (error < -180) error += 360;
    }
    // Proportional term
    float pTerm = _kp * error;
    float ddt=100;
    // Integral term with anti-windup
    _integral = _integral+ (error* _ki * ddt);
     if (_integral > _integralWindupLimit) {
        _integral = _integralWindupLimit;
    } else if (_integral < -_integralWindupLimit) {
        _integral = -_integralWindupLimit;
    }
    if ((error<_zeroLimit) && (error>-_zeroLimit) && (_zeroLimit!=0))
        _integral=0;


    float iTerm = _integral;
 
    // Derivative term
    float dTerm = 0;
    if (dt > 0) {
        dTerm = _kd * (error - _previousError) / dt;
    }
    _previousError = error;

    // Compute the final output and apply limits
    float output = pTerm + iTerm + dTerm;
    if (output > _outputMax) {
        output = _outputMax;
    } else if (output < _outputMin) {
        output = _outputMin;
    }

    //  Logger::logf("error   = %.2f", error);
    //  Logger::logf("dt   = %.2f", dt);
    //  Logger::logf("_integral   = %.2f", _integral);
    //  Logger::logf("iTerm   = %.2f", iTerm);
    //  Logger::logf("kp = %.2f ki = %.2f kd =  %.2f", _kp,_ki,_kd);
    //  Logger::logf("p = %.2f i = %.2f d =  %.2f", pTerm,iTerm,dTerm);
    return output;
}

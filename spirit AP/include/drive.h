#ifndef DRIVE_H
#define DRIVE_H

#include "PID.h"

#define PWM_FREQUENCY 50 // Typical servo PWM frequency (50Hz)
#define PWM_CHANNEL 0    // Use PWM channel 0
#define PWM_RESOLUTION 16 // Resolution of PWM (up to 16 bits)

class RudderDriver {

public:
    RudderDriver();  // Constructor
    void init(double drive_p, double drive_i, double drive_d, int pwmPin, int adPin, int clutchPin);     // Initialize the rudder driver
    void setPIDparameters(double drive_p, double drive_i, double drive_d,double drive_i_limit);
    void setTargetAngle(float angle);  // Set the rudder angle
    float getTargetAngle();  // Set the rudder angle
    
    void setPwmParameters(int pwmCenter, int pwmMin, int pwmMax);
    float getCurrentAngle();
    void update();
    void clutchEngage();
    void clutchDisengage();
    bool ClutchEngaged();
    float getDelta();
    float doDeadzoneTest(int step);

private:
    int updatePeriod;
    int lastUpdate;
    float currentAngle;  // Current rudder angle
    float targetAngle;  // Current rudder angle
    int ad_pin;
    int pwm_pin;
    int clutch_pin;
    float rudder_p;
    float rudder_i;
    float rudder_d;
    float rudder_i_limit;
    bool clutch_engaged;
    float rudder_average;

    int _pwm_center;
    int _pwm_min;
    int _pwm_max;

    PID _rudderPID;
};

extern RudderDriver rudder_driver;  // Declare the default object

#endif // DRIVE_H

#include "drive.h"
#include "logger.h"
#include <Arduino.h>



RudderDriver::RudderDriver() : currentAngle(0.0),  _rudderPID(1,0,0,-1000,1000,false), clutch_engaged(false), _pwm_center(1400),_pwm_min(800), _pwm_max(2000) {
    // Constructor
     _pwm_center=1400;
     _pwm_max=2000;
     _pwm_min=800;
     updatePeriod=100; // ms between updates

}

void RudderDriver::init(double drive_p, double drive_i, double drive_d, int pwmPin, int adPin, int clutchPin) {
     
    ad_pin=adPin;
    pwm_pin=pwmPin;
    clutch_pin=clutchPin;

    clutchDisengage();

    pinMode(clutch_pin, OUTPUT);  // Example pin setup for a rudder servo
    pinMode(pwm_pin, OUTPUT);  // Example pin setup for a rudder servo
    pinMode(ad_pin, INPUT);  // Example pin setup for a rudder servo  
    
    rudder_p=drive_p;
    rudder_i=drive_i;
    rudder_d=drive_d;
  
    _rudderPID.setIntegralWindupLimit(200);
    _rudderPID.setIntegralZeroingLimit(0.5);
   

    // Set up the PWM properties
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  
    // Attach the PWM channel to the specified pin
    ledcAttachPin(pwm_pin, PWM_CHANNEL);
    
    // Start esc with three second delay with angle 0
    ledcWrite(PWM_CHANNEL, _pwm_center * (65536 / 20000)); // Convert microseconds to duty cycle for 16-bit resolution
    delay(3000);
 
}

float RudderDriver::getDelta() {
    return _rudderPID.getDelta();
}

void RudderDriver::setPIDparameters(double drive_p, double drive_i, double drive_d,double drive_i_limit) {
    rudder_p=drive_p;
    rudder_i=drive_i;
    rudder_d=drive_d;
    rudder_i_limit=drive_i_limit;
    _rudderPID.setTunings(rudder_p,rudder_i, rudder_d);
    _rudderPID.setIntegralWindupLimit(rudder_i_limit);
}

void RudderDriver::setTargetAngle(float angle) {
    // Set the rudder angle
    targetAngle = angle;
   
}

float RudderDriver::getCurrentAngle() {
   return currentAngle;
}

float RudderDriver::getTargetAngle() {
   return targetAngle;
}

void RudderDriver::setPwmParameters(int pwmCenter, int pwmMin, int pwmMax) {
   if (pwmCenter != 0)
    _pwm_center=pwmCenter;
   if (pwmMin != 0)
    _pwm_min=pwmMin;
   if (pwmMax != 0)
    _pwm_max=pwmMax;
}

void RudderDriver::clutchEngage() {
   ledcAttachPin(pwm_pin, PWM_CHANNEL);
   digitalWrite(clutch_pin, HIGH);
   clutch_engaged=true;
}

void RudderDriver::clutchDisengage() {
    digitalWrite(clutch_pin, LOW);
    clutch_engaged=false;
    ledcDetachPin(pwm_pin);
}

bool RudderDriver::ClutchEngaged() {
    return clutch_engaged;
}

float RudderDriver::doDeadzoneTest(int step) {
    float pwm=_pwm_center;
    float analogValue;
    clutchEngage();
    while ((currentAngle<10) && (pwm<_pwm_max)) {    
        analogValue = analogRead(ad_pin);
        currentAngle=(analogValue-1735)*0.08;
        rudder_average=rudder_average*0.9+currentAngle*0.1;
        ledcWrite(PWM_CHANNEL, pwm * (65536 / 20000)); // Convert microseconds to duty cycle for 16-bit resolution
        pwm=pwm+step;
        Serial.print("pwm: ");
        Serial.print(pwm);
        Serial.print("  currentAngle: ");
        Serial.println(currentAngle);
        delay(200);
    }
    delay(2000);
    pwm=_pwm_center;
    while ((currentAngle>0) && (pwm>_pwm_min)) {    
        analogValue = analogRead(ad_pin);
        currentAngle=(analogValue-1735)*0.08;
        rudder_average=rudder_average*0.9+currentAngle*0.1;
        ledcWrite(PWM_CHANNEL, pwm * (65536 / 20000)); // Convert microseconds to duty cycle for 16-bit resolution
        pwm=pwm-step;
        Serial.print("pwm: ");
        Serial.print(pwm);
        Serial.print("  currentAngle: ");
        Serial.println(currentAngle);
        delay(200);
    }
    ledcWrite(PWM_CHANNEL, _pwm_center * (65536 / 20000)); // Convert microseconds to duty cycle for 16-bit resolution
    clutchDisengage();
}

void  RudderDriver::update() {
    int timeNow=millis();
    float dt=(float)(timeNow-lastUpdate);
    if (timeNow>(lastUpdate+updatePeriod)) {
       // Logger::logf("update period  = %.2f", dt);
        lastUpdate=timeNow;
        float analogValue = analogRead(ad_pin);
        currentAngle=(analogValue-1735)*0.08;
        rudder_average=rudder_average*0.9+currentAngle*0.1;

        //Logger::logf("Rudder angle  = %.0f", rudder_average);

        if (targetAngle>20)
            targetAngle=20;
        if (targetAngle<-20)
            targetAngle=-20;
        /*
        if (currentAngle>45)
            currentAngle=45;
        if (currentAngle<-45)
            currentAngle=-45;
        */

        float pidOutput=_rudderPID.compute(targetAngle,currentAngle,dt); 
         // Map angle to servo PWM range
        float pwmValue = _pwm_center+pidOutput; 
        
        // Limit pwm value
        if (pwmValue>_pwm_max)
            pwmValue=_pwm_max;
        if (pwmValue<_pwm_min)
            pwmValue=_pwm_min;

         // Logger::logf("Rudder angle = %.2f target angle  = %.2f update period = %.2f", currentAngle, targetAngle,dt);
         // Logger::logf("%.2f,%.2f,%.2f", currentAngle, targetAngle,dt);
          
        // Logger::logf("target angle  = %.2f", targetAngle);
        //  Logger::logf("pidout   = %.2f", pidOutput);
        //  Logger::logf("pwm value  = %.2f", pwmValue);
        //  Logger::logf("pwm cengter  = %.2f", (float)_pwm_center);
        //  Logger::logf("pwm p  = %.2f", (float)rudder_p);

        ledcWrite(PWM_CHANNEL, pwmValue * (65536 / 20000)); // Convert microseconds to duty cycle for 16-bit resolution
}
}
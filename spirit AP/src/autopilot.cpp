#include "autopilot.h"
#include "logger.h"
#include <cmath>
#include <Arduino.h>


Autopilot::Autopilot(AutopilotData &autopilotdata, RudderDriver* driveInstance)
    : autopilotdata(autopilotdata), _lastUpdateTime(0), _updateRate(100),
      _KpHeading(1.0), _KiHeading(0.1), _KdHeading(0.0), _KmaxIHeading(0.0),
      _headingIntegral(0), _headingDerivative(0), _headingPreviousError(0),
      _KpTurnRate(0.0), _KiTurnRate(0), _KdTurnRate(0), _KmaxITurn(0.0),
      _turnRateIntegral(5), _turnRateDerivative(0), _turnRatePreviousError(0),      
      _headingPID(2.0, 0.2, 0.0, -20, 20, true),_turnRatePID(1.0, 0, 0.0, -20, 20, false), _xtePID(20,0,0,-20,20, false),
      _targetRudderAngle(0), _rudderControlEnabled(false) 
{
    _headingPID.setIntegralWindupLimit(10);
    _turnRatePID.setIntegralWindupLimit(10);
    _xtePID.setIntegralWindupLimit(10);
    drive = driveInstance;
    
}

 

void Autopilot::setUpdateRate(int rate) {
   _updateRate=rate;
}

void Autopilot::setHeadingPID_P(float p) {
    _KpHeading=p;
    _headingPID.setTunings(_KpHeading,_KiHeading,_KdHeading);
}

void Autopilot::setHeadingPID_I(float i) {
    _KiHeading=i;
    _headingPID.setTunings(_KpHeading,_KiHeading,_KdHeading);
}

void Autopilot::setHeadingPID_maxI(float i) {
    _KmaxIHeading=i;
    _headingPID.setIntegralWindupLimit(i);
}

void Autopilot::setHeadingPID_D(float d) {
    _KdHeading=d;
    _headingPID.setTunings(_KpHeading,_KiHeading,_KdHeading);
}


void Autopilot::setTurnPID_maxI(float i) {
    _KmaxITurn=i;
}


void Autopilot::setTurnPID_P(float p) {
    _KpTurnRate=p;
}

void Autopilot::setTurnPID_I(float i) {
    _KiTurnRate=i;
}

void Autopilot::setTurnPID_D(float d) {
    _KdTurnRate=d;
}


void Autopilot::setXTEPID_maxI(float i) {
    _KmaxIXTE=i;
    _xtePID.setIntegralWindupLimit(_KmaxIXTE);
}


void Autopilot::setXTEPID_P(float p) {
    _KpXTE=p;
    _xtePID.setTunings(_KpXTE, _KiXTE, _KdXTE);
}

void Autopilot::setXTEPID_I(float i) {
    _KiXTE=i;
    _xtePID.setTunings(_KpXTE, _KiXTE, _KdXTE);
  //  Logger::logf("XTE I set to = %.2f", _KiXTE);
}

void Autopilot::setXTEPID_D(float d) {
    _KdXTE=d;
    _xtePID.setTunings(_KpXTE, _KiXTE, _KdXTE);
} 

float Autopilot::getTargetRudderAngle() {
    return _targetRudderAngle;
}

void Autopilot::update() {
    unsigned long timeNow=millis();
    float dt=(float)(timeNow-_lastUpdateTime);
    if (timeNow>(_lastUpdateTime+_updateRate)) {
      //  Logger::logf("update period  = %.2f", dt);
        _lastUpdateTime=timeNow;
        float xteOffset;
        // Call appropriate mode handler
        switch (autopilotdata.getMode()) {
            case Standby:
                drive->clutchDisengage();
                handleManualMode();
                break;
            case Auto:
                drive->clutchEngage();
                _targetRudderAngle=_headingPID.compute(autopilotdata.getTargetHeading(),autopilotdata.getActualHeading(),dt);
                drive->setTargetAngle(_targetRudderAngle);
                break;
            case Track:
                drive->clutchEngage();
                xteOffset=0;
                autopilotdata.setTargetHeading(autopilotdata.getDirectionToWaypoint()+xteOffset);
                _targetRudderAngle=_headingPID.compute(autopilotdata.getTargetHeading(),autopilotdata.getActualHeading(),dt);
                drive->setTargetAngle(_targetRudderAngle);
                break;
            case Wind:
                drive->clutchEngage();
                handleWindMode();
                break;
        }
    }
}

void Autopilot::handleManualMode() {
    _targetRudderAngle = 0;
}

void Autopilot::handleHeadingHoldMode() {
    // Minimize heding error
    _targetRudderAngle=_headingPID.compute(autopilotdata.getTargetHeading(),autopilotdata.getActualHeading(),_updateRate);
    drive->setTargetAngle(_targetRudderAngle);
}

void Autopilot::handleTrackMode() {
    // Minimize XTE set targetheading based on bearing to wp and xte offset
    //autopilotdata.setDirectionToWaypoint(calculateBearing(autopilotdata.getCurrentPosition(),autopilotdata.getWaypointPosition()));
    // calculate xte offset
    float xteOffset=0;
    autopilotdata.setTargetHeading(autopilotdata.getDirectionToWaypoint()+xteOffset);
    _targetRudderAngle=_headingPID.compute(autopilotdata.getTargetHeading(),autopilotdata.getActualHeading(),_updateRate);
    drive->setTargetAngle(_targetRudderAngle);
 //    Logger::logf(" actual heading= %.2f target heading= %.2f  target rudder angle= %.2f   ", autopilotdata.getActualHeading(), autopilotdata.getTargetHeading(), _targetRudderAngle);
   
    
}

void Autopilot::handleWindMode() {
    float windError = autopilotdata.getApparentWindAngle() - autopilotdata.getActualHeading();
    if (windError > 180) windError -= 360;
    if (windError < -180) windError += 360;
    _targetRudderAngle = windError * 0.1; // Simplified proportional control
    drive->setTargetAngle(_targetRudderAngle);
      
}

/*
 // Calculate heading compensation based on XTE
       
        float xteCorrection = _xtePID.compute(0, autopilotdata.getCrossTrackError(), _timeStep);
        if (autopilotdata.getDirectionToSteer()=='R')
            xteCorrection=xteCorrection*(-1);
        
        if (xteCorrection > 45) xteCorrection = 45;
        if (xteCorrection < -45) xteCorrection = -45;
    

        // Update the actual heading and turn rate
        float headingError = (autopilotdata.getTargetHeading() + xteCorrection) - autopilotdata.getActualHeading()  ;

        if (headingError > 180) headingError -= 360;
        if (headingError < -180) headingError += 360;

        double actualTurnRate = (autopilotdata.getTargetHeading() - _previousHeading) / _timeStep;
        if (actualTurnRate > 180) actualTurnRate -= 360;
        if (actualTurnRate < -180) actualTurnRate += 360;

        // Compute PID outputs for heading
        _headingIntegral += headingError * _timeStep;
        if (_headingIntegral > _KmaxIHeading) _headingIntegral =_KmaxIHeading;
        if (_headingIntegral < -_KmaxIHeading) _headingIntegral =-_KmaxIHeading;

        _headingDerivative = (headingError - _headingPreviousError) / _timeStep;

        float headingOutput = _KpHeading * headingError + _KiHeading * _headingIntegral + _KdHeading * _headingDerivative;
        _headingPreviousError = headingError;

        // Compute PID outputs for turn rate
        float turnRateError=0;
        float turnRateOutput =0;
        if (abs(headingError)>2) {
          //  if (headingError<0)
                turnRateError = autopilotdata.getTargetTurnRate() - actualTurnRate;
          //  else    
          //      turnRateError =-(_targetTurnRate - actualTurnRate);
            _turnRateIntegral += turnRateError * _timeStep;
        
            if (_turnRateIntegral > _KmaxITurn) _turnRateIntegral =_KmaxITurn;
            if (_turnRateIntegral < -_KmaxITurn) _turnRateIntegral =-_KmaxITurn;

            _turnRateDerivative = (turnRateError - _turnRatePreviousError) / _timeStep;
            turnRateOutput = _KpTurnRate * turnRateError + _KiTurnRate * _turnRateIntegral + _KdTurnRate * _turnRateDerivative;
        }
        else {
           turnRateError=0; 
        }
         _turnRatePreviousError = turnRateError;

       

        // Combine the PID outputs
       _targetRudderAngle=(headingOutput*0.5 + turnRateOutput*0.5);

        // Clamp the rudder angle to a maximum range
        if (_targetRudderAngle > 45) _targetRudderAngle = 45;
        if (_targetRudderAngle < -45) _targetRudderAngle = -45;

      //  Logger::logf("AP heading error= %.2f actual heading= %.2f target heading= %.2f turn rate= %.3f rudder angle= %.2f  turnrate error= %.2f xte comp=%.2f %c", 
      //              headingError, _actualHeading, _targetHeading, actualTurnRate, _rudderAngle, turnRateError,xteCorrection, _directionToSteer);

*/

float Autopilot::calculateBearing(Position currentPos, Position wpPos) {
    float radLat1 = currentPos.lat * M_PI / 180.0;
    float radLon1 = currentPos.lon * M_PI / 180.0;
    float radLat2 = wpPos.lat * M_PI / 180.0;
    float radLon2 = wpPos.lon * M_PI / 180.0;

    float dLon = radLon2 - radLon1;
    float y = sin(dLon) * cos(radLat2);
    float x = cos(radLat1) * sin(radLat2) - sin(radLat1) * cos(radLat2) * cos(dLon);
    float bearing = atan2(y, x) * 180.0 / M_PI;

    return fmod(bearing + 360.0, 360.0);
}

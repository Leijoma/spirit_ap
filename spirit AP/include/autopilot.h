#ifndef AUTOPILOT_H
#define AUTOPILOT_H
#include "PID.h"
#include "autopilotdata.h"
#include "drive.h"



class Autopilot {
public:
    Autopilot(AutopilotData &autopilotdata,RudderDriver* driveInstance);

    void setHeadingPID_P(float p);
    void setHeadingPID_I(float i);
    void setHeadingPID_maxI(float i);
    void setHeadingPID_D(float d);

    void setTurnPID_P(float p);
    void setTurnPID_I(float i);
    void setTurnPID_maxI(float i);
    void setTurnPID_D(float d);

     void setXTEPID_P(float p);
    void setXTEPID_I(float i);
    void setXTEPID_maxI(float i);
    void setXTEPID_D(float d);
        
    void update();
    void setUpdateRate(int rate);
    float getTargetRudderAngle();
   
private:
    AutopilotData &autopilotdata;
    RudderDriver* drive;
    unsigned long _lastUpdateTime;
    unsigned long _updateRate;
    float _timeStep;

    void handleManualMode();
    void handleHeadingHoldMode();
    void handleTrackMode();
    void handleWindMode();
    float calculateBearing(Position currentPos, Position wpPos);

    float _previousHeading;
    float _targetRudderAngle;
    bool _rudderControlEnabled;
    // PID Controller variables
    float _KpHeading, _KiHeading, _KdHeading, _KmaxIHeading;
    float _headingIntegral, _headingDerivative, _headingPreviousError;

    float _KpTurnRate, _KiTurnRate, _KdTurnRate, _KmaxITurn;
    float _KpXTE, _KiXTE, _KdXTE, _KmaxIXTE;
    
    float _turnRateIntegral, _turnRateDerivative, _turnRatePreviousError;

    PID _headingPID;
    PID _turnRatePID;
    PID _xtePID;

};

#endif // AUTOPILOT_H

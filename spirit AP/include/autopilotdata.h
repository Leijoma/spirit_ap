#ifndef AUTOPILOTDATA_H
#define AUTOPILOTDATA_H

enum AutopilotMode {
    Standby,
    Auto,
    Auto_waiting_for_confirmation,
    Track,
    Wind
};

struct Position {
    float lat=0;
    float lon=0;
};

class AutopilotData {
    public:
      

        void setMode(AutopilotMode mode);
        void setActualHeading(float heading);
        void setTargetHeading(float heading);
        void setCrossTrackError(float xte);
        void setDistanceToWaypoint(float distance);
        void setDirectionToWaypoint(float angle);
        void setApparentWindAngle(float wa);
        void setRudderAngle(float ra);
        void setCurrentPosition(float latitude, float longitude);
        void setWaypointPosition(float latitude, float longitude);
        void setTargetTurnRate(float turnRate);
        void setDirectionToSteer(char dts);

        AutopilotMode getMode();
        float getActualHeading();
        float getTargetHeading();
        float getCrossTrackError();
        float getDistanceToWaypoint();
        float getDirectionToWaypoint();
        
        float getApparentWindAngle();
        float getRudderAngle();
        Position getCurrentPosition();
        Position getWaypointPosition();
        float getTargetTurnRate();
        char getDirectionToSteer();
        AutopilotData();
    private:
        AutopilotMode ap_mode;
        float targetHeading;
        float actualHeading;
        float crossTrackError;
        float apparentWindAngle;
        Position currentPos;
        Position wp_position;
        float rudderAngle;
        float wp_distance;
        float wp_bearing;
        float ap_course;
        float targetTurnrate;
        char directionToSteer;


    // Add more shared data as needed

  
};

#endif // AUTOPILOTDATA_H
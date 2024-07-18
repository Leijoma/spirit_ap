#include "autopilotdata.h"

AutopilotData::AutopilotData()
    :  
    currentPos(),
    wp_position(),
    ap_mode(Standby), 
    apparentWindAngle(0),
    actualHeading(0), 
    targetHeading(0), 
    crossTrackError(0),
    rudderAngle(0),
    wp_distance(0),
    wp_bearing(0),
    ap_course(0),
    targetTurnrate(2),
    directionToSteer(0) {
    // Initialize other shared data as needed
    }


        void AutopilotData::setMode(AutopilotMode mode){
            ap_mode=mode;
        }
        void AutopilotData::setActualHeading(float heading){
            actualHeading=heading;
        }
        void AutopilotData::setTurnrate(float rot){
            turnrate=rot;
        }
        
        void AutopilotData::setTargetHeading(float heading){
            targetHeading=heading;
        }
        void AutopilotData::setCrossTrackError(float xte){
            crossTrackError=xte;
        }
        void AutopilotData::setDistanceToWaypoint(float distance){
            wp_distance=distance;
        } 
         void AutopilotData::setDirectionToWaypoint(float angle){
            wp_bearing=angle;
        } 
        void AutopilotData::setApparentWindAngle(float wa){
            apparentWindAngle=wa;
        }
        void AutopilotData::setRudderAngle(float ra){
            rudderAngle=ra;
        }
        void AutopilotData::setCurrentPosition(float latitude, float longitude){
            currentPos.lat=latitude;
            currentPos.lon=longitude;
            
        }
        void AutopilotData::setWaypointPosition(float latitude, float longitude){
            wp_position.lat=latitude;
            wp_position.lon=longitude;
        }
        void AutopilotData::setTargetTurnRate(float turnRate){
            targetTurnrate=turnRate;
        }
        void AutopilotData::setDirectionToSteer(char dts){
            directionToSteer=dts;
        }

        AutopilotMode AutopilotData::getMode(){
            return ap_mode;
        }

        float AutopilotData::getActualHeading(){
            return actualHeading;
        }
        float AutopilotData::getTurnrate(){
            return turnrate;
        }

        float AutopilotData::getTargetHeading(){
            return targetHeading;
        }
        float AutopilotData::getCrossTrackError(){
            return crossTrackError;
        }
        float AutopilotData::getDistanceToWaypoint(){
            return wp_distance;
        } 
        float AutopilotData::getDirectionToWaypoint(){
            return wp_bearing;
        } 
        float AutopilotData::getApparentWindAngle(){
            return apparentWindAngle;
        }
        float AutopilotData::getRudderAngle(){
            return rudderAngle;
        }
        Position AutopilotData::getCurrentPosition(){
            return currentPos;
        }
        Position AutopilotData::getWaypointPosition(){
            return wp_position;
        }
        float AutopilotData::getTargetTurnRate(){
            return targetTurnrate;
        }
        char AutopilotData::getDirectionToSteer(){
            return directionToSteer;
        }
        
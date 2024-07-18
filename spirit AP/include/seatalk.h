#ifndef SEATALK_H
#define SEATALK_H
#define BEEP_DURATION 150 // 150 ms beep time
#define MAX_BUF_SIZE 20
#define RX_IN 18
#define TX_OUT 21
#define LED_PIN 2
#define METRES_TO_FEET 3.28084
#define METRES_TO_FATHONS 0.546807
#define FEET_TO_METERS 0.3048

#include <SoftwareSerial.h>

#include "Commands.h"
#include "autopilotdata.h"

#include <vector>


class SeaTalk
{

public:
    SeaTalk(AutopilotData &autopilotdata);

    const uint8_t ST_APid[3] = {0x90, 0x00, 0x05};
    
    void processMessages();
  
    void sendAP90();
    void sendAP9C();
    void sendAP83(char data );
    void sendAP84();
   
private:
    SoftwareSerial _mySerial;
    AutopilotData &autopilotdata;
    uint8_t AP_mode=0x00;
    int checkBus();
    void checkClearToWrite();
    void printArrayAsHex(uint8_t* array, size_t length);
    bool send2ST(const uint8_t cmd[], int bytes);
    uint8_t calcChecksum(uint8_t x);


};

#endif
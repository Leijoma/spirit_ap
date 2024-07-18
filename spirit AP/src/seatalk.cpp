
#include "SeaTalk.h"

/// @brief Seatalk class constructor
/// @param signalManager Signal manager to send messages to other systems
SeaTalk::SeaTalk( AutopilotData &autopilotdata)
: autopilotdata(autopilotdata)
{
    _mySerial.begin(4800, SWSERIAL_8S1, RX_IN, TX_OUT, true, 95);
   // _signalManager = signalManager;
}

uint8_t SeaTalk::calcChecksum(uint8_t x) {
  uint8_t temp = 0xFF - x;
  
  return temp;

}


void SeaTalk::printArrayAsHex(uint8_t* array, size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (array[i] < 0x10) {
      // Print leading zero for single-digit hex numbers
      Serial.print("0");
    }
    Serial.print(array[i], HEX);
    Serial.print(" ");
  }
  Serial.println(); // Print a newline at the end
}

/// @brief Method to run messages
void SeaTalk::processMessages()
{
    checkBus();
}

/// @brief Checks the seatalk bus for messages
/// @return if valid
int SeaTalk::checkBus()
{
    static std::vector<uint8_t> message;
    static size_t expectedLength = 0;

    while (_mySerial.available())
    {
        uint8_t inByte = _mySerial.read();
        bool parity = _mySerial.readParity();

        if (parity) // Command byte received
        {
            message.clear(); // Clear previous message
            message.push_back(inByte);
        }
        else if (!message.empty()) // Data byte received
        {
            message.push_back(inByte);

            if (message.size() == 2)
            {
                // Determine the expected length of the message
                expectedLength = 3 + (message[1] & 0x0F);
            }

            if (message.size() == expectedLength)
            {
                // Log the complete message
                //Serial.print("Received message: ");
                for (uint8_t byte : message)
                {
                    Serial.printf("%02X ", byte);
                }
                Serial.println();

                // Process messages
                switch (message[0])
                {
                case 0x10: // Apparent Wind Angle
                    if (message.size() == 4)
                    {
                        double apparentWindAngle = ((message[2] << 8) | message[3]) / 2.0;
                        //Serial.printf("Apparent Wind Angle: %.1f degrees\n", apparentWindAngle);
                    }
                    break;
                case 0x11: // Apparent Wind Speed
                    if (message.size() == 4)
                    {
                        double apparentWindSpeed = ((message[2] & 0x7f) + (message[3] & 0x0f) / 10.0);
                        Serial.printf("Apparent Wind Speed: %.1f knots\n", apparentWindSpeed);
                    }
                    break;
                case 0x20: // Speed Through Water
                    if (message.size() == 4)
                    {
                        double speedThroughWater = ((message[3] << 8) | message[2]) / 10;
                        //Serial.printf("Speed Through Water: %.1f knots\n", speedThroughWater);
                    }
                    break;
                case 0x86: // AP button pressed
                    if (message.size() == 4)
                    {
                       // Serial.print("Button pressed: ");
                        Serial.println(message[2]);
                        switch (message[2])
                        {
                        case 0x02:
                            autopilotdata.setMode(Standby);
                            Serial.println("AP mode set to standby");
                            break;
                        case 0x01:
                            autopilotdata.setMode(Auto);
                            autopilotdata.setTargetHeading(autopilotdata.getActualHeading());
                            Serial.println("AP mode set to Auto/Pilot");
                            break;
                        case 0x03:
                            // first check that navigation data is available
                            // do that here

                            if (autopilotdata.getMode()==Auto)
                                 autopilotdata.setMode(Auto_waiting_for_confirmation);
                            else if (autopilotdata.getMode()==Auto) {
                                autopilotdata.setMode(Track);
                                Serial.println("AP mode set to Track");
                                // set target heading to bearing to waypoint
                                // Set autopilotdata.setTargetHeading(autopilotdata.getActualHeading());
                            }
                            break;
                        case 0x23:
                            // first check that wind data is available
                            // do that here
                            autopilotdata.setMode(Wind);

                            // set target heading to actual heading 
                            // Set target wind angle to current wind angle
                            autopilotdata.setTargetHeading(autopilotdata.getActualHeading());
                            Serial.println("AP mode set to Wind");
                            break;
                        case 0x07:
                            autopilotdata.setTargetHeading(autopilotdata.getTargetHeading() + 1);
                            break;
                        case 0x05:
                            autopilotdata.setTargetHeading(autopilotdata.getTargetHeading() - 1);
                            break;
                        case 0x06:
                            autopilotdata.setTargetHeading(autopilotdata.getTargetHeading() - 10);
                            break;
                        case 0x08:
                            autopilotdata.setTargetHeading(autopilotdata.getTargetHeading() + 10);
                            break;
                        default:
                            break;
                        }
                    }
                    break;
                case 0x52: // Speed Over Ground
                    if (message.size() == 4)
                    {
                        double speedOverGround = ((message[3] << 8) | message[2]) / 10;
                        //Serial.printf("Speed Over Ground: %.1f knots\n", speedOverGround);
                    }
                    break;
                case 0x53: // Course Over Ground
                    if (message.size() == 4)
                    {
                        uint8_t u = (message[1] & 0xf0) >> 4;
                        uint8_t vw = message[2];
                        double courseOverGround = (u & 0x3) * 90 + (vw & 0x3F) * 2 + ((u & 0xC) >> 2) / 2;
                        //Serial.printf("Course Over Ground: %.1f degrees\n", courseOverGround);
                    }
                    break;
                case 0xAC: // Nauti-Control Echo Command 2nd byte returns 1 to signify return
                    if (message.size() == 4 && message[1] == 0x00)
                    {
                        uint8_t testNumber = message[3];
                        uint8_t responsemessage[] = {0xAC, 0x01, 0x00, testNumber};
                        send2ST(responsemessage, 4);
                    }
                    break;
                default:
                    break;
                }

                // Clear message after processing
                message.clear();
            }
        }
    }

    return -1;
}

/*
 84  U6  VW  XY 0Z 0M RR SS TT  Compass heading  Autopilot course and
                  Rudder position (see also command 9C)
                  Compass heading in degrees:
                    The two lower  bits of  U * 90 +
                    the six lower  bits of VW *  2 +
                    number of bits set in the two higher bits of U =
                    (U & 0x3)* 90 + (VW & 0x3F)* 2 + (U & 0xC ? (U & 0xC == 0xC ? 2 : 1): 0)
                  Turning direction:
                    Most significant bit of U = 1: Increasing heading, Ship turns right
                    Most significant bit of U = 0: Decreasing heading, Ship turns left
                  Autopilot course in degrees:
                    The two higher bits of  V * 90 + XY / 2
                  Z & 0x2 = 0 : Autopilot in Standby-Mode
                  Z & 0x2 = 2 : Autopilot in Auto-Mode
                  Z & 0x4 = 4 : Autopilot in Vane Mode (WindTrim), requires regular "10" datagrams
                  Z & 0x8 = 8 : Autopilot in Track Mode
                  M: Alarms + audible beeps
                    M & 0x04 = 4 : Off course
                    M & 0x08 = 8 : Wind Shift
                  Rudder position: RR degrees (positive values steer right,
                    negative values steer left. Example: 0xFE = 2° left)
                  SS & 0x01 : when set, turns off heading display on 600R control.
                  SS & 0x02 : always on with 400G
                  SS & 0x08 : displays “NO DATA” on 600R
                  SS & 0x10 : displays “LARGE XTE” on 600R
                  SS & 0x80 : Displays “Auto Rel” on 600R
                  TT : Always 0x08 on 400G computer, always 0x05 on 150(G) computer 
                    uint8_t ST_AP84[9] = {0x84,0x16,0x23,0x22,0x00,0x00,0x23,0x00,0x05};
  
*/
void SeaTalk::sendAP84() {
    uint8_t command[9];
    char c;
    uint16_t cmd;

    uint16_t U= (int)(autopilotdata.getActualHeading() / 90);
    // Serial.println(U,HEX);
    U=U << 4;

    uint16_t VW = (int)(autopilotdata.getActualHeading() - (U >> 4)*90)/2;
    uint16_t temp=((U>>4)*90+VW*2);
    uint16_t rest=autopilotdata.getActualHeading()-temp;
    // Serial.println(VW,BIN);
    if (rest==1) 
        U=U+(rest << 6); 
    U=U+6;

    uint16_t tempC=(int)(autopilotdata.getTargetHeading() / 90);

    tempC = tempC << 6;
    VW=VW+tempC;
    //  Serial.println(VW,BIN);
    uint16_t XY=(int)(autopilotdata.getTargetHeading() - (tempC >> 6)*90)*2;

    //84,F6,9,0,40,0,FC,0,6,
    boolean APlevels=false;
    boolean APresponselevels=false;
    if ( (APlevels==true) || (APresponselevels==true ))
        command[0]=0x95;
    else  
        command[0]=0x84;
    command[1]=U;
    command[2]=VW;
    command[3]=XY;
    // Serial.println(autopilotdata.getMode());
    switch (autopilotdata.getMode()) {
        case Standby:
            command[4]=0x00;
            break;
        case Auto:
            command[4]=0x02;
            break;
        case Track:
            command[4]=0x0A; //2+8
            break;  
        case Wind:
            command[4]=0x06; //2+4
            break;  
        default:
            command[4]=0x02;
            Serial.println("ERROR");
            break;
    }
    command[5]=0x00;
    command[6]=0xFA;
    command[7]=0x00;
    command[8]=0x06;
    //  printArrayAsHex(command, sizeof(command));
send2ST(command, sizeof(command));
}

/*
 83  07  XX  00  00  00  00  00  80  00  00  Sent by course computer.
                 XX = 0 after clearing a failure condition, also sent once after power-up.
                 XX = 1 failure, auto release error. Repeated once per second.
                 XX = 2 Waiting for track mode confirmation
                 XX = 8 failure, drive stopped.
*/
void SeaTalk::sendAP83(char data ) {
    uint8_t ST_AP83[11] = {0x83,0x07,0x0F,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00};
    ST_AP83[02]=data;
    send2ST(ST_AP83, sizeof(ST_AP83));
}

// ***************** Send AP device ID to Seatalk *****************
// 90  00  XX
void SeaTalk::sendAP90() {
    uint8_t command[3];
    command[0]=0x90;
    command[1]=0x00;
    command[2]=0x05;
    send2ST(command, sizeof(command));
}

// ***************** Send compass heading and rudder angle to Seatalk *****************

void SeaTalk::sendAP9C() {
    uint8_t command[4];
    uint16_t U= (int)(autopilotdata.getActualHeading() / 90);
    U=U << 4;
    uint16_t VW = (int)(autopilotdata.getActualHeading()  - (U >> 4)*90)/2;
    uint16_t temp=((U>>4)*90+VW*2);
    uint16_t rest=autopilotdata.getActualHeading() -temp;
    if (rest==1) 
    U=U+(rest << 6);
    U=U+1;

    command[0]=0x9C;
    command[1]=U;
    command[2]=VW;
    command[3]=autopilotdata.getRudderAngle();
 
    send2ST(command, sizeof(command));
   
}


/// @brief Send to seatalk Bus
/// @param cmd cmd
/// @param bytes length
/// @return did send
bool SeaTalk::send2ST(const uint8_t cmd[], int bytes)
{
    int j = 0;
    boolean ok = true;
    // Retry 5 Times
    while (j < 5)
    {
        checkClearToWrite();
        digitalWrite(LED_PIN, HIGH);
        for (int i = 0; (i < bytes) & (ok); i++)
        {
            (i == 0) ? _mySerial.write(cmd[i], SWSERIAL_PARITY_MARK) : _mySerial.write(cmd[i], SWSERIAL_PARITY_SPACE);
            delay(1);
            if (_mySerial.available())
            {
                uint8_t nextByte = _mySerial.read();
                if (nextByte != cmd[i])
                {
                    Serial.printf("Failed Byte Sent = %x byte received= %x \n", cmd[i], nextByte);
                    ok = false;
                }
            }
            else
            {
               // Serial.println("Serial Not Available");
                ok = false;
            }
        }

        if (ok)
        {
            digitalWrite(LED_PIN, LOW);
            delay(1);

//            Serial.println("Command Sent");
            return ok;
        }
        j++; // Collision detected
       // Serial.println("Collision Detected");
        delay(random(2, 5));
        ok = true;
    }

   // Serial.println("Send Failed");
    return false;
}

/// @brief Checks bus clear to write
void SeaTalk::checkClearToWrite()
{

    while (_mySerial.available())
    { // Wait for silence on the bus
        uint8_t inbyte = _mySerial.read();
        delay(3);
    }
   // Serial.println("Clear To Send");
}

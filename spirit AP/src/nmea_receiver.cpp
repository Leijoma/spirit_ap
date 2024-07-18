#include "nmea_receiver.h"
#include "logger.h"

NMEAReceiver::NMEAReceiver(const char* udpAddress, uint16_t udpPort)
    : _udpAddress(udpAddress), _udpPort(udpPort) {}

void NMEAReceiver::init() {
    _udp.begin(_udpPort);
}

void NMEAReceiver::receive() {
    int packetSize = _udp.parsePacket();
    if (packetSize) {
        char packetBuffer[255];
        int len = _udp.read(packetBuffer, 255);
        if (len > 0) {
            packetBuffer[len] = 0;
        }
        String nmeaSentence = String(packetBuffer);
        parseNMEASentence(nmeaSentence);
    }
}

void NMEAReceiver::onRudderAngleReceived(std::function<void(float)> callback) {
    _rudderAngleCallback = callback;
}

void NMEAReceiver::onRMBReceived(std::function<void(float, float, float, char)> callback) {
    _rmbCallback = callback;
}

void NMEAReceiver::onRMCReceived(std::function<void(float, float, float, float, float, char)> callback) {
    _rmcCallback = callback;
}

void NMEAReceiver::onAPBReceived(std::function<void(float, float, float, char)> callback) {
    _apbCallback = callback;
}

void NMEAReceiver::onXTEReceived(std::function<void(float, char)> callback) {
    _xteCallback = callback;
}

void NMEAReceiver::onHDGReceived(std::function<void(float)> callback) {
    _hdgCallback = callback;
}

void NMEAReceiver::parseNMEASentence(const String& sentence) {
    // Logger::log(sentence);
    if (sentence.startsWith("$ECRMB")) {
        // Parse RMB sentence
       
       
        int startIdx = sentence.indexOf(',') + 1;
        char active = sentence[startIdx];
        startIdx = sentence.indexOf(',') + 1;
        float crossTrackError = sentence.substring(startIdx).toFloat();
        startIdx = sentence.indexOf(',', startIdx) + 1;
        char dirToSteer = sentence[startIdx];
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float destLat = parseLatitude(sentence.substring(startIdx, startIdx + 9), sentence[startIdx + 10]);
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float destLon = parseLongitude(sentence.substring(startIdx, startIdx + 10), sentence[startIdx + 11]);
        if (_rmbCallback) _rmbCallback(crossTrackError, destLat, destLon, dirToSteer);

    } else if (sentence.startsWith("$ECRMC")) {
        // Parse RMC sentence
       
        int startIdx = sentence.indexOf(',') + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        char status = sentence[startIdx];
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float lat = parseLatitude(sentence.substring(startIdx, startIdx + 9), sentence[startIdx + 10]);
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float lon = parseLongitude(sentence.substring(startIdx, startIdx + 10), sentence[startIdx + 11]);
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float speedOverGround = sentence.substring(startIdx).toFloat();
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float courseOverGround = sentence.substring(startIdx).toFloat();
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float date = sentence.substring(startIdx).toFloat();
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float magneticVariation = sentence.substring(startIdx).toFloat();
        startIdx = sentence.indexOf(',', startIdx) + 1;
        char modeIndicator = sentence[startIdx];
        if (_rmcCallback) _rmcCallback(lat, lon, speedOverGround, courseOverGround, magneticVariation, modeIndicator);

    } else if (sentence.startsWith("$ECAPB")) {
       // Logger::log(sentence);
        // Parse APB sentence
        //$ECAPB,A,A,7.758,R,N,V,V,79.483,T,002,165.176,T,165.176,T*13
        /*
        1. Status V = LORAN-C Blink or SNR warning V = general warning flag or other navigation systems when a reliable fix is not available
        2. Status V = Loran-C Cycle Lock warning flag A = OK or not used
        3. Cross Track Error Magnitude
        4. Direction to steer, L or R
        5. Cross Track Units, N = Nautical Miles
        6. Status A = Arrival Circle Entered
        7. Status A = Perpendicular passed at waypoint
        8. Bearing origin to destination
        9. M = Magnetic, T = True
        10. Destination Waypoint ID
        11. Bearing, present position to Destination
        12. M = Magnetic, T = True
        13. Heading to steer to destination waypoint
        14. M = Magnetic, T = True*/
        int startIdx = sentence.indexOf(',') + 1;
        startIdx = sentence.indexOf(',') + 1;
        startIdx = sentence.indexOf(',') + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float crossTrackError = sentence.substring(startIdx).toFloat();
        startIdx = sentence.indexOf(',', startIdx) + 1;
        char dirToSteer = sentence[startIdx];
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float bearingOriginToDestination = sentence.substring(startIdx).toFloat();
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        startIdx = sentence.indexOf(',', startIdx) + 1;
        float headingToSteerToDestination = sentence.substring(startIdx).toFloat();
       
        if (_apbCallback) _apbCallback(crossTrackError, bearingOriginToDestination, headingToSteerToDestination, dirToSteer);

    } else if (sentence.startsWith("$ECXTE")) {
        // Parse XTE sentence
        int startIdx = sentence.indexOf(',') + 1;
        float crossTrackError = sentence.substring(startIdx).toFloat();
        startIdx = sentence.indexOf(',', startIdx) + 1;
        char dirToSteer = sentence[startIdx];
        if (_xteCallback) _xteCallback(crossTrackError, dirToSteer);

    } else if (sentence.startsWith("$HCHDG")) {
        // Parse HDG sentence
        int startIdx = sentence.indexOf(',') + 1;
        float heading = sentence.substring(startIdx).toFloat();
        if (_hdgCallback) _hdgCallback(heading);
    }
}

float NMEAReceiver::parseLatitude(const String& lat, char dir) {
    float degrees = lat.substring(0, 2).toFloat();
    float minutes = lat.substring(2).toFloat();
    float decimalDegrees = degrees + (minutes / 60.0);
    if (dir == 'S') decimalDegrees = -decimalDegrees;
    return decimalDegrees;
}

float NMEAReceiver::parseLongitude(const String& lon, char dir) {
    float degrees = lon.substring(0, 3).toFloat();
    float minutes = lon.substring(3).toFloat();
    float decimalDegrees = degrees + (minutes / 60.0);
    if (dir == 'W') decimalDegrees = -decimalDegrees;
    return decimalDegrees;
}

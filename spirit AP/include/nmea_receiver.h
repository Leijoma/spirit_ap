#ifndef NMEA_RECEIVER_H
#define NMEA_RECEIVER_H

#include <WiFiUdp.h>
#include <functional>

class NMEAReceiver {
public:
    NMEAReceiver(const char* udpAddress, uint16_t udpPort);

    void init();
    void receive();

    void onRudderAngleReceived(std::function<void(float)> callback);
    void onRMBReceived(std::function<void(float, float, float, char)> callback);
    void onRMCReceived(std::function<void(float, float, float, float, float, char)> callback); // Updated for current position
    void onAPBReceived(std::function<void(float, float, float, char)> callback);
    void onXTEReceived(std::function<void(float, char)> callback);
    void onHDGReceived(std::function<void(float)> callback);

private:
    const char* _udpAddress;
    uint16_t _udpPort;
    WiFiUDP _udp;

    std::function<void(float)> _rudderAngleCallback;
    std::function<void(float, float, float, char)> _rmbCallback;
    std::function<void(float, float, float, float, float, char)> _rmcCallback; // Updated for current position
    std::function<void(float, float, float, char)> _apbCallback;
    std::function<void(float, char)> _xteCallback;
    std::function<void(float)> _hdgCallback;

    void parseNMEASentence(const String& sentence);
    float parseLatitude(const String& lat, char dir);
    float parseLongitude(const String& lon, char dir);
};

#endif // NMEA_RECEIVER_H

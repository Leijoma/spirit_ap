#include "logger.h"
#include <cstdarg>

void Logger::begin(unsigned long baudRate) {
    Serial.begin(baudRate);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
}

void Logger::log(const String& message) {
    Serial.println(message);
}

void Logger::log(const char* message) {
    Serial.println(message);
}

void Logger::logf(const char* format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    Serial.println(buf);
}

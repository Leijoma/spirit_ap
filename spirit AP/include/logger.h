#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

class Logger {
public:
    static void begin(unsigned long baudRate);
    static void log(const String& message);
    static void log(const char* message);
    static void logf(const char* format, ...);
};

#endif // LOGGER_H

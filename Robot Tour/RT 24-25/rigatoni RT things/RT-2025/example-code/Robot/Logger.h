#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <ArduinoLog.h>

/*
 https://github.com/thijse/Arduino-Log

format specifiers:
 * %s	display as string (char*)
 * %S    display as string from flash memory (__FlashStringHelper* or char[] PROGMEM)
 * %c	display as single character
 * %C    display as single character or as hexadecimal value (prefixed by `0x`) if not a printable character
 * %d	display as integer value
 * %l	display as long value
 * %u	display as unsigned long value
 * %x	display as hexadecimal value
 * %X	display as hexadecimal value prefixed by `0x` and leading zeros
 * %b	display as binary number
 * %B	display as binary number, prefixed by `0b`
 * %t	display as boolean value "t" or "f"
 * %T	display as boolean value "true" or "false"
 * %D,%F display as double value
 * %p    display a  printable object
 */

class Logger
{
private:
    static void printTimestamp(Print *_logOutput, int level)
    {
        char c[12];
        sprintf(c, "%10lu ", millis());
        _logOutput->print(c);
    }

    static void printNewline(Print *_logOutput, int level)
    {
        _logOutput->print('\n');
    }

public:
    Logger(unsigned long baudRate = 115200)
    {
        Serial.begin(baudRate);
        while (!Serial)
            ;

        Log.begin(LOG_LEVEL_VERBOSE, &Serial);
        Log.setPrefix(printTimestamp);
        Log.setSuffix(printNewline);
    }

    template <typename... Args>
    void info(const char *format, Args... args)
    {
        Log.info(format, args...);
    }

    template <typename... Args>
    void warn(const char *format, Args... args)
    {
        Log.warning(format, args...);
    }

    template <typename... Args>
    void error(const char *format, Args... args)
    {
        Log.error(format, args...);
    }

    template <typename... Args>
    void verbose(const char *format, Args... args)
    {
        Log.verbose(format, args...);
    }
};

extern Logger logger;

#endif

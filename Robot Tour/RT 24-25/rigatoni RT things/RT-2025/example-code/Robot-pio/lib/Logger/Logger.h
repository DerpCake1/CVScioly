#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <ArduinoLog.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <stdarg.h>

// LCD display dimensions and pins
#define TFT_CS 15 // Chip select control pin
#define TFT_DC 2  // Data Command control pin
#define TFT_RST 4 // Reset pin

// Custom color macros
#define COLOR_WHITE ST77XX_WHITE
#define COLOR_YELLOW ST77XX_YELLOW
#define COLOR_RED ST77XX_RED
#define COLOR_GREEN ST77XX_GREEN
#define COLOR_BLUE ST7735_BLUE
#define COLOR_CYAN ST77XX_CYAN

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
    Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
    uint8_t lcdLine = 0;

    // static void printTimestamp(Print *_logOutput, int level)
    // {
    //     char c[12];
    //     sprintf(c, "%10lu ", millis());
    //     _logOutput->print(c);
    // }

    static void printPrefix(Print *_logOutput, int logLevel)
    {
        printTimestamp(_logOutput);
        printLogLevel(_logOutput, logLevel);
    }

    static void printTimestamp(Print *_logOutput)
    {

        // Division constants
        const unsigned long MSECS_PER_SEC = 1000;
        const unsigned long SECS_PER_MIN = 60;
        const unsigned long SECS_PER_HOUR = 3600;
        const unsigned long SECS_PER_DAY = 86400;

        // Total time
        const unsigned long msecs = millis();
        const unsigned long secs = msecs / MSECS_PER_SEC;

        // Time in components
        const unsigned long MilliSeconds = msecs % MSECS_PER_SEC;
        const unsigned long Seconds = secs % SECS_PER_MIN;
        const unsigned long Minutes = (secs / SECS_PER_MIN) % SECS_PER_MIN;
        const unsigned long Hours = (secs % SECS_PER_DAY) / SECS_PER_HOUR;

        // Time as string
        char timestamp[20];
        sprintf(timestamp, "%02d:%02d:%02d.%03d ", Hours, Minutes, Seconds, MilliSeconds);
        _logOutput->print(timestamp);
    }

    static void printLogLevel(Print *_logOutput, int logLevel)
    {
        /// Show log description based on log level
        switch (logLevel)
        {
        default:
        case 0:
            _logOutput->print("SILENT: ");
            break;
        case 1:
            _logOutput->print("FATAL: ");
            break;
        case 2:
            _logOutput->print("ERROR: ");
            break;
        case 3:
            _logOutput->print("WARNING: ");
            break;
        case 4:
            _logOutput->print("INFO: ");
            break;
        case 5:
            _logOutput->print("TRACE: ");
            break;
        case 6:
            _logOutput->print("VERBOSE: ");
            break;
        }
    }

    static void printNewline(Print *_logOutput, int logLevel)
    {
        _logOutput->print('\n');
    }

public:
    Logger(unsigned long baudRate = 115200)
    {
        Serial.begin(baudRate);
        while (!Serial)
            ;

        // Arduino logger
        Log.setPrefix(printPrefix);
        Log.setSuffix(printNewline);
        Log.begin(LOG_LEVEL_VERBOSE, &Serial);
        Log.setShowLevel(false);

        // LCD display
        tft.initR(INITR_MINI160x80_PLUGIN); // Init ST7735S mini display
        tft.setRotation(3);
        tft.setTextWrap(true);
        tft.setTextSize(3);
        tft.fillScreen(ST7735_BLACK);
    }

    void lcdPrint(const char *message, uint16_t color = COLOR_WHITE, uint8_t size = 3)
    {
        // Print message
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(0, 0);
        tft.setTextColor(color, color);
        tft.setTextSize(size);
        tft.print(message);
    }

    void lcdSet(uint16_t color = COLOR_WHITE, uint8_t size = 3)
    {
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(0, 0);
        tft.setTextColor(color, color);
        tft.setTextSize(size);
    }

    template <typename... Args>
    void lcdPrintf(const char *format, Args... args)
    {
        tft.printf(format, args...);
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

extern Logger logger; // Global logger instance that other classes will use

#endif

/*
 * log.h
 *
 * Timestamped printf to Serial. Replaces the old Logger class: its runtime
 * log-level switch was never changed and nothing read it back, so all that
 * remained was "print a line with a prefix".
 */

#pragma once

#include <Arduino.h>
#include <stdarg.h>

inline void logPrint(const char *level, const char *fmt, ...)
{
    char msg[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
    Serial.printf("%lu %s: %s\n", (unsigned long)millis(), level, msg);
}

#define logInfo(...)  logPrint("INFO",  __VA_ARGS__)
#define logWarn(...)  logPrint("WARN",  __VA_ARGS__)
#define logError(...) logPrint("ERROR", __VA_ARGS__)

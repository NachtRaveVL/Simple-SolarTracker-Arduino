/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Logger
*/

#ifndef HelioLogger_H
#define HelioLogger_H

class HelioLogger;
struct HelioLoggerSubData;

#include "Helioduino.h"

// Logging Level
// Log levels that can be filtered upon if desired.
enum Helio_LogLevel : signed char {
    Helio_LogLevel_All,                                     // All (info, warn, err)
    Helio_LogLevel_Warnings,                                // Warnings & errors (warn, err)
    Helio_LogLevel_Errors,                                  // Just errors (err)
    Helio_LogLevel_None = -1,                               // None / disabled
    Helio_LogLevel_Info = Helio_LogLevel_All                // Info alias
};

// Logging Events
// Logging event structure that is used in signaling.
struct HelioLogEvent {
    Helio_LogLevel level;                                   // Log level
    String timestamp;                                       // Timestamp (generated)
    String prefix;                                          // Prefix
    String msg;                                             // Message
    String suffix1;                                         // Suffix1 (optional)
    String suffix2;                                         // Suffix2 (optional)

    HelioLogEvent(Helio_LogLevel levelIn,
                  const String &prefixIn,
                  const String &msgIn,
                  const String &suffix1In = String(),
                  const String &suffix2In = String());
};

// Data Logger
// The Logger acts as the system's event monitor that collects and reports on the various
// processes of interest inside of the system. It allows for different log levels to be
// used that can help filter out unwanted noise, as well as attempts to be more optimized
// for embedded systems by spreading string data out over multiple call parameters to
// avoid large string concatenations that can overstress and crash constrained devices.
// Logging to SD card .txt log files (via SPI card reader) is supported as is logging to
// WiFiStorage .txt log files (via OS/OTA filesystem / WiFiNINA_Generic only).
class HelioLogger {
public:
    HelioLogger();
    ~HelioLogger();

    bool beginLoggingToSDCard(String logFilePrefix);
    inline bool isLoggingToSDCard() const;

#ifdef HELIO_USE_WIFI_STORAGE
    bool beginLoggingToWiFiStorage(String logFilePrefix);
    inline bool isLoggingToWiFiStorage() const;
#endif

    inline void logActivation(const HelioActuator *actuator);
    inline void logDeactivation(const HelioActuator *actuator);
    inline void logProcess(const HelioObjInterface *obj, const String &processString = String(), const String &statusString = String());
    inline void logStatus(const HelioObjInterface *obj, const String &statusString = String());

    void logSystemUptime();
    inline void logSystemSave() { logMessage(SFP(HStr_Log_SystemDataSaved)); }

    void logMessage(const String &msg, const String &suffix1 = String(), const String &suffix2 = String());
    void logWarning(const String &warn, const String &suffix1 = String(), const String &suffix2 = String());
    void logError(const String &err, const String &suffix1 = String(), const String &suffix2 = String());
    void flush();

    void setLogLevel(Helio_LogLevel logLevel);
    inline Helio_LogLevel getLogLevel() const;

    inline bool isLoggingEnabled() const;
    inline time_t getSystemUptime() const { return unixNow() - (_initTime ?: SECS_YR_2000); }

    Signal<const HelioLogEvent, HELIO_LOG_SIGNAL_SLOTS> &getLogSignal();

    void notifyDayChanged();

protected:
#if HELIO_SYS_LEAVE_FILES_OPEN
    File *_logFileSD;                                       // SD card log file instance (owned)
#ifdef HELIO_USE_WIFI_STORAGE
    WiFiStorageFile *_logFileWS;                            // WiFiStorageFile log file instance (owned)
#endif
#endif
    String _logFilename;                                    // Resolved log file name (based on day)
    time_t _initTime;                                       // Time of init, for uptime (UTC)
    time_t _lastSpaceCheck;                                 // Last time enough space was checked (UTC)

    Signal<const HelioLogEvent, HELIO_LOG_SIGNAL_SLOTS> _logSignal; // Logging signal

    friend class Helioduino;

    inline HelioLoggerSubData *loggerData() const;
    inline bool hasLoggerData() const;

    inline void updateInitTracking() { _initTime = unixNow(); }
    void log(const HelioLogEvent &event);
    void cleanupOldestLogs(bool force = false);
};

// Logger Serialization Sub Data
// A part of HSYS system data.
struct HelioLoggerSubData : public HelioSubData {
    Helio_LogLevel logLevel;                                // Log level filter (default: All)
    char logFilePrefix[HELIO_PREFIX_MAXSIZE];               // Base log file name prefix / folder (default: "logs/he")
    bool logToSDCard;                                       // If system logging to SD card is enabled (default: false)
    bool logToWiFiStorage;                                  // If system logging to WiFiStorage is enabled (default: false)

    HelioLoggerSubData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioLogger_H

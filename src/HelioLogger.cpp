/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Logger
*/

#include "Helioduino.h"

HelioLogEvent::HelioLogEvent(Helio_LogLevel levelIn, const String &prefixIn, const String &msgIn, const String &suffix1In, const String &suffix2In)
    : level(levelIn), timestamp(getCurrentTime().timestamp(DateTime::TIMESTAMP_FULL)), prefix(prefixIn), msg(msgIn), suffix1(suffix1In), suffix2(suffix2In)
{ ; }


HelioLogger::HelioLogger()
    : _logFilename(), _initDate(0), _lastSpaceCheck(0)
{ ; }

HelioLogger::~HelioLogger()
{
    flush();

    #if HELIO_SYS_LEAVE_FILES_OPEN
        if (_logFileSD) {
            _logFileSD->close();
            delete _logFileSD; _logFileSD = nullptr;
            Helioduino::_activeInstance->endSDCard();
        }
        #ifdef HELIO_USE_WIFI_STORAGE
            if (_logFileWS) {
                _logFileWS->close();
                delete _logFileWS; _logFileWS = nullptr;
            }
        #endif
    #endif
}

bool HelioLogger::beginLoggingToSDCard(String logFilePrefix)
{
    HELIO_SOFT_ASSERT(hasLoggerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasLoggerData() && !loggerData()->logToSDCard) {
        auto sd = Helioduino::_activeInstance->getSDCard();

        if (sd) {
            String logFilename = getYYMMDDFilename(logFilePrefix, SFP(HStr_txt));
            createDirectoryFor(sd, logFilename);
            #if HELIO_SYS_LEAVE_FILES_OPEN
                auto &logFile = _logFileSD ? *_logFileSD : *(_logFileSD = new File(sd->open(logFilename.c_str(), FILE_WRITE)));
            #else
                auto logFile = sd->open(logFilename.c_str(), FILE_WRITE);
            #endif

            if (logFile) {
                #if !HELIO_SYS_LEAVE_FILES_OPEN
                    logFile.close();
                    Helioduino::_activeInstance->endSDCard(sd);
                #endif

                Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
                strncpy(loggerData()->logFilePrefix, logFilePrefix.c_str(), 16);
                loggerData()->logToSDCard = true;
                _logFilename = logFilename;

                return true;
            }
        }

        #if !HELIO_SYS_LEAVE_FILES_OPEN
            Helioduino::_activeInstance->endSDCard(sd);
        #endif
    }

    return false;
}

#ifdef HELIO_USE_WIFI_STORAGE

bool HelioLogger::beginLoggingToWiFiStorage(String logFilePrefix)
{
    HELIO_SOFT_ASSERT(hasLoggerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasLoggerData() && !loggerData()->logToWiFiStorage) {
        String logFilename = getYYMMDDFilename(logFilePrefix, SFP(HStr_txt));
        #if HELIO_SYS_LEAVE_FILES_OPEN
            auto &logFile = _logFileWS ? *_logFileWS : *(_logFileWS = new WiFiStorageFile(WiFiStorage.open(logFilename.c_str())));
        #else
            auto logFile = WiFiStorage.open(logFilename.c_str());
        #endif

        if (logFile) {
            #if !HELIO_SYS_LEAVE_FILES_OPEN
                logFile.close();
            #endif

            Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
            strncpy(loggerData()->logFilePrefix, logFilePrefix.c_str(), 16);
            loggerData()->logToWiFiStorage = true;
            _logFilename = logFilename;

            return true;
        }
    }

    return false;
}

#endif

void HelioLogger::logSystemUptime()
{
    TimeSpan elapsed(getSystemUptime());
    if (elapsed.totalseconds()) {
        logMessage(SFP(HStr_Log_SystemUptime), timeSpanToString(elapsed));
    }
}

void HelioLogger::logMessage(const String &msg, const String &suffix1, const String &suffix2)
{
    if (hasLoggerData() && loggerData()->logLevel != Helio_LogLevel_None && loggerData()->logLevel <= Helio_LogLevel_All) {
        log(HelioLogEvent(Helio_LogLevel_Info, SFP(HStr_Log_Prefix_Info), msg, suffix1, suffix2));
    }
}

void HelioLogger::logWarning(const String &warn, const String &suffix1, const String &suffix2)
{
    if (hasLoggerData() && loggerData()->logLevel != Helio_LogLevel_None && loggerData()->logLevel <= Helio_LogLevel_Warnings) {
        log(HelioLogEvent(Helio_LogLevel_Warnings, SFP(HStr_Log_Prefix_Warning), warn, suffix1, suffix2));
    }
}

void HelioLogger::logError(const String &err, const String &suffix1, const String &suffix2)
{
    if (hasLoggerData() && loggerData()->logLevel != Helio_LogLevel_None && loggerData()->logLevel <= Helio_LogLevel_Errors) {
        log(HelioLogEvent(Helio_LogLevel_Errors, SFP(HStr_Log_Prefix_Error), err, suffix1, suffix2));
    }
}

void HelioLogger::log(const HelioLogEvent &event)
{
    #ifdef HELIO_ENABLE_DEBUG_OUTPUT
        if (Serial) {
            Serial.print(event.timestamp);
            Serial.print(' ');
            Serial.print(event.prefix);
            Serial.print(event.msg);
            Serial.print(event.suffix1);
            Serial.println(event.suffix2);
        }
    #endif

    if (isLoggingToSDCard()) {
        auto sd = Helioduino::_activeInstance->getSDCard(HELIO_LOFS_BEGIN);

        if (sd) {
            #if HELIO_SYS_LEAVE_FILES_OPEN
                auto &logFile = _logFileSD ? *_logFileSD : *(_logFileSD = new File(sd->open(_logFilename.c_str(), FILE_WRITE)));
            #else
                createDirectoryFor(sd, _logFilename);
                auto logFile = sd->open(_logFilename.c_str(), FILE_WRITE);
            #endif

            if (logFile) {
                logFile.print(event.timestamp);
                logFile.print(' ');
                logFile.print(event.prefix);
                logFile.print(event.msg);
                logFile.print(event.suffix1);
                logFile.println(event.suffix2);

                #if !HELIO_SYS_LEAVE_FILES_OPEN
                    logFile.flush();
                    logFile.close();
                #endif
            }

            #if !HELIO_SYS_LEAVE_FILES_OPEN
                Helioduino::_activeInstance->endSDCard(sd);
            #endif
        }
    }

#ifdef HELIO_USE_WIFI_STORAGE

    if (isLoggingToWiFiStorage()) {
        #if HELIO_SYS_LEAVE_FILES_OPEN
            auto &logFile = _logFileWS ? *_logFileWS : *(_logFileWS = new WiFiStorageFile(WiFiStorage.open(_logFilename.c_str())));
        #else
            auto logFile = WiFiStorage.open(_logFilename.c_str());
        #endif

        if (logFile) {
            auto logFileStream = HelioWiFiStorageFileStream(logFile, logFile.size());

            logFileStream.print(event.timestamp);
            logFileStream.print(' ');
            logFileStream.print(event.prefix);
            logFileStream.print(event.msg);
            logFileStream.print(event.suffix1);
            logFileStream.println(event.suffix2);

            #if !HELIO_SYS_LEAVE_FILES_OPEN
                logFileStream.flush();
                logFile.close();
            #endif
        }
    }

#endif

    #ifdef HELIO_USE_MULTITASKING
        scheduleSignalFireOnce<const HelioLogEvent>(_logSignal, event);
    #else
        _logSignal.fire(event);
    #endif
}

void HelioLogger::flush()
{
    #ifdef HELIO_ENABLE_DEBUG_OUTPUT
        if (Serial) { Serial.flush(); }
    #endif
    #if HELIO_SYS_LEAVE_FILES_OPEN
        if(_logFileSD) { _logFileSD->flush(); }
    #endif
    yield();
}

void HelioLogger::setLogLevel(Helio_LogLevel logLevel)
{
    HELIO_SOFT_ASSERT(hasLoggerData(), SFP(HStr_Err_NotYetInitialized));
    if (hasLoggerData() && loggerData()->logLevel != logLevel) {
        Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
        loggerData()->logLevel = logLevel;
    }
}

Signal<const HelioLogEvent, HELIO_LOG_STATE_SLOTS> &HelioLogger::getLogSignal()
{
    return _logSignal;
}

void HelioLogger::notifyDayChanged()
{
    if (isLoggingEnabled()) {
        _logFilename = getYYMMDDFilename(charsToString(loggerData()->logFilePrefix, 16), SFP(HStr_txt));
        cleanupOldestLogs();
    }
}

void HelioLogger::cleanupOldestLogs(bool force)
{
    // TODO: Old data cleanup.
}


HelioLoggerSubData::HelioLoggerSubData()
    : HelioSubData(0), logLevel(Helio_LogLevel_All), logFilePrefix{0}, logToSDCard(false), logToWiFiStorage(false)
{ ; }

void HelioLoggerSubData::toJSONObject(JsonObject &objectOut) const
{
    //HelioSubData::toJSONObject(objectOut); // purposeful no call to base method (ignores type)

    if (logLevel != Helio_LogLevel_All) { objectOut[SFP(HStr_Key_LogLevel)] = logLevel; }
    if (logFilePrefix[0]) { objectOut[SFP(HStr_Key_LogFilePrefix)] = charsToString(logFilePrefix, 16); }
    if (logToSDCard != false) { objectOut[SFP(HStr_Key_LogToSDCard)] = logToSDCard; }
    if (logToWiFiStorage != false) { objectOut[SFP(HStr_Key_LogToWiFiStorage)] = logToWiFiStorage; }
}

void HelioLoggerSubData::fromJSONObject(JsonObjectConst &objectIn)
{
    //HelioSubData::fromJSONObject(objectIn); // purposeful no call to base method (ignores type)

    logLevel = objectIn[SFP(HStr_Key_LogLevel)] | logLevel;
    const char *logFilePrefixStr = objectIn[SFP(HStr_Key_LogFilePrefix)];
    if (logFilePrefixStr && logFilePrefixStr[0]) { strncpy(logFilePrefix, logFilePrefixStr, 16); }
    logToSDCard = objectIn[SFP(HStr_Key_LogToSDCard)] | logToSDCard;
    logToWiFiStorage = objectIn[SFP(HStr_Key_LogToWiFiStorage)] | logToWiFiStorage;
}

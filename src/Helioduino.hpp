/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino System
*/

#include "Helioduino.h"

inline bool Twilight::isDaytime(time_t unixTime) const {
    DateTime time = isUTC ? DateTime((uint32_t)unixTime) : localTime(unixTime);
    double hour = time.hour() + (time.minute() / 60.0) + (time.second() / 3600.0);
    return sunrise <= sunset ? hour >= sunrise && hour <= sunset
                             : hour >= sunrise || hour <= sunset;
}

inline bool Twilight::isDaytime(DateTime localTime) const
{
    DateTime time = isUTC ? DateTime((uint32_t)unixTime(localTime)) : localTime;
    double hour = time.hour() + (time.minute() / 60.0) + (time.second() / 3600.0);
    return sunrise <= sunset ? hour >= sunrise && hour <= sunset
                             : hour >= sunrise || hour <= sunset;
}

inline time_t Twilight::hourToUnixTime(double hour, bool isUTC)
{
    return isUTC ? unixDayStart() + (time_t)(hour * SECS_PER_HOUR)
                 : unixTime(localDayStart() + TimeSpan(hour * SECS_PER_HOUR));
}

inline DateTime Twilight::hourToLocalTime(double hour, bool isUTC)
{
    return isUTC ? localTime(unixDayStart() + (time_t)(hour * SECS_PER_HOUR))
                 : localDayStart() + TimeSpan(hour * SECS_PER_HOUR);
}


#ifdef HELIO_USE_WIFI

inline WiFiClass *Helioduino::getWiFi(bool begin)
{
    return getWiFi(getWiFiSSID(), getWiFiPassword(), begin);
}

#endif
#ifdef HELIO_USE_ETHERNET

inline EthernetClass *Helioduino::getEthernet(bool begin)
{
    return getEthernet(getMACAddress(), begin);
}

#endif


inline HelioLoggerSubData *HelioLogger::loggerData() const
{
    return &Helioduino::_activeInstance->_systemData->logger;
}

inline bool HelioLogger::hasLoggerData() const
{
    return Helioduino::_activeInstance && Helioduino::_activeInstance->_systemData;
}

inline bool HelioLogger::isLoggingToSDCard() const
{
    return hasLoggerData() && loggerData()->logLevel != Helio_LogLevel_None && loggerData()->logToSDCard;
}

#ifdef HELIO_USE_WIFI_STORAGE

inline bool HelioLogger::isLoggingToWiFiStorage() const
{
    return hasLoggerData() && loggerData()->logLevel != Helio_LogLevel_None && loggerData()->logToWiFiStorage;
}

#endif

inline void HelioLogger::logActivation(const HelioActuator *actuator)
{
    if (actuator) { logMessage(actuator->getId().getDisplayString(), SFP(HStr_Log_HasEnabled)); }
}

inline void HelioLogger::logDeactivation(const HelioActuator *actuator)
{
    if (actuator) { logMessage(actuator->getId().getDisplayString(), SFP(HStr_Log_HasDisabled)); }
}

inline void HelioLogger::logProcess(const HelioObjInterface *obj, const String &processString, const String &statusString)
{
    if (obj) { logMessage(obj->getId().getDisplayString(), processString, statusString); }
}

inline void HelioLogger::logStatus(const HelioObjInterface *obj, const String &statusString)
{
    if (obj) { logMessage(obj->getId().getDisplayString(), statusString); }
}

inline Helio_LogLevel HelioLogger::getLogLevel() const
{
    return hasLoggerData() ? loggerData()->logLevel : Helio_LogLevel_None;
}

inline bool HelioLogger::isLoggingEnabled() const
{
    return hasLoggerData() && loggerData()->logLevel != Helio_LogLevel_None && (loggerData()->logToSDCard || loggerData()->logToWiFiStorage);
}


inline HelioPublisherSubData *HelioPublisher::publisherData() const
{
    return &Helioduino::_activeInstance->_systemData->publisher;
}

inline bool HelioPublisher::hasPublisherData() const
{
    return Helioduino::_activeInstance && Helioduino::_activeInstance->_systemData;
}

inline bool HelioPublisher::isPublishingToSDCard() const
{
    return hasPublisherData() && publisherData()->pubToSDCard;
}

#ifdef HELIO_USE_WIFI_STORAGE

inline bool HelioPublisher::isPublishingToWiFiStorage() const
{
    return hasPublisherData() && publisherData()->pubToWiFiStorage;
}

#endif
#ifdef HELIO_USE_MQTT

inline bool HelioPublisher::isPublishingToMQTTClient() const
{
    return hasPublisherData() && _mqttClient;
}

#endif

inline bool HelioPublisher::isPublishingEnabled() const
{
    return hasPublisherData() && (publisherData()->pubToSDCard || publisherData()->pubToWiFiStorage
        #ifdef HELIO_USE_MQTT
            || _mqttClient
        #endif
        );
}

inline void HelioPublisher::setNeedsTabulation()
{
    _needsTabulation = hasPublisherData();
}


inline HelioSchedulerSubData *HelioScheduler::schedulerData() const
{
    return &Helioduino::_activeInstance->_systemData->scheduler;
}

inline bool HelioScheduler::hasSchedulerData() const
{
    return Helioduino::_activeInstance && Helioduino::_activeInstance->_systemData;
}

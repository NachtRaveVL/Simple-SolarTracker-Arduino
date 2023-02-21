/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Scheduler
*/

#include "Helioduino.h"

HelioScheduler::HelioScheduler()
    : _needsScheduling(false), _inDaytimeMode(false), _lastDay{0}
{ ; }

HelioScheduler::~HelioScheduler()
{
    while (_trackings.size()) {
        auto trackingIter = _trackings.begin();
        delete trackingIter->second;
        _trackings.erase(trackingIter);
    }
}

void HelioScheduler::update()
{
    if (hasSchedulerData()) {
        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("Scheduler::update")); flushYield();
        #endif

        {   time_t time = unixNow();
            DateTime currTime = localTime(time);
            bool daytimeMode = _dailyTwilight.isDaytime(time);

            if (_inDaytimeMode != daytimeMode) {
                _inDaytimeMode = daytimeMode;
                setNeedsScheduling();
                Helioduino::_activeInstance->setNeedsLayout();
            }

            if (!(_lastDay[0] == currTime.year() &&
                  _lastDay[1] == currTime.month() &&
                  _lastDay[2] == currTime.day())) {
                // only log uptime upon actual day change and if uptime has been at least 1d
                if (getLogger()->getSystemUptime() >= SECS_PER_DAY) {
                    getLogger()->logSystemUptime();
                }
                broadcastDayChange();
            }
        }

        if (needsScheduling()) { performScheduling(); }

        for (auto trackingIter = _trackings.begin(); trackingIter != _trackings.end(); ++trackingIter) {
            trackingIter->second->update();
        }

        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("Scheduler::~update")); flushYield();
        #endif
    }
}

void HelioScheduler::setupPanelDriver(HelioPanel *panel, hposi_t axisIndex, SharedPtr<HelioDriver> panelDriver)
{
    if (panel && panelDriver) {
        if ((panel->drivesHorizontalAxis() && axisIndex == 0) ||
            (panel->drivesHorizontalAxis() && axisIndex == 1)) {
            Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> actuators;

            auto axisActuators = linksFilterTravelActuatorsByPanelAxisAndType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, axisIndex, panelDriver->isIncrementalType());
            linksResolveActuatorsToAttachments<HELIO_DRV_ACTUATORS_MAXSIZE>(axisActuators, panelDriver.get(), axisIndex, actuators);

            panelDriver->setActuators(actuators);
        }
    }
}

void HelioScheduler::setAirReportInterval(TimeSpan interval)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasSchedulerData() && schedulerData()->airReportInterval != interval.totalseconds()) {
        Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
        schedulerData()->airReportInterval = interval.totalseconds();
    }
}

TimeSpan HelioScheduler::getAirReportInterval() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return TimeSpan(hasSchedulerData() ? schedulerData()->airReportInterval : 0);
}

void HelioScheduler::updateDayTracking()
{
    time_t time = unixNow();
    DateTime currTime = localTime(time);
    _lastDay[0] = currTime.year();
    _lastDay[1] = currTime.month();
    _lastDay[2] = currTime.day();

    Location loc = getController()->getSystemLocation();
    if (loc.hasPosition()) {
        double transit; // high noon, hours +fractional
        calcSunriseSunset((unsigned long)time, loc.latitude, loc.longitude, transit, _dailyTwilight.sunrise, _dailyTwilight.sunset,
                          loc.resolveSunAlt(), HELIO_SYS_SUNRISESET_CALCITERS);
        _dailyTwilight.isUTC = true;
    } else if (_dailyTwilight.isUTC) {
        _dailyTwilight = Twilight();
    }
    _inDaytimeMode = _dailyTwilight.isDaytime(time);

    setNeedsScheduling();
    Helioduino::_activeInstance->setNeedsLayout();
}

void HelioScheduler::performScheduling()
{
    HELIO_HARD_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    for (auto iter = Helioduino::_activeInstance->_objects.begin(); iter != Helioduino::_activeInstance->_objects.end(); ++iter) {
        if (iter->second->isPanelType()) {
            auto panel = static_pointer_cast<HelioPanel>(iter->second);

            {   auto trackingIter = _trackings.find(panel->getKey());

                if (linksCountTravelActuators(panel->getLinkages())) {
                    if (trackingIter != _trackings.end()) {
                        if (trackingIter->second) {
                            trackingIter->second->setupStaging();
                        }
                    } else {
                        #ifdef HELIO_USE_VERBOSE_OUTPUT
                            Serial.print(F("Scheduler::performScheduling Travel actuator linkages found for: ")); Serial.print(iter->second->getKeyString());
                            Serial.print(':'); Serial.print(' '); Serial.println(linksCountTravelActuators(feedReservoir->getLinkages())); flushYield();
                        #endif

                        HelioTracking *tracking = new HelioTracking(panel);
                        HELIO_SOFT_ASSERT(tracking, SFP(HStr_Err_AllocationFailure));
                        if (tracking) { _trackings[panel->getKey()] = tracking; }
                    }
                } else if (trackingIter != _trackings.end()) { // No travel actuators to warrant process -> delete if exists
                    #ifdef HYDRO_USE_VERBOSE_OUTPUT
                        Serial.print(F("Scheduler::performScheduling NO travel actuator linkages found for: ")); Serial.println(iter->second->getKeyString()); flushYield();
                    #endif
                    if (trackingIter->second) { delete trackingIter->second; }
                    _trackings.erase(trackingIter);
                }
            }
        }
    }

    _needsScheduling = false;
}

void HelioScheduler::broadcastDayChange()
{
    updateDayTracking();

    #ifdef HELIO_USE_MULTITASKING
        // these can take a while to complete
        taskManager.scheduleOnce(0, []{
            if (getController()) {
                getController()->notifyDayChanged();
            }
            yield();
            if (getLogger()) {
                getLogger()->notifyDayChanged();
            }
            yield();
            if (getPublisher()) {
                getPublisher()->notifyDayChanged();
            }
            yield();
        });
    #else
        if (getController()) {
            getController()->notifyDayChanged();
        }
        if (getLogger()) {
            getLogger()->notifyDayChanged();
        }
        if (getPublisher()) {
            getPublisher()->notifyDayChanged();
        }
    #endif
}


HelioProcess::HelioProcess(SharedPtr<HelioPanel> panelIn)
    : panel(panelIn), stageStart(0)
{ ; }

void HelioProcess::clearActuatorReqs()
{
    while (actuatorReqs.size()) {
        actuatorReqs.erase(actuatorReqs.begin());
    }
}

void HelioProcess::setActuatorReqs(const Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> &actuatorReqsIn)
{
    for (auto attachIter = actuatorReqs.begin(); attachIter != actuatorReqs.end(); ++attachIter) {
        bool found = false;
        auto key = attachIter->getKey();
    
        for (auto attachInIter = actuatorReqsIn.begin(); attachInIter != actuatorReqsIn.end(); ++attachInIter) {
            if (key == attachInIter->getKey()) {
                found = true;
                break;
            }
        }
    
        if (!found) { // disables actuators not found in new list
            attachIter->disableActivation();
        }
    }

    {   actuatorReqs.clear();
        for (auto attachInIter = actuatorReqsIn.begin(); attachInIter != actuatorReqsIn.end(); ++attachInIter) {
            actuatorReqs.push_back(*attachInIter);
            actuatorReqs.back().setParent(nullptr);
        }
    }
}


HelioTracking::HelioTracking(SharedPtr<HelioPanel> panel)
    : HelioProcess(panel), stage(Unknown), canProcessAfter(0), lastAirReport(0)
{
    reset();
}

HelioTracking::~HelioTracking()
{
    clearActuatorReqs();
}

void HelioTracking::reset()
{
    clearActuatorReqs();
    stage = Init; stageStart = unixNow();
    setupStaging();
}

void HelioTracking::setupStaging()
{
    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFS1 = (int8_t)-1; if (_stageFS1 != (int8_t)stage) {
        Serial.print(F("Tracking::setupStaging stage: ")); Serial.println((_stageFS1 = (int8_t)stage)); flushYield(); } }
    #endif

    // if (panel->getTemperatureSensor()) {
    //     auto airTempBalancer = panel->getTemperatureBalancer();
    //     if (!airTempBalancer) {
    //         airTempBalancer = SharedPtr<HelioAbsoluteDriver>(new HelioAbsoluteDriver(panel->getTemperatureSensor(), airTempSetpoint, HELIO_RANGE_TEMP_HALF, -HELIO_RANGE_TEMP_HALF * 0.25f, HELIO_RANGE_TEMP_HALF * 0.5f));
    //         HELIO_SOFT_ASSERT(airTempBalancer, SFP(HStr_Err_AllocationFailure));
    //         getScheduler()->setupTemperatureBalancer(panel.get(), airTempBalancer);
    //         panel->setTemperatureBalancer(airTempBalancer);
    //     }
    //     if (airTempBalancer) {
    //         airTempBalancer->setTargetSetpoint(airTempSetpoint);
    //         airTempBalancer->setMeasurementUnits(Helio_UnitsType_Temperature_Celsius);
    //         airTempBalancer->setEnabled(true);
    //     }
    // } else {
    //     auto airTempBalancer = panel->getTemperatureBalancer();
    //     if (airTempBalancer) { airTempBalancer->setEnabled(false); }
    // }

    switch (stage) {
        case Init: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newActuatorReqs;
            auto panelBrakes = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            setActuatorReqs(newActuatorReqs);

            auto sunrise = getScheduler()->getDailyTwilight().getSunriseLocalTime();
            bool needsCleaning = false; // todo
            if (needsCleaning) {
                auto panelSprayers = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelSprayer);
                needsCleaning = panelSprayers.size();
            }
            auto panelHeaters = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelHeater);
            canProcessAfter = unixTime(sunrise - (needsCleaning ? TimeSpan(0,0,30,0) : TimeSpan(0))
                                               - (panelHeaters.size() ? TimeSpan(0,0,30,0) : TimeSpan(0)));
        } break;

        case Preheat: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newActuatorReqs;
            auto panelHeaters = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelHeater);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelHeaters, nullptr, 0, newActuatorReqs);
            auto panelBrakes = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            setActuatorReqs(newActuatorReqs);

            auto sunrise = getScheduler()->getDailyTwilight().getSunriseLocalTime();
            bool needsCleaning = false; // todo
            if (needsCleaning) {
                auto panelSprayers = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelSprayer);
                needsCleaning = panelSprayers.size();
            }
            canProcessAfter = unixTime(sunrise - (needsCleaning ? TimeSpan(0,0,30,0) : TimeSpan(0)));
        } break;

        case Uncover: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newDrvActuators;
            auto panelCovers = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelCover);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelCovers, nullptr, 0, newDrvActuators);
            auto coverDriver = panel->getCoverDriver();
            coverDriver->setActuators(newDrvActuators);
            coverDriver->setTargetSetpoint(coverDriver->getTrackRange().first);

            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newActuatorReqs;
            auto panelHeaters = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelHeater);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelHeaters, nullptr, 0, newActuatorReqs);
            auto panelBrakes = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            setActuatorReqs(newActuatorReqs);

            canProcessAfter = stageStart + 15;
        } break;

        case Clean: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newActuatorReqs;
            auto panelHeaters = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelHeater);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelHeaters, nullptr, 0, newActuatorReqs);
            auto panelSprayers = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelSprayer);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelSprayers, nullptr, 0, newActuatorReqs);
            auto panelBrakes = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            setActuatorReqs(newActuatorReqs);

            canProcessAfter = stageStart + SECS_PER_MIN;
        } break;

        case Track: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newActuatorReqs;
            auto panelHeaters = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelHeater);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelHeaters, nullptr, 0, newActuatorReqs);
            auto panelBrakes = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            setActuatorReqs(newActuatorReqs);

            canProcessAfter = stageStart + 15;
        } break;

        case Cover: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newDrvActuators;
            auto panelCovers = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelCover);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelCovers, nullptr, 0, newDrvActuators);
            auto coverDriver = panel->getCoverDriver();
            coverDriver->setActuators(newDrvActuators);
            coverDriver->setTargetSetpoint(coverDriver->getTrackRange().second);

            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newActuatorReqs;
            auto panelBrakes = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
            linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            setActuatorReqs(newActuatorReqs);

            canProcessAfter = stageStart + 15;
        } break;

        default:
            break;
    }

    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFS2 = (int8_t)-1; if (_stageFS2 != (int8_t)stage) {
        Serial.print(F("Tracking::~setupStaging stage: ")); Serial.println((_stageFS2 = (int8_t)stage)); flushYield(); } }
    #endif
}

void HelioTracking::update()
{
    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFU1 = (int8_t)-1; if (_stageFU1 != (int8_t)stage) {
        Serial.print(F("Tracking::update stage: ")); Serial.println((_stageFU1 = (int8_t)stage)); flushYield(); } }
    #endif

    // if ((!lastAirReport || unixNow() >= lastAirReport + getScheduler()->getAirReportInterval().totalseconds()) &&
    //     (getScheduler()->getAirReportInterval().totalseconds() > 0) && // 0 disables
    //     panel->getTemperatureSensor()) {
    //     getLogger()->logProcess(panel.get(), SFP(HStr_Log_AirReport));
    //     logTracking(HelioTrackingLogType_Setpoints);
    //     logTracking(HelioTrackingLogType_Measures);
    //     lastAirReport = unixNow();
    // }

    switch (stage) {
        case Init: {
            if (!canProcessAfter || unixNow() >= canProcessAfter) {
                auto currTime = localNow();
                if (currTime > getScheduler()->getDailyTwilight().getSunsetLocalTime()) {
                    stage = Cover; stageStart = unixNow(); canProcessAfter = 0;
                    setupStaging();
                } else if (currTime >= getScheduler()->getDailyTwilight().getSunriseLocalTime()) {
                    stage = Uncover; stageStart = unixNow(); canProcessAfter = 0;
                    setupStaging();
                } else if (canProcessAfter) {
                    stage = Preheat; stageStart = unixNow(); canProcessAfter = 0;
                    setupStaging();
                }
            }
        } break;

        case Preheat: {
            if (unixNow() >= canProcessAfter) {
                stage = Uncover; stageStart = unixNow(); canProcessAfter = 0;
                setupStaging();
            }
        } break;

        case Uncover: {
             if (unixNow() >= canProcessAfter) {
                if (panel->getCoverDriver()->isAligned()) {
                    bool needsCleaning = false; // todo
                    auto panelSprayers = linksFilterActuatorsByPanelAndType(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelSprayer);
                    if (needsCleaning && panelSprayers.size()) {
                        stage = Clean; stageStart = unixNow(); canProcessAfter = 0;
                        setupStaging();
                    } else {
                        stage = Track; stageStart = unixNow(); canProcessAfter = 0;
                        setupStaging();
                    }
                } else {
                    canProcessAfter = canProcessAfter + 15;
                }
             }
        } break;

        case Clean: {

        } break;

        case Track: {
            
        } break;

        case Cover: {
            
        } break;

        default:
            break;
    }

    if (actuatorReqs.size()) {
        for (auto attachIter = actuatorReqs.begin(); attachIter != actuatorReqs.end(); ++attachIter) {
            attachIter->enableActivation();
        }
    }

    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFU2 = (int8_t)-1; if (_stageFU2 != (int8_t)stage) {
        Serial.print(F("Tracking::~update stage: ")); Serial.println((_stageFU2 = (int8_t)stage)); flushYield(); } }
    #endif
}

void HelioTracking::logTracking(HelioTrackingLogType logType)
{
    switch (logType) {
        case HelioTrackingLogType_WaterSetpoints:
            {   auto ph = HelioSingleMeasurement(phSetpoint, Helio_UnitsType_Alkalinity_pH_14);
                getLogger()->logMessage(SFP(HStr_Log_Field_pH_Setpoint), measurementToString(ph));
            }
            {   auto tds = HelioSingleMeasurement(tdsSetpoint, Helio_UnitsType_Concentration_TDS);
                convertUnits(&tds, panel->getTDSUnits());
                getLogger()->logMessage(SFP(HStr_Log_Field_TDS_Setpoint), measurementToString(tds, 1));
            }
            {   auto temp = HelioSingleMeasurement(waterTempSetpoint, Helio_UnitsType_Temperature_Celsius);
                convertUnits(&temp, panel->getTemperatureUnits());
                getLogger()->logMessage(SFP(HStr_Log_Field_Temp_Setpoint), measurementToString(temp));
            }
            break;

        case HelioTrackingLogType_WaterMeasures:
            #ifdef HELIO_USE_MULTITASKING
                // Yield will allow measurements to complete, ensures first log out doesn't contain zero'ed values
                if ((panel->getWaterPHSensor() && !panel->getWaterPH().getMeasurementFrame()) ||
                    (panel->getWaterTDSSensor() && !panel->getWaterTDS().getMeasurementFrame()) ||
                    (panel->getWaterTemperatureSensor() && !panel->getWaterTemperature().getMeasurementFrame())) {
                    yield();
                }
            #endif
             if (panel->getWaterPHSensor()) {
                auto ph = panel->getWaterPH().getMeasurement(true);
                getLogger()->logMessage(SFP(HStr_Log_Field_pH_Measured), measurementToString(ph));
            }
            if (panel->getWaterTDSSensor()) {
                auto tds = panel->getWaterTDS().getMeasurement(true);
                convertUnits(&tds, panel->getTDSUnits());
                getLogger()->logMessage(SFP(HStr_Log_Field_TDS_Measured), measurementToString(tds, 1));
            }
            if (panel->getWaterTemperatureSensor()) {
                auto temp = panel->getWaterTemperature().getMeasurement(true);
                convertUnits(&temp, panel->getTemperatureUnits());
                getLogger()->logMessage(SFP(HStr_Log_Field_Temp_Measured), measurementToString(temp));
            }
            break;

        case HelioTrackingLogType_Setpoints:
            {   auto temp = HelioSingleMeasurement(airTempSetpoint, Helio_UnitsType_Temperature_Celsius);
                convertUnits(&temp, panel->getTemperatureUnits());
                getLogger()->logMessage(SFP(HStr_Log_Field_Temp_Setpoint), measurementToString(temp));
            }
            {   auto co2 = HelioSingleMeasurement(co2Setpoint, Helio_UnitsType_Concentration_PPM);
                getLogger()->logMessage(SFP(HStr_Log_Field_CO2_Setpoint), measurementToString(co2));
            }
            break;

        case HelioTrackingLogType_Measures:
            #ifdef HELIO_USE_MULTITASKING
                // Yield will allow measurements to complete, ensures first log out doesn't contain zero'ed values
                if ((panel->getTemperatureSensor() && !panel->getTemperatureSensorAttachment().getMeasurementFrame()) ||
                    (panel->getAirCO2Sensor() && !panel->getAirCO2().getMeasurementFrame())) {
                    yield();
                }
            #endif
            if (panel->getTemperatureSensor()) {
                auto temp = panel->getTemperatureSensorAttachment().getMeasurement(true);
                convertUnits(&temp, panel->getTemperatureUnits());
                getLogger()->logMessage(SFP(HStr_Log_Field_Temp_Measured), measurementToString(temp));
            }
            if (panel->getAirCO2Sensor()) {
                auto co2 = panel->getAirCO2().getMeasurement(true);
                getLogger()->logMessage(SFP(HStr_Log_Field_CO2_Measured), measurementToString(co2));
            }
            break;
    }
}

void HelioTracking::broadcastTracking(HelioTrackingBroadcastType broadcastType)
{
    getLogger()->logProcess(panel.get(), SFP(HStr_Log_TrackingSequence),
                                    SFP(broadcastType == HelioTrackingBroadcastType_Began ? HStr_Log_HasBegan : HStr_Log_HasEnded));
    logTracking(HelioTrackingLogType_WaterMeasures);

    broadcastType == HelioTrackingBroadcastType_Began ? panel->notifyTrackingBegan() : panel->notifyTrackingEnded();

    {   auto panels = linksFilterPanels(panel->getLinkages());
        for (auto panelIter = panels.begin(); panelIter != panels.end(); ++panelIter) {
            broadcastType == HelioTrackingBroadcastType_Began ? ((HelioPanel *)(*panelIter))->notifyTrackingBegan()
                                                              : ((HelioPanel *)(*panelIter))->notifyTrackingEnded();
        }
    }
}


HelioSchedulerSubData::HelioSchedulerSubData()
    : HelioSubData(0), airReportInterval(8 * SECS_PER_HOUR)
{ ; }

void HelioSchedulerSubData::toJSONObject(JsonObject &objectOut) const
{
    //HelioSubData::toJSONObject(objectOut); // purposeful no call to base method (ignores type)

    if (airReportInterval != (8 * SECS_PER_HOUR)) { objectOut[SFP(HStr_Key_AirReportInterval)] = airReportInterval; }
}

void HelioSchedulerSubData::fromJSONObject(JsonObjectConst &objectIn)
{
    //HelioSubData::fromJSONObject(objectIn); // purposeful no call to base method (ignores type)

    airReportInterval = objectIn[SFP(HStr_Key_AirReportInterval)] | airReportInterval;
}

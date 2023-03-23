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
                Helioduino::_activeInstance->setNeedsRedraw();
            }

            if (!(_lastDay[0] == currTime.year()-2000 &&
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

void HelioScheduler::setCleaningIntervalDays(unsigned int cleaningIntDays)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasSchedulerData() && schedulerData()->cleaningIntervalDays != cleaningIntDays) {
        schedulerData()->cleaningIntervalDays = cleaningIntDays;

        setNeedsScheduling();
        Helioduino::_activeInstance->_systemData->bumpRevisionIfNeeded();
    }
}

void HelioScheduler::setPreDawnCleaningMins(unsigned int cleaningMins)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasSchedulerData() && schedulerData()->preDawnCleaningMins != cleaningMins) {
        schedulerData()->preDawnCleaningMins = cleaningMins;

        setNeedsScheduling();
        Helioduino::_activeInstance->_systemData->bumpRevisionIfNeeded();
    }
}

void HelioScheduler::setPreDawnHeatingMins(unsigned int heatingMins)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasSchedulerData() && schedulerData()->preDawnHeatingMins != heatingMins) {
        schedulerData()->preDawnHeatingMins = heatingMins;

        setNeedsScheduling();
        Helioduino::_activeInstance->_systemData->bumpRevisionIfNeeded();
    }
}

void HelioScheduler::setReportInterval(TimeSpan interval)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasSchedulerData() && schedulerData()->reportInterval != interval.totalseconds()) {
        schedulerData()->reportInterval = interval.totalseconds();
        Helioduino::_activeInstance->_systemData->bumpRevisionIfNeeded();
    }
}

unsigned int HelioScheduler::getCleaningIntervalDays() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return hasSchedulerData() ? schedulerData()->cleaningIntervalDays : 0;
}

unsigned int HelioScheduler::getPreDawnCleaningMins() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return hasSchedulerData() ? schedulerData()->preDawnCleaningMins : 0;
}

unsigned int HelioScheduler::getPreDawnHeatingMins() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return hasSchedulerData() ? schedulerData()->preDawnHeatingMins : 0;
}

TimeSpan HelioScheduler::getReportInterval() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return TimeSpan(hasSchedulerData() ? schedulerData()->reportInterval : 0);
}

void HelioScheduler::updateDayTracking()
{
    time_t time = unixNow();
    DateTime currTime = localTime(time);
    _lastDay[0] = currTime.year()-2000;
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
    Helioduino::_activeInstance->setNeedsRedraw();
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
                            Serial.print(F("Scheduler::performScheduling Travel actuator linkages found for: ")); Serial.print(iter->second->getId().getDisplayString());
                            Serial.print(':'); Serial.print(' '); Serial.println(linksCountTravelActuators(panel->getLinkages())); flushYield();
                        #endif

                        HelioTracking *tracking = new HelioTracking(panel);
                        HELIO_SOFT_ASSERT(tracking, SFP(HStr_Err_AllocationFailure));
                        if (tracking) { _trackings[panel->getKey()] = tracking; }
                    }
                } else if (trackingIter != _trackings.end()) { // No travel actuators to warrant process -> delete if exists
                    #ifdef HELIO_USE_VERBOSE_OUTPUT
                        Serial.print(F("Scheduler::performScheduling NO travel actuator linkages found for: ")); Serial.println(iter->second->getId().getDisplayString()); flushYield();
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
    : panel(panelIn), stageStart(unixNow())
{ ; }

void HelioProcess::clearActuatorReqs()
{
    while (actuatorReqs.size()) {
        actuatorReqs.begin()->disableActivation();
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
    : HelioProcess(panel), stage(Init), canProcessAfter(0), lastEnvReport(0),
      stormingReported(false), nightSeqReported(false), coverSeqReported(false)
{
    setupStaging();
}

HelioTracking::~HelioTracking()
{
    clearActuatorReqs();
}

void HelioTracking::setupStaging()
{
    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFS1 = (int8_t)-1; if (_stageFS1 != (int8_t)stage) {
        Serial.print(F("Tracking::setupStaging stage: ")); Serial.println((_stageFS1 = (int8_t)stage)); flushYield(); } }
    #endif

    auto trackingPanel = panel->isAnyTrackingClass() ? static_pointer_cast<HelioTrackingPanel>(panel) : nullptr;
    bool isStorming = trackingPanel && trackingPanel->getStormingTriggerAttachment().isTriggered();
    bool canUncover = stage >= Uncover && stage < Cover && !isStorming;

    if (panel->drivesHorizontalAxis()) {
        auto axisHorz = linksFilterTravelActuatorsByPanelAxisAndMotor<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), 0, true);
        bool anyMotors = axisHorz.size();
        if (!anyMotors) { axisHorz = linksFilterTravelActuatorsByPanelAxisAndMotor<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), 0, false); }
        if (axisHorz.size()) {
            auto horzDriver = panel->getAxisDriver(0);
            if (!horzDriver || horzDriver->isIncrementalType() != anyMotors) {
                if (anyMotors) { horzDriver = SharedPtr<HelioIncrementalDriver>(new HelioIncrementalDriver()); }
                else { horzDriver = SharedPtr<HelioAbsoluteDriver>(new HelioAbsoluteDriver()); }
                HELIO_SOFT_ASSERT(horzDriver, SFP(HStr_Err_AllocationFailure));
                horzDriver->setEnabled(true);
                panel->setAxisDriver(horzDriver, 0);
            }
            Vector<HelioActuatorAttachment,HELIO_DRV_ACTUATORS_MAXSIZE> horzActivations;
            linksResolveActuatorsToAttachments<HELIO_DRV_ACTUATORS_MAXSIZE>(axisHorz, panel.get(), 0, horzActivations);
            horzDriver->setActuators(horzActivations);
        } else {
            auto horzDriver = panel->getAxisDriver(0);
            if (horzDriver) { horzDriver->setEnabled(false); }
        }
    }

    if (panel->drivesVerticalAxis()) {
        auto axisVert = linksFilterTravelActuatorsByPanelAxisAndMotor<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), 1, true);
        bool anyMotors = axisVert.size();
        if (!anyMotors) { axisVert = linksFilterTravelActuatorsByPanelAxisAndMotor<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), 1, false); }
        if (axisVert.size()) {
            auto vertDriver = panel->getAxisDriver(1);
            if (!vertDriver || vertDriver->isIncrementalType() != anyMotors) {
                if (anyMotors) { vertDriver = SharedPtr<HelioIncrementalDriver>(new HelioIncrementalDriver()); }
                else { vertDriver = SharedPtr<HelioAbsoluteDriver>(new HelioAbsoluteDriver()); }
                HELIO_SOFT_ASSERT(vertDriver, SFP(HStr_Err_AllocationFailure));
                vertDriver->setEnabled(true);
                panel->setAxisDriver(vertDriver, 1);
            }
            Vector<HelioActuatorAttachment,HELIO_DRV_ACTUATORS_MAXSIZE> vertActivations;
            linksResolveActuatorsToAttachments<HELIO_DRV_ACTUATORS_MAXSIZE>(axisVert, panel.get(), 1, vertActivations);
            vertDriver->setActuators(vertActivations);
        } else {
            auto vertDriver = panel->getAxisDriver(1);
            if (vertDriver) { vertDriver->setEnabled(false); }
        }
    }

    {   auto panelCovers = linksFilterActuatorsByPanelAndType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelCover);
        if (panelCovers.size()) {
            bool hasMotor = false;
            for (auto obj : panelCovers) { if (((HelioActuator *)obj)->isMotorType()) { hasMotor = true; break; } }
            auto coverDriver = panel->getPanelCoverDriver();
            if (!coverDriver || coverDriver->isIncrementalType() != hasMotor) {
                if (hasMotor) { coverDriver = SharedPtr<HelioIncrementalDriver>(new HelioIncrementalDriver()); }
                else { coverDriver = SharedPtr<HelioAbsoluteDriver>(new HelioAbsoluteDriver()); }
                HELIO_SOFT_ASSERT(coverDriver, SFP(HStr_Err_AllocationFailure));
                coverDriver->setEnabled(true);
                panel->setPanelCoverDriver(coverDriver);
            }
            Vector<HelioActuatorAttachment,HELIO_DRV_ACTUATORS_MAXSIZE> coverActivations;
            linksResolveActuatorsToAttachments<HELIO_DRV_ACTUATORS_MAXSIZE>(panelCovers, panel.get(), 0, coverActivations);
            coverDriver->setActuators(coverActivations);
            coverDriver->setTargetSetpoint(canUncover ? coverDriver->getTrackRange().first : coverDriver->getTrackRange().second);
        } else {
            auto coverDriver = panel->getPanelCoverDriver();
            if (coverDriver) { coverDriver->setEnabled(false); }
        }
    }

    Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> newActuatorReqs;

    switch (stage) {
        case Init:
        case Cover: {
            {   auto panelBrakes = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
                linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            }
        } break;

        case Warm:
        case Uncover:
        case Track: {
            {   auto panelBrakes = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
                linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            }
            {   auto panelHeaters = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelHeater);
                linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelHeaters, nullptr, 0, newActuatorReqs);
            }
        } break;

        case Clean: {
            {   auto panelBrakes = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelBrake);
                linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelBrakes, nullptr, 0, newActuatorReqs);
            }
            {   auto panelHeaters = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelHeater);
                linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelHeaters, nullptr, 0, newActuatorReqs);
            }
            {   auto panelSprayers = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_PanelSprayer);
                linksResolveActuatorsToAttachments<HELIO_SCH_REQACTS_MAXSIZE>(panelSprayers, nullptr, 0, newActuatorReqs);
            }
        } break;

        default:
            break;
    }

    setActuatorReqs(newActuatorReqs);
    panel->setInDaytimeMode(stage == Track && canUncover);
    canProcessAfter = unixNow();

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

    time_t time = unixNow();
    auto trackingPanel = panel->isAnyTrackingClass() ? static_pointer_cast<HelioTrackingPanel>(panel) : nullptr;

    if (trackingPanel && (!lastEnvReport || time >= lastEnvReport + getScheduler()->schedulerData()->reportInterval) &&
        (getScheduler()->schedulerData()->reportInterval > 0) && // 0 disables
        (panel->isAnyTrackingClass() && static_pointer_cast<HelioTrackingPanel>(panel)->getTemperatureSensor()) ||
        (panel->isAnyTrackingClass() && static_pointer_cast<HelioTrackingPanel>(panel)->getWindSpeedSensor())) {
        getLogger()->logProcess(panel.get(), SFP(HStr_Log_EnvReport));
        if (trackingPanel->getTemperatureSensor(true)) {
            #ifdef HELIO_USE_MULTITASKING
                trackingPanel->getTemperatureSensor()->yieldForMeasurement();
            #endif
            auto temp = trackingPanel->getTemperatureSensorAttachment().getMeasurement();
            convertUnits(&temp, trackingPanel->getTemperatureUnits());
            getLogger()->logMessage(SFP(HStr_Log_Field_Temp_Measured), measurementToString(temp));
        }
        if (trackingPanel->getWindSpeedSensor(true)) {
            #ifdef HELIO_USE_MULTITASKING
                trackingPanel->getWindSpeedSensor()->yieldForMeasurement();
            #endif
            auto windSpeed = trackingPanel->getWindSpeedSensorAttachment().getMeasurement();
            getLogger()->logMessage(SFP(HStr_Log_Field_WindSpeed_Measured), measurementToString(windSpeed));
        }
        lastEnvReport = time;
    }

    if (!canProcessAfter || time >= canProcessAfter) {
        auto stageWas = stage != Init ? stage : Cover;
        auto currTime = localTime(time);
        bool logStage = false;
        auto sunrise = getScheduler()->getDailyTwilight().getSunriseLocalTime();
        auto sunset = getScheduler()->getDailyTwilight().getSunsetLocalTime();
        bool afterSunset = currTime > sunset;
        bool afterSunrise = currTime >= sunrise;
        bool cleaningDue = trackingPanel && currTime >= sunrise - TimeSpan(0,0,getScheduler()->schedulerData()->preDawnCleaningMins,0) &&
                           currTime >= trackingPanel->getLastPanelCleaningTime() + TimeSpan(getScheduler()->schedulerData()->cleaningIntervalDays,0,0,0) &&
                           linksFilterActuatorsByPanelAndType(trackingPanel->getLinkages(), trackingPanel.get(), Helio_ActuatorType_PanelSprayer).size();
        bool preHeatingDue = trackingPanel && currTime >= sunrise - TimeSpan(0,0,getScheduler()->schedulerData()->preDawnHeatingMins,0) &&
                             trackingPanel->getHeatingTriggerAttachment().isTriggered() &&
                             linksFilterActuatorsByPanelAndType(trackingPanel->getLinkages(), trackingPanel.get(), Helio_ActuatorType_PanelHeater).size();
        bool isStorming = trackingPanel && trackingPanel->getStormingTriggerAttachment().isTriggered();

        switch (stage) {
            case Init: {
                if (afterSunset) { // storm trigger handled in later update
                    stage = Cover; stageStart = time;
                    setupStaging();
                } else if (afterSunrise || cleaningDue) {
                    stage = Uncover; stageStart = time;
                    setupStaging();
                } else if (preHeatingDue) {
                    stage = Warm; stageStart = time;
                    setupStaging();
                } else { // before-sunrise / fail-safe
                    stage = Cover; stageStart = time;
                    setupStaging();
                }
            } break;

            case Warm: {
                if (afterSunset || isStorming) {
                    stage = Cover; stageStart = time;
                    setupStaging(); logStage = true;
                } else if (afterSunrise || cleaningDue) {
                    stage = Uncover; stageStart = time;
                    setupStaging();  logStage = true;
                } // else running heating
            } break;

            case Uncover: {
                if (afterSunset || isStorming) {
                    stage = Cover; stageStart = time;
                    setupStaging(); logStage = true;
                } else if (!panel->getPanelCoverDriver() || panel->getPanelCoverDriver()->isAligned()) {
                    stage = (cleaningDue ? Clean : Track); stageStart = time;
                    setupStaging(); logStage = true;
                } // else running uncover
            } break;

            case Clean: {
                if (afterSunset || isStorming) {
                    stage = Cover; stageStart = time;
                    setupStaging(); logStage = true;
                } if (time >= stageStart + (getScheduler()->schedulerData()->preDawnCleaningMins * SECS_PER_MIN)) {
                    stage = Track; stageStart = time;
                    setupStaging(); logStage = true;

                    if (trackingPanel) { trackingPanel->notifyPanelCleaned(); }
                } // else running cleaning
            } break;

            case Track: {
                if (afterSunset || isStorming) {
                    stage = Cover; stageStart = time;
                    setupStaging(); logStage = true;
                } // else running tracking
            } break;

            case Cover: {
                if (afterSunset || isStorming) { // running cover
                    if (afterSunset && !nightSeqReported) { stormingReported = false; logStage = true; }
                } else if (afterSunrise || cleaningDue) {
                    stage = Uncover; stageStart = time;
                    setupStaging(); logStage = true;
                } else if (preHeatingDue) {
                    stage = Warm; stageStart = time;
                    setupStaging(); logStage = true;
                } // else before-sunrise / running cover

                if (coverSeqReported && panel->getPanelCoverDriver() && panel->getPanelCoverDriver()->isAligned()) {
                    getLogger()->logProcess(panel.get(), SFP(HStr_Log_CoverSequence), SFP(HStr_Log_HasEnded));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Time_Measured), timeSpanToString(TimeSpan(time - stageStart)));
                    coverSeqReported = false;
                }
            } break;

            default:
                break;
        }

        if (logStage) {
            if (stageWas != stage) {
                switch (stageWas) {
                    case Warm: {
                        getLogger()->logProcess(panel.get(), SFP(HStr_Log_PreDawnWarmup), SFP(HStr_Log_HasEnded));
                        getLogger()->logMessage(SFP(HStr_Log_Field_Time_Measured), timeSpanToString(TimeSpan(time - stageStart)));
                    } break;

                    case Uncover: {
                        if (panel->getPanelCoverDriver() && panel->getPanelCoverDriver()->isAligned()) {
                            getLogger()->logProcess(panel.get(), SFP(HStr_Log_UncoverSequence), SFP(HStr_Log_HasEnded));
                            getLogger()->logMessage(SFP(HStr_Log_Field_Time_Measured), timeSpanToString(TimeSpan(time - stageStart)));
                        }
                    } break;

                    case Clean: {
                        getLogger()->logProcess(panel.get(), SFP(HStr_Log_PreDawnCleaning), SFP(HStr_Log_HasEnded));
                        getLogger()->logMessage(SFP(HStr_Log_Field_Time_Measured), timeSpanToString(TimeSpan(time - stageStart)));
                    } break;

                    case Track: {
                        getLogger()->logProcess(panel.get(), SFP(HStr_Log_TrackingSequence), SFP(HStr_Log_HasEnded));
                        getLogger()->logMessage(SFP(HStr_Log_Field_Time_Measured), timeSpanToString(TimeSpan(time - stageStart)));
                    } break;

                    case Cover: {
                        if (stormingReported) {
                            getLogger()->logProcess(panel.get(), SFP(HStr_Log_StormingSequence), SFP(HStr_Log_HasEnded));
                            getLogger()->logMessage(SFP(HStr_Log_Field_Time_Measured), timeSpanToString(TimeSpan(time - stageStart)));
                            stormingReported = false;
                        } else if (nightSeqReported) {
                            getLogger()->logProcess(panel.get(), SFP(HStr_Log_NightSequence), SFP(HStr_Log_HasEnded));
                            getLogger()->logMessage(SFP(HStr_Log_Field_Time_Measured), timeSpanToString(TimeSpan(time - stageStart)));
                            nightSeqReported = false;
                        }
                        if (coverSeqReported) {
                            coverSeqReported = false;
                        }
                    } break;
                }
            }

            switch (stage) {
                case Warm: {
                    getLogger()->logProcess(panel.get(), SFP(HStr_Log_PreDawnWarmup), SFP(HStr_Log_HasBegan));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Heating_Duration), roundToString(getScheduler()->schedulerData()->preDawnHeatingMins), String('m'));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Time_Start), localTime(stageStart).timestamp(DateTime::TIMESTAMP_TIME));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Time_Finish), sunrise.timestamp(DateTime::TIMESTAMP_TIME));
                } break;

                case Uncover: {
                    if (panel->getPanelCoverDriver() && panel->getPanelCoverDriver()->getActuators().size()) {
                        getLogger()->logProcess(panel.get(), SFP(HStr_Log_UncoverSequence), SFP(HStr_Log_HasBegan));
                    }
                } break;

                case Clean: {
                    getLogger()->logProcess(panel.get(), SFP(HStr_Log_PreDawnCleaning), SFP(HStr_Log_HasBegan));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Cleaning_Duration), roundToString(getScheduler()->schedulerData()->preDawnCleaningMins), String('m'));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Time_Start), localTime(stageStart).timestamp(DateTime::TIMESTAMP_TIME));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Time_Finish), localTime(stageStart + (getScheduler()->schedulerData()->preDawnCleaningMins * SECS_PER_MIN)).timestamp(DateTime::TIMESTAMP_TIME));
                } break;

                case Track: {
                    TimeSpan daySpan = sunrise - sunset;
                    getLogger()->logProcess(panel.get(), SFP(HStr_Log_TrackingSequence), SFP(HStr_Log_HasBegan));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Light_Duration), roundToString(daySpan.totalseconds() / (float)SECS_PER_HOUR, 1), String('h'));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Time_Start), localTime(stageStart).timestamp(DateTime::TIMESTAMP_TIME));
                    getLogger()->logMessage(SFP(HStr_Log_Field_Time_Finish), sunset.timestamp(DateTime::TIMESTAMP_TIME));
                } break;

                case Cover: {
                    if (isStorming && !afterSunset && (afterSunrise || cleaningDue || preHeatingDue) && !stormingReported) {
                        getLogger()->logProcess(panel.get(), SFP(HStr_Log_StormingSequence), SFP(HStr_Log_HasBegan));
                        stormingReported = true;
                    } else if (afterSunset && !nightSeqReported) {
                        getLogger()->logProcess(panel.get(), SFP(HStr_Log_NightSequence), SFP(HStr_Log_HasBegan));
                        nightSeqReported = true;
                    }
                    if (panel->getPanelCoverDriver() && !panel->getPanelCoverDriver()->isAligned() && !coverSeqReported) {
                        getLogger()->logProcess(panel.get(), SFP(HStr_Log_CoverSequence), SFP(HStr_Log_HasBegan));
                        coverSeqReported = true;
                    }
                } break;

                default:
                    break;
            }
        }
    }

    if (actuatorReqs.size()) {
        for (auto attachIter = actuatorReqs.begin(); attachIter != actuatorReqs.end(); ++attachIter) {
            attachIter->setupActivation();
            attachIter->enableActivation();
        }
    }

    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFU2 = (int8_t)-1; if (_stageFU2 != (int8_t)stage) {
        Serial.print(F("Tracking::~update stage: ")); Serial.println((_stageFU2 = (int8_t)stage)); flushYield(); } }
    #endif
}


HelioSchedulerSubData::HelioSchedulerSubData()
    : HelioSubData(0), reportInterval(8 * SECS_PER_HOUR), cleaningIntervalDays(14), preDawnCleaningMins(1), preDawnHeatingMins(30)
{ ; }

void HelioSchedulerSubData::toJSONObject(JsonObject &objectOut) const
{
    //HelioSubData::toJSONObject(objectOut); // purposeful no call to base method (ignores type)

    if (cleaningIntervalDays != 14) { objectOut[SFP(HStr_Key_CleaningIntervalDays)] = cleaningIntervalDays; }
    if (preDawnCleaningMins != 1) { objectOut[SFP(HStr_Key_PreDawnCleaningMins)] = preDawnCleaningMins; }
    if (preDawnHeatingMins != 30) { objectOut[SFP(HStr_Key_PreDawnHeatingMins)] = preDawnHeatingMins; }
    if (reportInterval != (8 * SECS_PER_HOUR)) { objectOut[SFP(HStr_Key_ReportInterval)] = reportInterval; }
}

void HelioSchedulerSubData::fromJSONObject(JsonObjectConst &objectIn)
{
    //HelioSubData::fromJSONObject(objectIn); // purposeful no call to base method (ignores type)

    cleaningIntervalDays = objectIn[SFP(HStr_Key_CleaningIntervalDays)] | cleaningIntervalDays;
    preDawnCleaningMins = objectIn[SFP(HStr_Key_PreDawnCleaningMins)] | preDawnCleaningMins;
    preDawnHeatingMins = objectIn[SFP(HStr_Key_PreDawnHeatingMins)] | preDawnHeatingMins;
    reportInterval = objectIn[SFP(HStr_Key_ReportInterval)] | reportInterval;
}

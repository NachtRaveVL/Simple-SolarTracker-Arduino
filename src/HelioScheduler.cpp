/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Scheduler
*/

#include "Helioduino.h"

HelioScheduler::HelioScheduler()
    : _needsScheduling(false), _inDaytimeMode(false), _lastDayNum(-1)
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

        {   DateTime currTime = getCurrentTime();
            bool daytimeMode = _dailyTwilight.isDaytime();

            if (_inDaytimeMode != daytimeMode) {
                _inDaytimeMode = daytimeMode;
                setNeedsScheduling();

                if (Helioduino::_activeInstance->_activeUIInstance) {
                    Helioduino::_activeInstance->_activeUIInstance->setNeedsLayout();
                }
            }

            if (_lastDayNum != currTime.day()) {
                // only log uptime upon actual day change and if uptime has been at least 1d
                if (getLoggerInstance()->getSystemUptime() >= SECS_PER_DAY) {
                    getLoggerInstance()->logSystemUptime();
                }
                broadcastDayChange();
            }
        }

        if (_needsScheduling) { performScheduling(); }

        for (auto trackingIter = _trackings.begin(); trackingIter != _trackings.end(); ++trackingIter) {
            trackingIter->second->update();
        }

        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("Scheduler::~update")); flushYield();
        #endif
    }
}

void HelioScheduler::setupServoDriver(HelioPanel *panel, SharedPtr<HelioDriver> servoDriver)
{
    if (panel && servoDriver) {
        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> incActuators;
            auto phUpMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_PanelType_PhUpSolution);
            float dosingRate = getCombinedDosingRate(panel, Helio_PanelType_PhUpSolution);

            linksResolveActuatorsWithRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(phUpMotors, dosingRate, incActuators, Helio_ActuatorType_PeristalticMotor);
            if (!incActuators.size()) { // prefer peristaltic, else use full motor
                linksResolveActuatorsWithRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(phUpMotors, dosingRate, incActuators, Helio_ActuatorType_WaterMotor);
            }

            servoDriver->setIncrementActuators(incActuators);
        }

        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> decActuators;
            auto phDownMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_PanelType_PhDownSolution);
            float dosingRate = getCombinedDosingRate(panel, Helio_PanelType_PhDownSolution);

            linksResolveActuatorsWithRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(phDownMotors, dosingRate, decActuators, Helio_ActuatorType_PeristalticMotor);
            if (!decActuators.size()) { // prefer peristaltic, else use full motor
                linksResolveActuatorsWithRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(phDownMotors, dosingRate, decActuators, Helio_ActuatorType_WaterMotor);
            }

            servoDriver->setDecrementActuators(decActuators);
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
    _lastDayNum = getCurrentTime().day();

    Location loc = getHelioInstance()->getSystemLocation();
    if (loc.hasPosition()) {
        double transit;
        calcSunriseSunset((unsigned long)unixNow(), loc.latitude, loc.longitude, transit, _dailyTwilight.sunrise, _dailyTwilight.sunset,
                          loc.hasAltitude() ? loc.altitude : SUNRISESET_STD_ALTITUDE, HELIO_SYS_SUNRISESET_CALCITERS);
    }
    _inDaytimeMode = _dailyTwilight.isDaytime();

    setNeedsScheduling();

    if (Helioduino::_activeInstance->_activeUIInstance) {
        Helioduino::_activeInstance->_activeUIInstance->setNeedsLayout();
    }
}

void HelioScheduler::performScheduling()
{
    HELIO_HARD_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    for (auto iter = Helioduino::_activeInstance->_objects.begin(); iter != Helioduino::_activeInstance->_objects.end(); ++iter) {
        if (iter->second->isPanelType()) {
            auto panel = static_pointer_cast<HelioPanel>(iter->second);

            {   auto trackingIter = _trackings.find(panel->getKey());

                if (trackingIter != _trackings.end()) {
                    if (trackingIter->second) {
                        trackingIter->second->recalcTracking();
                    }
                } else {
                    #ifdef HELIO_USE_VERBOSE_OUTPUT
                        Serial.print(F("Scheduler::performScheduling Crop linkages found for: ")); Serial.print(iter->second->getKeyString());
                        Serial.print(':'); Serial.print(' '); Serial.println(linksCountCrops(panel->getLinkages())); flushYield();
                    #endif

                    HelioTracking *tracking = new HelioTracking(panel);
                    HELIO_SOFT_ASSERT(tracking, SFP(HStr_Err_AllocationFailure));
                    if (tracking) { _trackings[panel->getKey()] = tracking; }
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
            if (getHelioInstance()) {
                getHelioInstance()->notifyDayChanged();
            }
            yield();
            if (getLoggerInstance()) {
                getLoggerInstance()->notifyDayChanged();
            }
            yield();
            if (getPublisherInstance()) {
                getPublisherInstance()->notifyDayChanged();
            }
            yield();
        });
    #else
        if (getHelioInstance()) {
            getHelioInstance()->notifyDayChanged();
        }
        if (getLoggerInstance()) {
            getLoggerInstance()->notifyDayChanged();
        }
        if (getPublisherInstance()) {
            getPublisherInstance()->notifyDayChanged();
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

void HelioProcess::setActuatorReqs(const Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> &actuatorReqsIn)
{
    for (auto activationIter = actuatorReqs.begin(); activationIter != actuatorReqs.end(); ++activationIter) {
        bool found = false;
        auto key = activationIter->getKey();
    
        for (auto activationInIter = actuatorReqsIn.begin(); activationInIter != actuatorReqsIn.end(); ++activationInIter) {
            if (key == activationInIter->getKey()) {
                found = true;
                break;
            }
        }
    
        if (!found) { // disables actuators not found in new list
            activationIter->disableActivation();
        }
    }

    {   actuatorReqs.clear();
        for (auto activationInIter = actuatorReqsIn.begin(); activationInIter != actuatorReqsIn.end(); ++activationInIter) {
            actuatorReqs.push_back(*activationInIter);
            actuatorReqs.back().setParent(nullptr);
        }
    }
}


HelioTracking::HelioTracking(SharedPtr<HelioPanel> panel)
    : HelioProcess(panel), stage(Unknown), canTrackAfter(0), lastAirReport(0),
      phSetpoint(0), tdsSetpoint(0), waterTempSetpoint(0), airTempSetpoint(0), co2Setpoint(0)
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
    recalcTracking();
}

void HelioTracking::recalcTracking()
{
    float totalWeights = 0;
    float totalSetpoints[5] = {0,0,0,0,0};

    {   auto crops = linksFilterCrops(panel->getLinkages());
        for (auto panelIter = crops.begin(); panelIter != crops.end(); ++panelIter) {
            auto crop = (HelioPanel *)(*panelIter);
            auto cropsLibData = helioPanelsLib.checkoutCropsData(crop->getCropType());

            if (cropsLibData) {
                float weight = crop->getTrackingWeight();
                totalWeights += weight;

                float trackRate = ((cropsLibData->tdsRange[0] + cropsLibData->tdsRange[1]) * 0.5);
                if (!getSchedulerInstance()->inDaytimeMode()) {
                    trackRate *= cropsLibData->nightlyTrackRate;
                }
                trackRate *= getSchedulerInstance()->getBaseTrackMultiplier();

                totalSetpoints[0] += trackRate * weight;
                totalSetpoints[1] += ((cropsLibData->phRange[0] + cropsLibData->phRange[1]) * 0.5) * weight;
                totalSetpoints[2] += ((cropsLibData->waterTempRange[0] + cropsLibData->waterTempRange[1]) * 0.5) * weight;
                totalSetpoints[3] += ((cropsLibData->airTempRange[0] + cropsLibData->airTempRange[1]) * 0.5) * weight;
                totalSetpoints[4] += cropsLibData->co2Levels[(crop->getCropPhase() <= Helio_CropPhase_Vegetative ? 0 : 1)] * weight;

                helioPanelsLib.returnCropsData(cropsLibData);
            }
        }
    }

    if (totalWeights < FLT_EPSILON) {
        totalWeights = 1.0f;
        totalSetpoints[0] = 1;
        totalSetpoints[1] = 6;
    }

    tdsSetpoint = totalSetpoints[0] / totalWeights;
    phSetpoint = tdsSetpoint > FLT_EPSILON ? (totalSetpoints[1] / totalWeights) : 7.0f; // handle flushing
    waterTempSetpoint = totalSetpoints[2] / totalWeights;
    airTempSetpoint = totalSetpoints[3] / totalWeights;
    co2Setpoint = totalSetpoints[4] / totalWeights;

    #ifdef HELIO_USE_VERBOSE_OUTPUT // only works for singular track res in system, otherwise output will be erratic
    {   static float _totalSetpoints[5] = {0,0,0,0,0};
        if (!isFPEqual(_totalSetpoints[0], totalSetpoints[0]) || !isFPEqual(_totalSetpoints[1], totalSetpoints[1]) || !isFPEqual(_totalSetpoints[2], totalSetpoints[2]) || !isFPEqual(_totalSetpoints[3], totalSetpoints[3]) || !isFPEqual(_totalSetpoints[4], totalSetpoints[4])) {
            _totalSetpoints[0] = totalSetpoints[0]; _totalSetpoints[1] = totalSetpoints[1]; _totalSetpoints[2] = totalSetpoints[2]; _totalSetpoints[3] = totalSetpoints[3]; _totalSetpoints[4] = totalSetpoints[4];
            Serial.print(F("Tracking::recalcTracking setpoints: {tds,pH,wTmp,aTmp,aCO2} = [")); Serial.print(_totalSetpoints[0]); Serial.print(' '); Serial.print(_totalSetpoints[1]); Serial.print(' '); Serial.print(_totalSetpoints[2]); Serial.print(' '); Serial.print(_totalSetpoints[3]); Serial.print(' '); Serial.print(_totalSetpoints[4]); Serial.println(']'); flushYield(); } }
    #endif

    setupStaging();
}

void HelioTracking::setupStaging()
{
    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFS1 = (int8_t)-1; if (_stageFS1 != (int8_t)stage) {
        Serial.print(F("Tracking::setupStaging stage: ")); Serial.println((_stageFS1 = (int8_t)stage)); flushYield(); } }
    #endif

    if (stage == PreTrack) {
        if (panel->getWaterPHSensor()) {
            auto phBalancer = panel->getServoDriver();
            if (!phBalancer) {
                phBalancer = SharedPtr<HelioMotorDriver>(new HelioMotorDriver(panel->getWaterPHSensor(), phSetpoint, HELIO_RANGE_PH_HALF, panel->getMaxVolume(), panel->getVolumeUnits()));
                HELIO_SOFT_ASSERT(phBalancer, SFP(HStr_Err_AllocationFailure));
                getSchedulerInstance()->setupServoDriver(panel.get(), phBalancer);
                panel->setServoDriver(phBalancer);
            }
            if (phBalancer) {
                phBalancer->setTargetSetpoint(phSetpoint);
                phBalancer->setTargetUnits(Helio_UnitsType_Alkalinity_pH_0_14);
                phBalancer->setEnabled(true);
            }
        }
        if (panel->getWaterTDSSensor()) {
            auto tdsBalancer = panel->getWaterTDSBalancer();
            if (!tdsBalancer) {
                tdsBalancer = SharedPtr<HelioMotorDriver>(new HelioMotorDriver(panel->getWaterTDSSensor(), tdsSetpoint, HELIO_RANGE_EC_HALF, panel->getMaxVolume(), panel->getVolumeUnits()));
                HELIO_SOFT_ASSERT(tdsBalancer, SFP(HStr_Err_AllocationFailure));
                getSchedulerInstance()->setupWaterTDSBalancer(panel.get(), tdsBalancer);
                panel->setWaterTDSBalancer(tdsBalancer);
            }
            if (tdsBalancer) {
                tdsBalancer->setTargetSetpoint(tdsSetpoint);
                tdsBalancer->setTargetUnits(Helio_UnitsType_Concentration_EC);
                tdsBalancer->setEnabled(true);
            }
        }
    } else {
        auto phBalancer = panel->getServoDriver();
        if (phBalancer) { phBalancer->setEnabled(false); }
        auto tdsBalancer = panel->getWaterTDSBalancer();
        if (tdsBalancer) { tdsBalancer->setEnabled(false); }
    }

    if ((stage == PreTrack || stage == Track) && panel->getWaterTemperatureSensor()) {
        auto waterTempBalancer = panel->getWaterTemperatureBalancer();
        if (!waterTempBalancer) {
            waterTempBalancer = SharedPtr<HelioServoDriver>(new HelioServoDriver(panel->getWaterTemperatureSensor(), waterTempSetpoint, HELIO_RANGE_TEMP_HALF, -HELIO_RANGE_TEMP_HALF * 0.25f, HELIO_RANGE_TEMP_HALF * 0.5f));
            HELIO_SOFT_ASSERT(waterTempBalancer, SFP(HStr_Err_AllocationFailure));
            getSchedulerInstance()->setupWaterTemperatureBalancer(panel.get(), waterTempBalancer);
            panel->setWaterTemperatureBalancer(waterTempBalancer);
        }
        if (waterTempBalancer) {
            waterTempBalancer->setTargetSetpoint(waterTempSetpoint);
            waterTempBalancer->setTargetUnits(Helio_UnitsType_Temperature_Celsius);
            waterTempBalancer->setEnabled(true);
        }
    } else {
        auto waterTempBalancer = panel->getWaterTemperatureBalancer();
        if (waterTempBalancer) { waterTempBalancer->setEnabled(false); }
    }

    if (panel->getAirTemperatureSensor()) {
        auto airTempBalancer = panel->getAirTemperatureBalancer();
        if (!airTempBalancer) {
            airTempBalancer = SharedPtr<HelioServoDriver>(new HelioServoDriver(panel->getAirTemperatureSensor(), airTempSetpoint, HELIO_RANGE_TEMP_HALF, -HELIO_RANGE_TEMP_HALF * 0.25f, HELIO_RANGE_TEMP_HALF * 0.5f));
            HELIO_SOFT_ASSERT(airTempBalancer, SFP(HStr_Err_AllocationFailure));
            getSchedulerInstance()->setupAirTemperatureBalancer(panel.get(), airTempBalancer);
            panel->setAirTemperatureBalancer(airTempBalancer);
        }
        if (airTempBalancer) {
            airTempBalancer->setTargetSetpoint(airTempSetpoint);
            airTempBalancer->setTargetUnits(Helio_UnitsType_Temperature_Celsius);
            airTempBalancer->setEnabled(true);
        }
    } else {
        auto airTempBalancer = panel->getAirTemperatureBalancer();
        if (airTempBalancer) { airTempBalancer->setEnabled(false); }
    }

    if (panel->getAirCO2Sensor()) {
        auto co2Balancer = panel->getAirTemperatureBalancer();
        if (!co2Balancer) {
            co2Balancer = SharedPtr<HelioServoDriver>(new HelioServoDriver(panel->getAirCO2Sensor(), co2Setpoint, HELIO_RANGE_CO2_HALF, -HELIO_RANGE_CO2_HALF * 0.25f, HELIO_RANGE_CO2_HALF * 0.5f));
            HELIO_SOFT_ASSERT(co2Balancer, SFP(HStr_Err_AllocationFailure));
            getSchedulerInstance()->setupAirCO2Balancer(panel.get(), co2Balancer);
            panel->setAirCO2Balancer(co2Balancer);
        }
        if (co2Balancer) {
            co2Balancer->setTargetSetpoint(co2Setpoint);
            co2Balancer->setTargetUnits(Helio_UnitsType_Concentration_PPM);
            co2Balancer->setEnabled(true);
        }
    } else {
        auto co2Balancer = panel->getAirCO2Balancer();
        if (co2Balancer) { co2Balancer->setEnabled(false); }
    }

    switch (stage) {
        case Init: {
            auto maxTrackingsDay = getSchedulerInstance()->getTotalTrackingsDay();
            auto trackingsToday = panel->getTrackingsToday();

            if (!maxTrackingsDay) {
                canTrackAfter = (time_t)0;
            } else if (trackingsToday < maxTrackingsDay) {
                // this will force trackings to be spread out during the entire day
                canTrackAfter = getCurrentDayStartTime() + (time_t)(((float)SECS_PER_DAY / (maxTrackingsDay + 1)) * trackingsToday);
            } else {
                canTrackAfter = (time_t)UINT32_MAX; // no more trackings today
            }

            if (canTrackAfter > unixNow()) { clearActuatorReqs(); } // clear on wait
        } break;

        case TopOff: {
            if (!panel->isFilled()) {
                Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;
                auto topOffMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_SCH_REQACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_PanelType_FreshWater);

                linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(topOffMotors, newActuatorReqs, Helio_ActuatorType_WaterMotor); // fresh water motors
                if (!newActuatorReqs.size()) {
                    linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(topOffMotors, newActuatorReqs, Helio_ActuatorType_PeristalticMotor); // fresh water peristaltic motors
                }

                HELIO_SOFT_ASSERT(newActuatorReqs.size(), SFP(HStr_Err_MissingLinkage)); // no fresh water motors
                setActuatorReqs(newActuatorReqs);
            } else {
                clearActuatorReqs();
            }
        } break;

        case PreTrack: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;
            auto aerators = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_WaterAerator);

            linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(aerators, newActuatorReqs, Helio_ActuatorType_WaterAerator);

            setActuatorReqs(newActuatorReqs);
        } break;

        case Track: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;

            {   auto trackMotors = linksFilterMotorActuatorsByInputPanelAndOutputPanelType<HELIO_SCH_REQACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_PanelType_TrackWater);

                linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(trackMotors, newActuatorReqs, Helio_ActuatorType_WaterMotor); // track water motor
            }

            if (!newActuatorReqs.size() && getHelioInstance()->getSystemMode() == Helio_SystemMode_DrainToWaste) { // prefers track water motors, else direct to waste is track
                auto trackMotors = linksFilterMotorActuatorsByInputPanelAndOutputPanelType<HELIO_SCH_REQACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_PanelType_DrainageWater);

                linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(trackMotors, newActuatorReqs, Helio_ActuatorType_WaterMotor); // DTW track water motor
            }

            HELIO_SOFT_ASSERT(newActuatorReqs.size(), SFP(HStr_Err_MissingLinkage)); // no track water motors

            #if HELIO_SCH_AERATORS_TRACKRUN
                {   auto aerators = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_ActuatorType_WaterAerator);

                    linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(aerators, newActuatorReqs, Helio_ActuatorType_WaterAerator);
                }
            #endif

            setActuatorReqs(newActuatorReqs);
        } break;

        case Drain: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;
            auto drainMotors = linksFilterMotorActuatorsByInputPanelAndOutputPanelType<HELIO_SCH_REQACTUATORS_MAXSIZE>(panel->getLinkages(), panel.get(), Helio_PanelType_DrainageWater);

            linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(drainMotors, newActuatorReqs, Helio_ActuatorType_WaterMotor); // drainage water motor

            HELIO_SOFT_ASSERT(newActuatorReqs.size(), SFP(HStr_Err_MissingLinkage)); // no drainage water motors
            setActuatorReqs(newActuatorReqs);
        } break;

        case Done: {
            clearActuatorReqs();
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

    if ((!lastAirReport || unixNow() >= lastAirReport + getSchedulerInstance()->getAirReportInterval().totalseconds()) &&
        (getSchedulerInstance()->getAirReportInterval().totalseconds() > 0) && // 0 disables
        (panel->getAirTemperatureSensor() || panel->getAirCO2Sensor())) {
        getLoggerInstance()->logProcess(panel.get(), SFP(HStr_Log_AirReport));
        logTracking(HelioTrackingLogType_AirSetpoints);
        logTracking(HelioTrackingLogType_AirMeasures);
        lastAirReport = unixNow();
    }

    switch (stage) {
        case Init: {
            if (!canTrackAfter || unixNow() >= canTrackAfter) {
                int cropsCount = 0;
                int cropsHungry = 0;

                {   auto crops = linksFilterCrops(panel->getLinkages());
                    cropsCount = crops.size();
                    for (auto panelIter = crops.begin(); panelIter != crops.end(); ++panelIter) {
                        if (((HelioPanel *)(*panelIter))->needsTracking()) { cropsHungry++; }
                    }
                }

                if (!cropsCount || cropsHungry / (float)cropsCount >= HELIO_SCH_TRACK_FRACTION - FLT_EPSILON) {
                    stage = TopOff; stageStart = unixNow();
                    setupStaging();

                    if (actuatorReqs.size()) {
                        getLoggerInstance()->logProcess(panel.get(), SFP(HStr_Log_PreTrackTopOff), SFP(HStr_Log_HasBegan));
                    }
                }
            }
        } break;

        case TopOff: {
            if (panel->isFilled() || !actuatorReqs.size()) {
                stage = PreTrack; stageStart = unixNow();
                canTrackAfter = 0; // will be used to track how long balancers stay balanced
                setupStaging();

                getLoggerInstance()->logProcess(panel.get(), SFP(HStr_Log_PreTrackDriving), SFP(HStr_Log_HasBegan));
                if (actuatorReqs.size()) {
                    getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Aerator_Duration), String(getSchedulerInstance()->getPreTrackAeratorMins()), String('m'));
                }
                if (panel->getServoDriver() || panel->getWaterTDSBalancer()) {
                    auto balancer = static_pointer_cast<HelioMotorDriver>(panel->getServoDriver() ? panel->getServoDriver() : panel->getWaterTDSBalancer());
                    if (balancer) {
                        getLoggerInstance()->logMessage(SFP(HStr_Log_Field_MixTime_Duration), timeSpanToString(TimeSpan(balancer->getMixTime())));
                    }
                }
                logTracking(HelioTrackingLogType_WaterSetpoints);
                logTracking(HelioTrackingLogType_WaterMeasures);
            }
        } break;

        case PreTrack: {
            if (!actuatorReqs.size() || unixNow() >= stageStart + (getSchedulerInstance()->getPreTrackAeratorMins() * SECS_PER_MIN)) {
                auto phBalancer = panel->getServoDriver();
                auto tdsBalancer = panel->getWaterTDSBalancer();
                auto waterTempBalancer = panel->getWaterTemperatureBalancer();

                if ((!phBalancer || (phBalancer->isEnabled() && phBalancer->isOnTarget())) &&
                    (!tdsBalancer || (tdsBalancer->isEnabled() && tdsBalancer->isOnTarget())) &&
                    (!waterTempBalancer || (waterTempBalancer->isEnabled() && waterTempBalancer->isOnTarget()))) {
                    // Can proceed after above are marked balanced for min time
                    if (!canTrackAfter) { canTrackAfter = unixNow() + HELIO_SCH_BALANCE_MINTIME; }
                    else if (unixNow() >= canTrackAfter) {
                        stage = Track; stageStart = unixNow();
                        setupStaging();

                        broadcastTracking(HelioTrackingBroadcastType_Began);
                    }
                } else {
                    canTrackAfter = 0;
                }
            }
        } break;

        case Track: {
            int cropsCount = 0;
            int cropsFed = 0;

            {   auto crops = linksFilterCrops(panel->getLinkages());
                cropsCount = crops.size();
                for (auto panelIter = crops.begin(); panelIter != crops.end(); ++panelIter) {
                    if (!((HelioPanel *)(*panelIter))->needsTracking()) { cropsFed++; }
                }
            }

            if (!cropsCount || cropsFed / (float)cropsCount >= HELIO_SCH_TRACK_FRACTION - FLT_EPSILON ||
                panel->isEmpty()) {
                stage = (getHelioInstance()->getSystemMode() == Helio_SystemMode_DrainToWaste ? Drain : Done);
                stageStart = unixNow();
                setupStaging();

                broadcastTracking(HelioTrackingBroadcastType_Ended);
            }
        } break;

        case Drain: {
            if (getHelioInstance()->getSystemMode() != Helio_SystemMode_DrainToWaste ||
                panel->isEmpty()) {
                stage = Done; stageStart = unixNow();
                setupStaging();
            }
        } break;

        case Done: {
            stage = Init; stageStart = unixNow();
            setupStaging();
        } break;

        default:
            break;
    }

    if (actuatorReqs.size()) {
        for (auto activationIter = actuatorReqs.begin(); activationIter != actuatorReqs.end(); ++activationIter) {
            activationIter->enableActivation();
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
            {   auto ph = HelioSingleMeasurement(phSetpoint, Helio_UnitsType_Alkalinity_pH_0_14);
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_pH_Setpoint), measurementToString(ph));
            }
            {   auto tds = HelioSingleMeasurement(tdsSetpoint, Helio_UnitsType_Concentration_TDS);
                convertUnits(&tds, panel->getTDSUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_TDS_Setpoint), measurementToString(tds, 1));
            }
            {   auto temp = HelioSingleMeasurement(waterTempSetpoint, Helio_UnitsType_Temperature_Celsius);
                convertUnits(&temp, panel->getTemperatureUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Temp_Setpoint), measurementToString(temp));
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
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_pH_Measured), measurementToString(ph));
            }
            if (panel->getWaterTDSSensor()) {
                auto tds = panel->getWaterTDS().getMeasurement(true);
                convertUnits(&tds, panel->getTDSUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_TDS_Measured), measurementToString(tds, 1));
            }
            if (panel->getWaterTemperatureSensor()) {
                auto temp = panel->getWaterTemperature().getMeasurement(true);
                convertUnits(&temp, panel->getTemperatureUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Temp_Measured), measurementToString(temp));
            }
            break;

        case HelioTrackingLogType_AirSetpoints:
            {   auto temp = HelioSingleMeasurement(airTempSetpoint, Helio_UnitsType_Temperature_Celsius);
                convertUnits(&temp, panel->getTemperatureUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Temp_Setpoint), measurementToString(temp));
            }
            {   auto co2 = HelioSingleMeasurement(co2Setpoint, Helio_UnitsType_Concentration_PPM);
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_CO2_Setpoint), measurementToString(co2));
            }
            break;

        case HelioTrackingLogType_AirMeasures:
            #ifdef HELIO_USE_MULTITASKING
                // Yield will allow measurements to complete, ensures first log out doesn't contain zero'ed values
                if ((panel->getAirTemperatureSensor() && !panel->getAirTemperature().getMeasurementFrame()) ||
                    (panel->getAirCO2Sensor() && !panel->getAirCO2().getMeasurementFrame())) {
                    yield();
                }
            #endif
            if (panel->getAirTemperatureSensor()) {
                auto temp = panel->getAirTemperature().getMeasurement(true);
                convertUnits(&temp, panel->getTemperatureUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Temp_Measured), measurementToString(temp));
            }
            if (panel->getAirCO2Sensor()) {
                auto co2 = panel->getAirCO2().getMeasurement(true);
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_CO2_Measured), measurementToString(co2));
            }
            break;
    }
}

void HelioTracking::broadcastTracking(HelioTrackingBroadcastType broadcastType)
{
    getLoggerInstance()->logProcess(panel.get(), SFP(HStr_Log_TrackingSequence),
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
    : HelioSubData(0), baseTrackMultiplier(1), weeklyDosingRates{1}, stdDosingRates{1,0.5,0.5},
      totalTrackingsDay(0), preTrackAeratorMins(30), preLightSprayMins(60), airReportInterval(8 * SECS_PER_HOUR)
{ ; }

void HelioSchedulerSubData::toJSONObject(JsonObject &objectOut) const
{
    //HelioSubData::toJSONObject(objectOut); // purposeful no call to base method (ignores type)

    if (!isFPEqual(baseTrackMultiplier, 1.0f)) { objectOut[SFP(HStr_Key_BaseTrackMultiplier)] = baseTrackMultiplier; }
    bool hasWeeklyDosings = arrayElementsEqual(weeklyDosingRates, HELIO_CROP_GROWWEEKS_MAX, 1.0f);
    if (hasWeeklyDosings) { objectOut[SFP(HStr_Key_WeeklyDosingRates)] = commaStringFromArray(weeklyDosingRates, HELIO_CROP_GROWWEEKS_MAX); }
    bool hasStandardDosings = !isFPEqual(stdDosingRates[0], 1.0f) || !isFPEqual(stdDosingRates[1], 0.5f) || !isFPEqual(stdDosingRates[2], 0.5f);
    if (hasStandardDosings) { objectOut[SFP(HStr_Key_StdDosingRates)] = commaStringFromArray(stdDosingRates, 3); }
    if (totalTrackingsDay > 0) { objectOut[SFP(HStr_Key_TotalTrackingsDay)] = totalTrackingsDay; }
    if (preTrackAeratorMins != 30) { objectOut[SFP(HStr_Key_PreTrackAeratorMins)] = preTrackAeratorMins; }
    if (preLightSprayMins != 60) { objectOut[SFP(HStr_Key_PreLightSprayMins)] = preLightSprayMins; }
    if (airReportInterval != (8 * SECS_PER_HOUR)) { objectOut[SFP(HStr_Key_AirReportInterval)] = airReportInterval; }
}

void HelioSchedulerSubData::fromJSONObject(JsonObjectConst &objectIn)
{
    //HelioSubData::fromJSONObject(objectIn); // purposeful no call to base method (ignores type)

    baseTrackMultiplier = objectIn[SFP(HStr_Key_BaseTrackMultiplier)] | baseTrackMultiplier;
    JsonVariantConst weeklyDosingRatesVar = objectIn[SFP(HStr_Key_WeeklyDosingRates)];
    commaStringToArray(weeklyDosingRatesVar, weeklyDosingRates, HELIO_CROP_GROWWEEKS_MAX);
    JsonVariantConst stdDosingRatesVar = objectIn[SFP(HStr_Key_StdDosingRates)];
    commaStringToArray(stdDosingRatesVar, stdDosingRates, 3);
    totalTrackingsDay = objectIn[SFP(HStr_Key_TotalTrackingsDay)] | totalTrackingsDay;
    preTrackAeratorMins = objectIn[SFP(HStr_Key_PreTrackAeratorMins)] | preTrackAeratorMins;
    preLightSprayMins = objectIn[SFP(HStr_Key_PreLightSprayMins)] | preLightSprayMins;
    airReportInterval = objectIn[SFP(HStr_Key_AirReportInterval)] | airReportInterval;
}

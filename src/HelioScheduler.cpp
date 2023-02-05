/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Scheduler
*/

#include "Helioduino.h"

HelioScheduler::HelioScheduler()
    : _inDaytimeMode(false), _needsScheduling(false), _lastDayNum(-1)
{ ; }

HelioScheduler::~HelioScheduler()
{
    while (_feedings.size()) {
        auto feedingIter = _feedings.begin();
        delete feedingIter->second;
        _feedings.erase(feedingIter);
    }
    while (_lightings.size()) {
        auto lightingIter = _lightings.begin();
        delete lightingIter->second;
        _lightings.erase(lightingIter);
    }
}

void HelioScheduler::update()
{
    if (hasSchedulerData()) {
        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("Scheduler::update")); flushYield();
        #endif

        {   DateTime currTime = getCurrentTime();
            bool daytimeMode = currTime.hour() >= HELIO_CROP_NIGHT_ENDHR && currTime.hour() < HELIO_CROP_NIGHT_BEGINHR;

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

        for (auto feedingIter = _feedings.begin(); feedingIter != _feedings.end(); ++feedingIter) {
            feedingIter->second->update();
        }
        for (auto lightingIter = _lightings.begin(); lightingIter != _lightings.end(); ++lightingIter) {
            lightingIter->second->update();
        }

        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("Scheduler::~update")); flushYield();
        #endif
    }
}

void HelioScheduler::setupWaterPHBalancer(HelioPanel *panel, SharedPtr<HelioDriver> waterPHBalancer)
{
    if (panel && waterPHBalancer) {
        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> incActuators;
            auto phUpMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_PanelType_PhUpSolution);
            float dosingRate = getCombinedDosingRate(panel, Helio_PanelType_PhUpSolution);

            linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(phUpMotors, dosingRate, incActuators, Helio_ActuatorType_PeristalticMotor);
            if (!incActuators.size()) { // prefer peristaltic, else use full motor
                linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(phUpMotors, dosingRate, incActuators, Helio_ActuatorType_WaterMotor);
            }

            waterPHBalancer->setIncrementActuators(incActuators);
        }

        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> decActuators;
            auto phDownMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_PanelType_PhDownSolution);
            float dosingRate = getCombinedDosingRate(panel, Helio_PanelType_PhDownSolution);

            linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(phDownMotors, dosingRate, decActuators, Helio_ActuatorType_PeristalticMotor);
            if (!decActuators.size()) { // prefer peristaltic, else use full motor
                linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(phDownMotors, dosingRate, decActuators, Helio_ActuatorType_WaterMotor);
            }

            waterPHBalancer->setDecrementActuators(decActuators);
        }
    }
}

void HelioScheduler::setupWaterTDSBalancer(HelioPanel *panel, SharedPtr<HelioDriver> waterTDSBalancer)
{
    if (panel && waterTDSBalancer) {
        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> incActuators;
            float dosingRate = getCombinedDosingRate(panel, Helio_PanelType_NutrientPremix);

            if (dosingRate > FLT_EPSILON) {
                auto nutrientMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_PanelType_NutrientPremix);

                linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(nutrientMotors, dosingRate, incActuators, Helio_ActuatorType_PeristalticMotor);
                if (!incActuators.size()) { // prefer peristaltic, else use full motor
                    linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(nutrientMotors, dosingRate, incActuators, Helio_ActuatorType_WaterMotor);
                }
            }

            if (helioAdditives.hasCustomAdditives()) {
                int prevIncSize = incActuators.size();

                for (int panelType = Helio_PanelType_CustomAdditive1; panelType < Helio_PanelType_CustomAdditive1 + Helio_PanelType_CustomAdditiveCount; ++panelType) {
                    if (helioAdditives.getCustomAdditiveData((Helio_PanelType)panelType)) {
                        dosingRate = getCombinedDosingRate(panel, (Helio_PanelType)panelType);

                        if (dosingRate > FLT_EPSILON) {
                            auto nutrientMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, (Helio_PanelType)panelType);

                            linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(nutrientMotors, dosingRate, incActuators, Helio_ActuatorType_PeristalticMotor);
                            if (incActuators.size() == prevIncSize) { // prefer peristaltic, else use full motor
                                linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(nutrientMotors, dosingRate, incActuators, Helio_ActuatorType_WaterMotor);
                            }
                        }

                        prevIncSize = incActuators.size();
                    }
                }
            }

            waterTDSBalancer->setIncrementActuators(incActuators);
        }

        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> decActuators;
            float dosingRate = getCombinedDosingRate(panel, Helio_PanelType_FreshWater);

            if (dosingRate > FLT_EPSILON) {
                auto dilutionMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_PanelType_NutrientPremix);

                linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(dilutionMotors, dosingRate, decActuators, Helio_ActuatorType_PeristalticMotor);
                if (!decActuators.size()) { // prefer peristaltic, else use full motor
                    linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(dilutionMotors, dosingRate, decActuators, Helio_ActuatorType_WaterMotor);
                }
            }

            waterTDSBalancer->setDecrementActuators(decActuators);
        }
    }
}

void HelioScheduler::setupWaterTemperatureBalancer(HelioPanel *panel, SharedPtr<HelioDriver> waterTempBalancer)
{
    if (panel && waterTempBalancer) {
        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> incActuators;
            auto heaters = linksFilterActuatorsByPanelAndType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_ActuatorType_WaterHeater);

            linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(heaters, 1.0f, incActuators, Helio_ActuatorType_WaterHeater);

            waterTempBalancer->setIncrementActuators(incActuators);
        }

        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> decActuators;
            waterTempBalancer->setDecrementActuators(decActuators);
        }
    }
}

void HelioScheduler::setupAirTemperatureBalancer(HelioPanel *panel, SharedPtr<HelioDriver> airTempBalancer)
{
    if (panel && airTempBalancer) {
        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> incActuators;
            airTempBalancer->setIncrementActuators(incActuators);
        }

        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> decActuators;
            auto fans = linksFilterActuatorsByPanelAndType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_ActuatorType_FanExhaust);

            linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(fans, 1.0f, decActuators, Helio_ActuatorType_FanExhaust);
        }
    }
}

void HelioScheduler::setupAirCO2Balancer(HelioPanel *panel, SharedPtr<HelioDriver> airCO2Balancer)
{
    if (panel && airCO2Balancer) {
        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> incActuators;
            auto fans = linksFilterActuatorsByPanelAndType<HELIO_DRV_ACTUATORS_MAXSIZE>(panel->getLinkages(), panel, Helio_ActuatorType_FanExhaust);

            linksResolveActuatorsPairRateByType<HELIO_DRV_ACTUATORS_MAXSIZE>(fans, 1.0f, incActuators, Helio_ActuatorType_FanExhaust);

            airCO2Balancer->setIncrementActuators(incActuators);
        }

        {   Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> decActuators;
            airCO2Balancer->setDecrementActuators(decActuators);
        }
    }
}

void HelioScheduler::setBaseFeedMultiplier(float baseFeedMultiplier)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    if (hasSchedulerData()) {
        Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
        schedulerData()->baseFeedMultiplier = baseFeedMultiplier;

        setNeedsScheduling();
    }
}

void HelioScheduler::setWeeklyDosingRate(int weekIndex, float dosingRate, Helio_PanelType panelType)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(!hasSchedulerData() || (weekIndex >= 0 && weekIndex < HELIO_CROP_GROWWEEKS_MAX), SFP(HStr_Err_InvalidParameter));

    if (hasSchedulerData() && weekIndex >= 0 && weekIndex < HELIO_CROP_GROWWEEKS_MAX) {
        if (panelType == Helio_PanelType_NutrientPremix) {
            Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
            schedulerData()->weeklyDosingRates[weekIndex] = dosingRate;

            setNeedsScheduling();
        } else if (panelType >= Helio_PanelType_CustomAdditive1 && panelType < Helio_PanelType_CustomAdditive1 + Helio_PanelType_CustomAdditiveCount) {
            HelioCustomAdditiveData newAdditiveData(panelType);
            newAdditiveData._bumpRevIfNotAlreadyModded();
            newAdditiveData.weeklyDosingRates[weekIndex] = dosingRate;
            helioAdditives.setCustomAdditiveData(&newAdditiveData);

            setNeedsScheduling();
        } else {
            HELIO_SOFT_ASSERT(false, SFP(HStr_Err_UnsupportedOperation));
        }
    }
}

void HelioScheduler::setStandardDosingRate(float dosingRate, Helio_PanelType panelType)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(!hasSchedulerData() || (panelType >= Helio_PanelType_FreshWater && panelType < Helio_PanelType_CustomAdditive1), SFP(HStr_Err_InvalidParameter));

    if (hasSchedulerData() && (panelType >= Helio_PanelType_FreshWater && panelType < Helio_PanelType_CustomAdditive1)) {
        Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
        schedulerData()->stdDosingRates[panelType - Helio_PanelType_FreshWater] = dosingRate;

        setNeedsScheduling();
    }
}

void HelioScheduler::setFlushWeek(int weekIndex)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(!hasSchedulerData() || (weekIndex >= 0 && weekIndex < HELIO_CROP_GROWWEEKS_MAX), SFP(HStr_Err_InvalidParameter));

    if (hasSchedulerData() && weekIndex >= 0 && weekIndex < HELIO_CROP_GROWWEEKS_MAX) {
        schedulerData()->weeklyDosingRates[weekIndex] = 0;

        for (Helio_PanelType panelType = Helio_PanelType_CustomAdditive1;
             panelType < Helio_PanelType_CustomAdditive1 + Helio_PanelType_CustomAdditiveCount;
             panelType = (Helio_PanelType)((int)panelType + 1)) {
            auto additiveData = helioAdditives.getCustomAdditiveData(panelType);
            if (additiveData) {
                HelioCustomAdditiveData newAdditiveData = *additiveData;
                newAdditiveData._bumpRevIfNotAlreadyModded();
                newAdditiveData.weeklyDosingRates[weekIndex] = 0;
                helioAdditives.setCustomAdditiveData(&newAdditiveData);
            }
        }

        setNeedsScheduling();
    }
}

void HelioScheduler::setTotalFeedingsDay(unsigned int feedingsDay)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasSchedulerData() && schedulerData()->totalFeedingsDay != feedingsDay) {
        Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
        schedulerData()->totalFeedingsDay = feedingsDay;

        setNeedsScheduling();
    }
}

void HelioScheduler::setPreFeedAeratorMins(unsigned int aeratorMins)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasSchedulerData() && schedulerData()->preFeedAeratorMins != aeratorMins) {
        Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
        schedulerData()->preFeedAeratorMins = aeratorMins;

        setNeedsScheduling();
    }
}

void HelioScheduler::setPreLightSprayMins(unsigned int sprayMins)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    if (hasSchedulerData() && schedulerData()->preLightSprayMins != sprayMins) {
        Helioduino::_activeInstance->_systemData->_bumpRevIfNotAlreadyModded();
        schedulerData()->preLightSprayMins = sprayMins;

        setNeedsScheduling();
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

float HelioScheduler::getCombinedDosingRate(HelioPanel *panel, Helio_PanelType panelType)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(!hasSchedulerData() || panel, SFP(HStr_Err_InvalidParameter));
    HELIO_SOFT_ASSERT(!hasSchedulerData() || !panel || (panelType >= Helio_PanelType_NutrientPremix &&
                                                               panelType < Helio_PanelType_CustomAdditive1 + Helio_PanelType_CustomAdditiveCount), SFP(HStr_Err_InvalidParameter));

    if (hasSchedulerData() && panel &&
        (panelType >= Helio_PanelType_NutrientPremix &&
         panelType < Helio_PanelType_CustomAdditive1 + Helio_PanelType_CustomAdditiveCount)) {
        auto crops = linksFilterCrops(panel->getLinkages());
        float totalWeights = 0;
        float totalDosing = 0;

        for (auto cropIter = crops.begin(); cropIter != crops.end(); ++cropIter) {
            auto crop = (HelioCrop *)(*cropIter);
            if (crop) {
                if (panelType <= Helio_PanelType_NutrientPremix) {
                    totalWeights += crop->getFeedingWeight();
                    totalDosing += schedulerData()->weeklyDosingRates[constrain(crop->getGrowWeek(), 0, crop->getTotalGrowWeeks() - 1)];
                } else if (panelType < Helio_PanelType_CustomAdditive1) {
                    totalWeights += crop->getFeedingWeight();
                    totalDosing += schedulerData()->stdDosingRates[panelType - Helio_PanelType_FreshWater];
                } else {
                    auto additiveData = helioAdditives.getCustomAdditiveData(panelType);
                    if (additiveData) {
                        totalWeights += crop->getFeedingWeight();
                        totalDosing += additiveData->weeklyDosingRates[constrain(crop->getGrowWeek(), 0, crop->getTotalGrowWeeks() - 1)];
                    }
                }
            }
        }

        if (totalWeights <= FLT_EPSILON) {
            totalWeights = 1.0f;
        }

        return totalDosing / totalWeights;
    }

    return 0.0f;
}

float HelioScheduler::getBaseFeedMultiplier() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return hasSchedulerData() ? schedulerData()->baseFeedMultiplier : 1.0f;
}

float HelioScheduler::getWeeklyDosingRate(int weekIndex, Helio_PanelType panelType) const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(!hasSchedulerData() || (weekIndex >= 0 && weekIndex < HELIO_CROP_GROWWEEKS_MAX), SFP(HStr_Err_InvalidParameter));

    if (hasSchedulerData() && weekIndex >= 0 && weekIndex < HELIO_CROP_GROWWEEKS_MAX) {
        if (panelType == Helio_PanelType_NutrientPremix) {
            return schedulerData()->weeklyDosingRates[weekIndex];
        } else if (panelType >= Helio_PanelType_CustomAdditive1 && panelType < Helio_PanelType_CustomAdditive1 + Helio_PanelType_CustomAdditiveCount) {
            auto additiveDate = helioAdditives.getCustomAdditiveData(panelType);
            return additiveDate ? additiveDate->weeklyDosingRates[weekIndex] : 0.0f;
        } else {
            HELIO_SOFT_ASSERT(false, SFP(HStr_Err_UnsupportedOperation));
        }
    }

    return 0.0f;
}

float HelioScheduler::getStandardDosingRate(Helio_PanelType panelType) const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(!hasSchedulerData() || (panelType >= Helio_PanelType_FreshWater && panelType < Helio_PanelType_CustomAdditive1), SFP(HStr_Err_InvalidParameter));

    if (hasSchedulerData() && panelType >= Helio_PanelType_FreshWater && panelType < Helio_PanelType_CustomAdditive1) {
        return schedulerData()->stdDosingRates[panelType - Helio_PanelType_FreshWater];
    }

    return 0.0f;
}

bool HelioScheduler::isFlushWeek(int weekIndex)
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(!hasSchedulerData() || (weekIndex >= 0 && weekIndex < HELIO_CROP_GROWWEEKS_MAX), SFP(HStr_Err_InvalidParameter));

    if (hasSchedulerData() && weekIndex >= 0 && weekIndex < HELIO_CROP_GROWWEEKS_MAX) {
        return isFPEqual(schedulerData()->weeklyDosingRates[weekIndex], 0.0f);
    }

    return false;
}

unsigned int HelioScheduler::getTotalFeedingsDay() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return hasSchedulerData() ? schedulerData()->totalFeedingsDay : 0;
}

unsigned int HelioScheduler::getPreFeedAeratorMins() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return hasSchedulerData() ? schedulerData()->preFeedAeratorMins : 0;
}

unsigned int HelioScheduler::getPreLightSprayMins() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return hasSchedulerData() ? schedulerData()->preLightSprayMins : 0;
}

TimeSpan HelioScheduler::getAirReportInterval() const
{
    HELIO_SOFT_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));
    return TimeSpan(hasSchedulerData() ? schedulerData()->airReportInterval : 0);
}

void HelioScheduler::updateDayTracking()
{
    auto currTime = getCurrentTime();
    _lastDayNum = currTime.day();
    _inDaytimeMode = currTime.hour() >= HELIO_CROP_NIGHT_ENDHR && currTime.hour() < HELIO_CROP_NIGHT_BEGINHR;

    setNeedsScheduling();

    if (Helioduino::_activeInstance->_activeUIInstance) {
        Helioduino::_activeInstance->_activeUIInstance->setNeedsLayout();
    }
}

void HelioScheduler::performScheduling()
{
    HELIO_HARD_ASSERT(hasSchedulerData(), SFP(HStr_Err_NotYetInitialized));

    for (auto iter = Helioduino::_activeInstance->_objects.begin(); iter != Helioduino::_activeInstance->_objects.end(); ++iter) {
        if (iter->second->isPanelType() && ((HelioPanel *)(iter->second.get()))->isFeedClass()) {
            auto feedPanel = static_pointer_cast<HelioFeedPanel>(iter->second);

            {   auto feedingIter = _feedings.find(feedPanel->getKey());

                if (linksCountCrops(feedPanel->getLinkages())) {
                    if (feedingIter != _feedings.end()) {
                        if (feedingIter->second) {
                            feedingIter->second->recalcFeeding();
                        }
                    } else {
                        #ifdef HELIO_USE_VERBOSE_OUTPUT
                            Serial.print(F("Scheduler::performScheduling Crop linkages found for: ")); Serial.print(iter->second->getKeyString());
                            Serial.print(':'); Serial.print(' '); Serial.println(linksCountCrops(feedPanel->getLinkages())); flushYield();
                        #endif

                        HelioFeeding *feeding = new HelioFeeding(feedPanel);
                        HELIO_SOFT_ASSERT(feeding, SFP(HStr_Err_AllocationFailure));
                        if (feeding) { _feedings[feedPanel->getKey()] = feeding; }
                    }
                } else if (feedingIter != _feedings.end()) { // No crops to warrant process -> delete if exists
                    #ifdef HELIO_USE_VERBOSE_OUTPUT
                        Serial.print(F("Scheduler::performScheduling NO more crop linkages found for: ")); Serial.println(iter->second->getKeyString()); flushYield();
                    #endif
                    if (feedingIter->second) { delete feedingIter->second; }
                    _feedings.erase(feedingIter);
                }
            }

            {   auto lightingIter = _lightings.find(feedPanel->getKey());

                if (linksCountActuatorsByPanelAndType(feedPanel->getLinkages(), feedPanel.get(), Helio_ActuatorType_PanelCleaner) ||
                    linksCountActuatorsByPanelAndType(feedPanel->getLinkages(), feedPanel.get(), Helio_ActuatorType_GrowLights)) {
                    if (lightingIter != _lightings.end()) {
                        if (lightingIter->second) {
                            lightingIter->second->recalcLighting();
                        }
                    } else {
                        #ifdef HELIO_USE_VERBOSE_OUTPUT
                            Serial.print(F("Scheduler::performScheduling Light linkages found for: ")); Serial.print(iter->second->getKeyString()); Serial.print(':'); Serial.print(' ');
                            Serial.println(linksCountActuatorsByPanelAndType(feedPanel->getLinkages(), feedPanel.get(), Helio_ActuatorType_PanelCleaner) +
                                            linksCountActuatorsByPanelAndType(feedPanel->getLinkages(), feedPanel.get(), Helio_ActuatorType_GrowLights)); flushYield();
                        #endif

                        HelioLighting *lighting = new HelioLighting(feedPanel);
                        HELIO_SOFT_ASSERT(lighting, SFP(HStr_Err_AllocationFailure));
                        if (lighting) { _lightings[feedPanel->getKey()] = lighting; }
                    }
                } else if (lightingIter != _lightings.end()) { // No lights or sprayers to warrant process -> delete if exists
                    #ifdef HELIO_USE_VERBOSE_OUTPUT
                        Serial.print(F("Scheduler::performScheduling NO more light linkages found for: ")); Serial.println(iter->second->getKeyString()); flushYield();
                    #endif
                    if (lightingIter->second) { delete lightingIter->second; }
                    _lightings.erase(lightingIter);
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
            Helioduino::_activeInstance->notifyDayChanged();
        }
        if (getLoggerInstance()) {
            getLoggerInstance()->notifyDayChanged();
        }
        if (getPublisherInstance()) {
            getPublisherInstance()->notifyDayChanged();
        }
    #endif
}


HelioProcess::HelioProcess(SharedPtr<HelioFeedPanel> feedResIn)
    : feedRes(feedResIn), stageStart(0)
{ ; }

void HelioProcess::clearActuatorReqs()
{
    while (actuatorReqs.size()) {
        actuatorReqs.begin()->get()->disableActuator();
        actuatorReqs.erase(actuatorReqs.begin());
    }
}

void HelioProcess::setActuatorReqs(const Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> &actuatorReqsIn)
{
    for (auto actuatorIter = actuatorReqs.begin(); actuatorIter != actuatorReqs.end(); ++actuatorIter) {
        bool found = false;
        auto key = (*actuatorIter)->getKey();
    
        for (auto actuatorInIter = actuatorReqsIn.begin(); actuatorInIter != actuatorReqsIn.end(); ++actuatorInIter) {
            if (key == (*actuatorInIter)->getKey()) {
                found = true;
                break;
            }
        }
    
        if (!found && (*actuatorIter)->isEnabled()) { // disables actuators not found in new list
            (*actuatorIter)->disableActuator();
        }
    }

    {   actuatorReqs.clear();
        for (auto actuatorInIter = actuatorReqsIn.begin(); actuatorInIter != actuatorReqsIn.end(); ++actuatorInIter) {
            auto actuator = (*actuatorInIter);
            actuatorReqs.push_back(actuator);
        }
    }  
}


HelioFeeding::HelioFeeding(SharedPtr<HelioFeedPanel> feedRes)
    : HelioProcess(feedRes), stage(Unknown), canFeedAfter(0), lastAirReport(0),
      phSetpoint(0), tdsSetpoint(0), waterTempSetpoint(0), airTempSetpoint(0), co2Setpoint(0)
{
    reset();
}

HelioFeeding::~HelioFeeding()
{
    clearActuatorReqs();
}

void HelioFeeding::reset()
{
    clearActuatorReqs();
    stage = Init; stageStart = unixNow();
    recalcFeeding();
}

void HelioFeeding::recalcFeeding()
{
    float totalWeights = 0;
    float totalSetpoints[5] = {0,0,0,0,0};

    {   auto crops = linksFilterCrops(feedRes->getLinkages());
        for (auto cropIter = crops.begin(); cropIter != crops.end(); ++cropIter) {
            auto crop = (HelioCrop *)(*cropIter);
            auto cropsLibData = helioCropsLib.checkoutCropsData(crop->getCropType());

            if (cropsLibData) {
                float weight = crop->getFeedingWeight();
                totalWeights += weight;

                float feedRate = ((cropsLibData->tdsRange[0] + cropsLibData->tdsRange[1]) * 0.5);
                if (!getSchedulerInstance()->inDaytimeMode()) {
                    feedRate *= cropsLibData->nightlyFeedRate;
                }
                feedRate *= getSchedulerInstance()->getBaseFeedMultiplier();

                totalSetpoints[0] += feedRate * weight;
                totalSetpoints[1] += ((cropsLibData->phRange[0] + cropsLibData->phRange[1]) * 0.5) * weight;
                totalSetpoints[2] += ((cropsLibData->waterTempRange[0] + cropsLibData->waterTempRange[1]) * 0.5) * weight;
                totalSetpoints[3] += ((cropsLibData->airTempRange[0] + cropsLibData->airTempRange[1]) * 0.5) * weight;
                totalSetpoints[4] += cropsLibData->co2Levels[(crop->getCropPhase() <= Helio_CropPhase_Vegetative ? 0 : 1)] * weight;

                helioCropsLib.returnCropsData(cropsLibData);
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

    #ifdef HELIO_USE_VERBOSE_OUTPUT // only works for singular feed res in system, otherwise output will be erratic
    {   static float _totalSetpoints[5] = {0,0,0,0,0};
        if (!isFPEqual(_totalSetpoints[0], totalSetpoints[0]) || !isFPEqual(_totalSetpoints[1], totalSetpoints[1]) || !isFPEqual(_totalSetpoints[2], totalSetpoints[2]) || !isFPEqual(_totalSetpoints[3], totalSetpoints[3]) || !isFPEqual(_totalSetpoints[4], totalSetpoints[4])) {
            _totalSetpoints[0] = totalSetpoints[0]; _totalSetpoints[1] = totalSetpoints[1]; _totalSetpoints[2] = totalSetpoints[2]; _totalSetpoints[3] = totalSetpoints[3]; _totalSetpoints[4] = totalSetpoints[4];
            Serial.print(F("Feeding::recalcFeeding setpoints: {tds,pH,wTmp,aTmp,aCO2} = [")); Serial.print(_totalSetpoints[0]); Serial.print(' '); Serial.print(_totalSetpoints[1]); Serial.print(' '); Serial.print(_totalSetpoints[2]); Serial.print(' '); Serial.print(_totalSetpoints[3]); Serial.print(' '); Serial.print(_totalSetpoints[4]); Serial.println(']'); flushYield(); } }
    #endif

    setupStaging();
}

void HelioFeeding::setupStaging()
{
    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFS1 = (int8_t)-1; if (_stageFS1 != (int8_t)stage) {
        Serial.print(F("Feeding::setupStaging stage: ")); Serial.println((_stageFS1 = (int8_t)stage)); flushYield(); } }
    #endif

    if (stage == PreFeed) {
        if (feedRes->getWaterPHSensor()) {
            auto phBalancer = feedRes->getWaterPHBalancer();
            if (!phBalancer) {
                phBalancer = SharedPtr<HelioMotorDriver>(new HelioMotorDriver(feedRes->getWaterPHSensor(), phSetpoint, HELIO_RANGE_PH_HALF, feedRes->getMaxVolume(), feedRes->getVolumeUnits()));
                HELIO_SOFT_ASSERT(phBalancer, SFP(HStr_Err_AllocationFailure));
                getSchedulerInstance()->setupWaterPHBalancer(feedRes.get(), phBalancer);
                feedRes->setWaterPHBalancer(phBalancer);
            }
            if (phBalancer) {
                phBalancer->setTargetSetpoint(phSetpoint);
                phBalancer->setTargetUnits(Helio_UnitsType_Alkalinity_pH_0_14);
                phBalancer->setEnabled(true);
            }
        }
        if (feedRes->getWaterTDSSensor()) {
            auto tdsBalancer = feedRes->getWaterTDSBalancer();
            if (!tdsBalancer) {
                tdsBalancer = SharedPtr<HelioMotorDriver>(new HelioMotorDriver(feedRes->getWaterTDSSensor(), tdsSetpoint, HELIO_RANGE_EC_HALF, feedRes->getMaxVolume(), feedRes->getVolumeUnits()));
                HELIO_SOFT_ASSERT(tdsBalancer, SFP(HStr_Err_AllocationFailure));
                getSchedulerInstance()->setupWaterTDSBalancer(feedRes.get(), tdsBalancer);
                feedRes->setWaterTDSBalancer(tdsBalancer);
            }
            if (tdsBalancer) {
                tdsBalancer->setTargetSetpoint(tdsSetpoint);
                tdsBalancer->setTargetUnits(Helio_UnitsType_Concentration_EC);
                tdsBalancer->setEnabled(true);
            }
        }
    } else {
        auto phBalancer = feedRes->getWaterPHBalancer();
        if (phBalancer) { phBalancer->setEnabled(false); }
        auto tdsBalancer = feedRes->getWaterTDSBalancer();
        if (tdsBalancer) { tdsBalancer->setEnabled(false); }
    }

    if ((stage == PreFeed || stage == Feed) && feedRes->getWaterTemperatureSensor()) {
        auto waterTempBalancer = feedRes->getWaterTemperatureBalancer();
        if (!waterTempBalancer) {
            waterTempBalancer = SharedPtr<HelioServoDriver>(new HelioServoDriver(feedRes->getWaterTemperatureSensor(), waterTempSetpoint, HELIO_RANGE_TEMP_HALF, -HELIO_RANGE_TEMP_HALF * 0.25f, HELIO_RANGE_TEMP_HALF * 0.5f));
            HELIO_SOFT_ASSERT(waterTempBalancer, SFP(HStr_Err_AllocationFailure));
            getSchedulerInstance()->setupWaterTemperatureBalancer(feedRes.get(), waterTempBalancer);
            feedRes->setWaterTemperatureBalancer(waterTempBalancer);
        }
        if (waterTempBalancer) {
            waterTempBalancer->setTargetSetpoint(waterTempSetpoint);
            waterTempBalancer->setTargetUnits(Helio_UnitsType_Temperature_Celsius);
            waterTempBalancer->setEnabled(true);
        }
    } else {
        auto waterTempBalancer = feedRes->getWaterTemperatureBalancer();
        if (waterTempBalancer) { waterTempBalancer->setEnabled(false); }
    }

    if (feedRes->getAirTemperatureSensor()) {
        auto airTempBalancer = feedRes->getAirTemperatureBalancer();
        if (!airTempBalancer) {
            airTempBalancer = SharedPtr<HelioServoDriver>(new HelioServoDriver(feedRes->getAirTemperatureSensor(), airTempSetpoint, HELIO_RANGE_TEMP_HALF, -HELIO_RANGE_TEMP_HALF * 0.25f, HELIO_RANGE_TEMP_HALF * 0.5f));
            HELIO_SOFT_ASSERT(airTempBalancer, SFP(HStr_Err_AllocationFailure));
            getSchedulerInstance()->setupAirTemperatureBalancer(feedRes.get(), airTempBalancer);
            feedRes->setAirTemperatureBalancer(airTempBalancer);
        }
        if (airTempBalancer) {
            airTempBalancer->setTargetSetpoint(airTempSetpoint);
            airTempBalancer->setTargetUnits(Helio_UnitsType_Temperature_Celsius);
            airTempBalancer->setEnabled(true);
        }
    } else {
        auto airTempBalancer = feedRes->getAirTemperatureBalancer();
        if (airTempBalancer) { airTempBalancer->setEnabled(false); }
    }

    if (feedRes->getAirCO2Sensor()) {
        auto co2Balancer = feedRes->getAirTemperatureBalancer();
        if (!co2Balancer) {
            co2Balancer = SharedPtr<HelioServoDriver>(new HelioServoDriver(feedRes->getAirCO2Sensor(), co2Setpoint, HELIO_RANGE_CO2_HALF, -HELIO_RANGE_CO2_HALF * 0.25f, HELIO_RANGE_CO2_HALF * 0.5f));
            HELIO_SOFT_ASSERT(co2Balancer, SFP(HStr_Err_AllocationFailure));
            getSchedulerInstance()->setupAirCO2Balancer(feedRes.get(), co2Balancer);
            feedRes->setAirCO2Balancer(co2Balancer);
        }
        if (co2Balancer) {
            co2Balancer->setTargetSetpoint(co2Setpoint);
            co2Balancer->setTargetUnits(Helio_UnitsType_Concentration_PPM);
            co2Balancer->setEnabled(true);
        }
    } else {
        auto co2Balancer = feedRes->getAirCO2Balancer();
        if (co2Balancer) { co2Balancer->setEnabled(false); }
    }

    switch (stage) {
        case Init: {
            auto maxFeedingsDay = getSchedulerInstance()->getTotalFeedingsDay();
            auto feedingsToday = feedRes->getFeedingsToday();

            if (!maxFeedingsDay) {
                canFeedAfter = (time_t)0;
            } else if (feedingsToday < maxFeedingsDay) {
                // this will force feedings to be spread out during the entire day
                canFeedAfter = getCurrentDayStartTime() + (time_t)(((float)SECS_PER_DAY / (maxFeedingsDay + 1)) * feedingsToday);
            } else {
                canFeedAfter = (time_t)UINT32_MAX; // no more feedings today
            }

            if (canFeedAfter > unixNow()) { clearActuatorReqs(); } // clear on wait
        } break;

        case TopOff: {
            if (!feedRes->isFilled()) {
                Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;
                auto topOffMotors = linksFilterMotorActuatorsByOutputPanelAndInputPanelType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedRes->getLinkages(), feedRes.get(), Helio_PanelType_FreshWater);

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

        case PreFeed: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;
            auto aerators = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedRes->getLinkages(), feedRes.get(), Helio_ActuatorType_WaterAerator);

            linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(aerators, newActuatorReqs, Helio_ActuatorType_WaterAerator);

            setActuatorReqs(newActuatorReqs);
        } break;

        case Feed: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;

            {   auto feedMotors = linksFilterMotorActuatorsByInputPanelAndOutputPanelType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedRes->getLinkages(), feedRes.get(), Helio_PanelType_FeedWater);

                linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedMotors, newActuatorReqs, Helio_ActuatorType_WaterMotor); // feed water motor
            }

            if (!newActuatorReqs.size() && getHelioInstance()->getSystemMode() == Helio_SystemMode_DrainToWaste) { // prefers feed water motors, else direct to waste is feed
                auto feedMotors = linksFilterMotorActuatorsByInputPanelAndOutputPanelType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedRes->getLinkages(), feedRes.get(), Helio_PanelType_DrainageWater);

                linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedMotors, newActuatorReqs, Helio_ActuatorType_WaterMotor); // DTW feed water motor
            }

            HELIO_SOFT_ASSERT(newActuatorReqs.size(), SFP(HStr_Err_MissingLinkage)); // no feed water motors

            #if HELIO_SCH_AERATORS_FEEDRUN
                {   auto aerators = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedRes->getLinkages(), feedRes.get(), Helio_ActuatorType_WaterAerator);

                    linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(aerators, newActuatorReqs, Helio_ActuatorType_WaterAerator);
                }
            #endif

            setActuatorReqs(newActuatorReqs);
        } break;

        case Drain: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;
            auto drainMotors = linksFilterMotorActuatorsByInputPanelAndOutputPanelType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedRes->getLinkages(), feedRes.get(), Helio_PanelType_DrainageWater);

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
        Serial.print(F("Feeding::~setupStaging stage: ")); Serial.println((_stageFS2 = (int8_t)stage)); flushYield(); } }
    #endif
}

void HelioFeeding::update()
{
    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFU1 = (int8_t)-1; if (_stageFU1 != (int8_t)stage) {
        Serial.print(F("Feeding::update stage: ")); Serial.println((_stageFU1 = (int8_t)stage)); flushYield(); } }
    #endif

    if ((!lastAirReport || unixNow() >= lastAirReport + getSchedulerInstance()->getAirReportInterval().totalseconds()) &&
        (getSchedulerInstance()->getAirReportInterval().totalseconds() > 0) && // 0 disables
        (feedRes->getAirTemperatureSensor() || feedRes->getAirCO2Sensor())) {
        getLoggerInstance()->logProcess(feedRes.get(), SFP(HStr_Log_AirReport));
        logFeeding(HelioFeedingLogType_AirSetpoints);
        logFeeding(HelioFeedingLogType_AirMeasures);
        lastAirReport = unixNow();
    }

    switch (stage) {
        case Init: {
            if (!canFeedAfter || unixNow() >= canFeedAfter) {
                int cropsCount = 0;
                int cropsHungry = 0;

                {   auto crops = linksFilterCrops(feedRes->getLinkages());
                    cropsCount = crops.size();
                    for (auto cropIter = crops.begin(); cropIter != crops.end(); ++cropIter) {
                        if (((HelioCrop *)(*cropIter))->needsFeeding()) { cropsHungry++; }
                    }
                }

                if (!cropsCount || cropsHungry / (float)cropsCount >= HELIO_SCH_FEED_FRACTION - FLT_EPSILON) {
                    stage = TopOff; stageStart = unixNow();
                    setupStaging();

                    if (actuatorReqs.size()) {
                        getLoggerInstance()->logProcess(feedRes.get(), SFP(HStr_Log_PreFeedTopOff), SFP(HStr_Log_HasBegan));
                    }
                }
            }
        } break;

        case TopOff: {
            if (feedRes->isFilled() || !actuatorReqs.size()) {
                stage = PreFeed; stageStart = unixNow();
                canFeedAfter = 0; // will be used to track how long balancers stay balanced
                setupStaging();

                getLoggerInstance()->logProcess(feedRes.get(), SFP(HStr_Log_PreFeedDriving), SFP(HStr_Log_HasBegan));
                if (actuatorReqs.size()) {
                    getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Aerator_Duration), String(getSchedulerInstance()->getPreFeedAeratorMins()), String('m'));
                }
                if (feedRes->getWaterPHBalancer() || feedRes->getWaterTDSBalancer()) {
                    auto balancer = static_pointer_cast<HelioMotorDriver>(feedRes->getWaterPHBalancer() ? feedRes->getWaterPHBalancer() : feedRes->getWaterTDSBalancer());
                    if (balancer) {
                        getLoggerInstance()->logMessage(SFP(HStr_Log_Field_MixTime_Duration), timeSpanToString(TimeSpan(balancer->getMixTime())));
                    }
                }
                logFeeding(HelioFeedingLogType_WaterSetpoints);
                logFeeding(HelioFeedingLogType_WaterMeasures);
            }
        } break;

        case PreFeed: {
            if (!actuatorReqs.size() || unixNow() >= stageStart + (getSchedulerInstance()->getPreFeedAeratorMins() * SECS_PER_MIN)) {
                auto phBalancer = feedRes->getWaterPHBalancer();
                auto tdsBalancer = feedRes->getWaterTDSBalancer();
                auto waterTempBalancer = feedRes->getWaterTemperatureBalancer();

                if ((!phBalancer || (phBalancer->isEnabled() && phBalancer->isOnTarget())) &&
                    (!tdsBalancer || (tdsBalancer->isEnabled() && tdsBalancer->isOnTarget())) &&
                    (!waterTempBalancer || (waterTempBalancer->isEnabled() && waterTempBalancer->isOnTarget()))) {
                    // Can proceed after above are marked balanced for min time
                    if (!canFeedAfter) { canFeedAfter = unixNow() + HELIO_SCH_BALANCE_MINTIME; }
                    else if (unixNow() >= canFeedAfter) {
                        stage = Feed; stageStart = unixNow();
                        setupStaging();

                        broadcastFeeding(HelioFeedingBroadcastType_Began);
                    }
                } else {
                    canFeedAfter = 0;
                }
            }
        } break;

        case Feed: {
            int cropsCount = 0;
            int cropsFed = 0;

            {   auto crops = linksFilterCrops(feedRes->getLinkages());
                cropsCount = crops.size();
                for (auto cropIter = crops.begin(); cropIter != crops.end(); ++cropIter) {
                    if (!((HelioCrop *)(*cropIter))->needsFeeding()) { cropsFed++; }
                }
            }

            if (!cropsCount || cropsFed / (float)cropsCount >= HELIO_SCH_FEED_FRACTION - FLT_EPSILON ||
                feedRes->isEmpty()) {
                stage = (getHelioInstance()->getSystemMode() == Helio_SystemMode_DrainToWaste ? Drain : Done);
                stageStart = unixNow();
                setupStaging();

                broadcastFeeding(HelioFeedingBroadcastType_Ended);
            }
        } break;

        case Drain: {
            if (getHelioInstance()->getSystemMode() != Helio_SystemMode_DrainToWaste ||
                feedRes->isEmpty()) {
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
        for (auto actuatorIter = actuatorReqs.begin(); actuatorIter != actuatorReqs.end(); ++actuatorIter) {
            auto actuator = (*actuatorIter);
            if (!actuator->isEnabled() && !actuator->enableActuator()) {
                // TODO: Something clever to track stalled actuators
            }
        }
    }

    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageFU2 = (int8_t)-1; if (_stageFU2 != (int8_t)stage) {
        Serial.print(F("Feeding::~update stage: ")); Serial.println((_stageFU2 = (int8_t)stage)); flushYield(); } }
    #endif
}

void HelioFeeding::logFeeding(HelioFeedingLogType logType)
{
    switch (logType) {
        case HelioFeedingLogType_WaterSetpoints:
            {   auto ph = HelioSingleMeasurement(phSetpoint, Helio_UnitsType_Alkalinity_pH_0_14);
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_pH_Setpoint), measurementToString(ph));
            }
            {   auto tds = HelioSingleMeasurement(tdsSetpoint, Helio_UnitsType_Concentration_TDS);
                convertUnits(&tds, feedRes->getTDSUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_TDS_Setpoint), measurementToString(tds, 1));
            }
            {   auto temp = HelioSingleMeasurement(waterTempSetpoint, Helio_UnitsType_Temperature_Celsius);
                convertUnits(&temp, feedRes->getTemperatureUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Temp_Setpoint), measurementToString(temp));
            }
            break;

        case HelioFeedingLogType_WaterMeasures:
            #ifdef HELIO_USE_MULTITASKING
                // Yield will allow measurements to complete, ensures first log out doesn't contain zero'ed values
                if ((feedRes->getWaterPHSensor() && !feedRes->getWaterPH().getMeasurementFrame()) ||
                    (feedRes->getWaterTDSSensor() && !feedRes->getWaterTDS().getMeasurementFrame()) ||
                    (feedRes->getWaterTemperatureSensor() && !feedRes->getWaterTemperature().getMeasurementFrame())) {
                    yield();
                }
            #endif
             if (feedRes->getWaterPHSensor()) {
                auto ph = feedRes->getWaterPH().getMeasurement(true);
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_pH_Measured), measurementToString(ph));
            }
            if (feedRes->getWaterTDSSensor()) {
                auto tds = feedRes->getWaterTDS().getMeasurement(true);
                convertUnits(&tds, feedRes->getTDSUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_TDS_Measured), measurementToString(tds, 1));
            }
            if (feedRes->getWaterTemperatureSensor()) {
                auto temp = feedRes->getWaterTemperature().getMeasurement(true);
                convertUnits(&temp, feedRes->getTemperatureUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Temp_Measured), measurementToString(temp));
            }
            break;

        case HelioFeedingLogType_AirSetpoints:
            {   auto temp = HelioSingleMeasurement(airTempSetpoint, Helio_UnitsType_Temperature_Celsius);
                convertUnits(&temp, feedRes->getTemperatureUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Temp_Setpoint), measurementToString(temp));
            }
            {   auto co2 = HelioSingleMeasurement(co2Setpoint, Helio_UnitsType_Concentration_PPM);
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_CO2_Setpoint), measurementToString(co2));
            }
            break;

        case HelioFeedingLogType_AirMeasures:
            #ifdef HELIO_USE_MULTITASKING
                // Yield will allow measurements to complete, ensures first log out doesn't contain zero'ed values
                if ((feedRes->getAirTemperatureSensor() && !feedRes->getAirTemperature().getMeasurementFrame()) ||
                    (feedRes->getAirCO2Sensor() && !feedRes->getAirCO2().getMeasurementFrame())) {
                    yield();
                }
            #endif
            if (feedRes->getAirTemperatureSensor()) {
                auto temp = feedRes->getAirTemperature().getMeasurement(true);
                convertUnits(&temp, feedRes->getTemperatureUnits());
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Temp_Measured), measurementToString(temp));
            }
            if (feedRes->getAirCO2Sensor()) {
                auto co2 = feedRes->getAirCO2().getMeasurement(true);
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_CO2_Measured), measurementToString(co2));
            }
            break;
    }
}

void HelioFeeding::broadcastFeeding(HelioFeedingBroadcastType broadcastType)
{
    getLoggerInstance()->logProcess(feedRes.get(), SFP(HStr_Log_FeedingSequence),
                                    SFP(broadcastType == HelioFeedingBroadcastType_Began ? HStr_Log_HasBegan : HStr_Log_HasEnded));
    logFeeding(HelioFeedingLogType_WaterMeasures);

    broadcastType == HelioFeedingBroadcastType_Began ? feedRes->notifyFeedingBegan() : feedRes->notifyFeedingEnded();

    {   auto crops = linksFilterCrops(feedRes->getLinkages());
        for (auto cropIter = crops.begin(); cropIter != crops.end(); ++cropIter) {
            broadcastType == HelioFeedingBroadcastType_Began ? ((HelioCrop *)(*cropIter))->notifyFeedingBegan()
                                                                   : ((HelioCrop *)(*cropIter))->notifyFeedingEnded();
        }
    }
}


HelioLighting::HelioLighting(SharedPtr<HelioFeedPanel> feedRes)
    : HelioProcess(feedRes), stage(Init), sprayStart(0), lightStart(0), lightEnd(0), lightHours(0.0f)
{
    stageStart = unixNow();
    recalcLighting();
}

HelioLighting::~HelioLighting()
{
    clearActuatorReqs();
}

void HelioLighting::recalcLighting()
{
    float totalWeights = 0;
    float totalLightHours = 0;
    bool sprayingNeeded = false;

    {   auto crops = linksFilterCrops(feedRes->getLinkages());

        for (auto cropIter = crops.begin(); cropIter != crops.end(); ++cropIter) {
            auto crop = (HelioCrop *)(*cropIter);
            auto cropPhase = (Helio_CropPhase)constrain((int)(crop->getCropPhase()), 0, (int)Helio_CropPhase_MainCount - 1);

            if ((int)cropPhase >= 0) {
                auto cropsLibData = helioCropsLib.checkoutCropsData(crop->getCropType());

                if (cropsLibData) {
                    auto weight = crop->getFeedingWeight();
                    totalWeights += weight;
                    totalLightHours += (cropsLibData->dailyLightHours[cropPhase] * weight);
                    sprayingNeeded = sprayingNeeded || cropsLibData->needsSpraying();

                    helioCropsLib.returnCropsData(cropsLibData);
                }
            }
        }
    }

    if (totalWeights < FLT_EPSILON) {
        totalWeights = 1.0f;
        totalLightHours = 12.0f;
    }

    {   lightHours = (totalLightHours / totalWeights);
        lightHours = constrain(lightHours, 0.0f, 24.0f);
        time_t dayLightSecs = lightHours * SECS_PER_HOUR;

        time_t daySprayerSecs = 0;
        if (sprayingNeeded && linksCountActuatorsByPanelAndType(feedRes->getLinkages(), feedRes.get(), Helio_ActuatorType_PanelCleaner)) {
            daySprayerSecs = getSchedulerInstance()->getPreLightSprayMins() * SECS_PER_MIN;
        }

        time_t dayStart = getCurrentDayStartTime();
        lightStart = dayStart + ((SECS_PER_DAY - dayLightSecs) >> 1);
        sprayStart = max(dayStart, lightStart - daySprayerSecs);
        lightStart = sprayStart + daySprayerSecs;
        lightEnd = lightStart + dayLightSecs;

        #ifdef HELIO_USE_VERBOSE_OUTPUT // only works for singular feed res in system, otherwise output will be erratic
        {   static float _totalLightHours = 0;
            if (!isFPEqual(_totalLightHours, lightHours)) {
                _totalLightHours = lightHours;
                Serial.print(F("Lighting::recalcLighting lightHours: ")); Serial.println(_totalLightHours); flushYield(); } }
        #endif
    }

    setupStaging();
}

void HelioLighting::setupStaging()
{
    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageLS1 = (int8_t)-1; if (_stageLS1 != (int8_t)stage) {
        Serial.print(F("Lighting::setupStaging stage: ")); Serial.println((_stageLS1 = (int8_t)stage)); flushYield(); } }
    #endif

    switch (stage) {
        case Init: {
            clearActuatorReqs();
        } break;

        case Spray: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;
            auto sprayers = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedRes->getLinkages(), feedRes.get(), Helio_ActuatorType_PanelCleaner);

            linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(sprayers, newActuatorReqs, Helio_ActuatorType_PanelCleaner);

            setActuatorReqs(newActuatorReqs);
        } break;

        case Light: {
            Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> newActuatorReqs;
            auto lights = linksFilterActuatorsByPanelAndType<HELIO_SCH_REQACTUATORS_MAXSIZE>(feedRes->getLinkages(), feedRes.get(), Helio_ActuatorType_GrowLights);

            linksResolveActuatorsByType<HELIO_SCH_REQACTUATORS_MAXSIZE>(lights, newActuatorReqs, Helio_ActuatorType_GrowLights);

            setActuatorReqs(newActuatorReqs);
        } break;

        case Done: {
            clearActuatorReqs();
        } break;

        default:
            break;
    }

    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageLS2 = (int8_t)-1; if (_stageLS2 != (int8_t)stage) {
        Serial.print(F("Lighting::~setupStaging stage: ")); Serial.println((_stageLS2 = (int8_t)stage)); flushYield(); } }
    #endif
}

void HelioLighting::update()
{
    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageLU1 = (int8_t)-1; if (_stageLU1 != (int8_t)stage) {
        Serial.print(F("Lighting::update stage: ")); Serial.println((_stageLU1 = (int8_t)stage)); flushYield(); } }
    #endif

    switch (stage) {
        case Init: {
            time_t currTime = getCurrentTime().unixtime();

            if (currTime >= sprayStart && currTime < lightEnd) {
                stage = Spray; stageStart = unixNow();
                setupStaging();

                if (lightStart > sprayStart) {
                    getLoggerInstance()->logProcess(feedRes.get(), SFP(HStr_Log_PreLightSpraying), SFP(HStr_Log_HasBegan));
                    getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Sprayer_Duration), String(getSchedulerInstance()->getPreLightSprayMins()), String('m'));
                    getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Time_Start), DateTime((uint32_t)sprayStart).timestamp(DateTime::TIMESTAMP_TIME));
                    getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Time_Finish), DateTime((uint32_t)lightStart).timestamp(DateTime::TIMESTAMP_TIME));
                }
            }
        } break;

        case Spray: {
            if (getCurrentTime().unixtime() >= lightStart) {
                stage = Light; stageStart = unixNow();
                setupStaging();

                getLoggerInstance()->logProcess(feedRes.get(), SFP(HStr_Log_LightingSequence), SFP(HStr_Log_HasBegan));
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Light_Duration), roundToString(lightHours), String('h'));
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Time_Start), DateTime((uint32_t)lightStart).timestamp(DateTime::TIMESTAMP_TIME));
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Time_Finish), DateTime((uint32_t)lightEnd).timestamp(DateTime::TIMESTAMP_TIME));
            } else {
                stage = Done; stageStart = unixNow();
                setupStaging();
            }
        } break;

        case Light: {
            if (getCurrentTime().unixtime() >= lightEnd) {
                TimeSpan elapsedTime(unixNow() - stageStart);
                stage = Done; stageStart = unixNow();
                setupStaging();

                getLoggerInstance()->logProcess(feedRes.get(), SFP(HStr_Log_LightingSequence), SFP(HStr_Log_HasEnded));
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Time_Measured), timeSpanToString(elapsedTime));
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
        for (auto actuatorIter = actuatorReqs.begin(); actuatorIter != actuatorReqs.end(); ++actuatorIter) {
            auto actuator = (*actuatorIter);
            if (!actuator->isEnabled() && !actuator->enableActuator()) {
                // TODO: Something clever to track stalled actuators
            }
        }
    }

    #ifdef HELIO_USE_VERBOSE_OUTPUT
    {   static int8_t _stageLU2 = (int8_t)-1; if (_stageLU2 != (int8_t)stage) {
        Serial.print(F("Lighting::~update stage: ")); Serial.println((_stageLU2 = (int8_t)stage)); flushYield(); } }
    #endif
}


HelioSchedulerSubData::HelioSchedulerSubData()
    : HelioSubData(0), baseFeedMultiplier(1), weeklyDosingRates{1}, stdDosingRates{1,0.5,0.5},
      totalFeedingsDay(0), preFeedAeratorMins(30), preLightSprayMins(60), airReportInterval(8 * SECS_PER_HOUR)
{ ; }

void HelioSchedulerSubData::toJSONObject(JsonObject &objectOut) const
{
    //HelioSubData::toJSONObject(objectOut); // purposeful no call to base method (ignores type)

    if (!isFPEqual(baseFeedMultiplier, 1.0f)) { objectOut[SFP(HStr_Key_BaseFeedMultiplier)] = baseFeedMultiplier; }
    bool hasWeeklyDosings = arrayElementsEqual(weeklyDosingRates, HELIO_CROP_GROWWEEKS_MAX, 1.0f);
    if (hasWeeklyDosings) { objectOut[SFP(HStr_Key_WeeklyDosingRates)] = commaStringFromArray(weeklyDosingRates, HELIO_CROP_GROWWEEKS_MAX); }
    bool hasStandardDosings = !isFPEqual(stdDosingRates[0], 1.0f) || !isFPEqual(stdDosingRates[1], 0.5f) || !isFPEqual(stdDosingRates[2], 0.5f);
    if (hasStandardDosings) { objectOut[SFP(HStr_Key_StdDosingRates)] = commaStringFromArray(stdDosingRates, 3); }
    if (totalFeedingsDay > 0) { objectOut[SFP(HStr_Key_TotalFeedingsDay)] = totalFeedingsDay; }
    if (preFeedAeratorMins != 30) { objectOut[SFP(HStr_Key_PreFeedAeratorMins)] = preFeedAeratorMins; }
    if (preLightSprayMins != 60) { objectOut[SFP(HStr_Key_PreLightSprayMins)] = preLightSprayMins; }
    if (airReportInterval != (8 * SECS_PER_HOUR)) { objectOut[SFP(HStr_Key_AirReportInterval)] = airReportInterval; }
}

void HelioSchedulerSubData::fromJSONObject(JsonObjectConst &objectIn)
{
    //HelioSubData::fromJSONObject(objectIn); // purposeful no call to base method (ignores type)

    baseFeedMultiplier = objectIn[SFP(HStr_Key_BaseFeedMultiplier)] | baseFeedMultiplier;
    JsonVariantConst weeklyDosingRatesVar = objectIn[SFP(HStr_Key_WeeklyDosingRates)];
    commaStringToArray(weeklyDosingRatesVar, weeklyDosingRates, HELIO_CROP_GROWWEEKS_MAX);
    JsonVariantConst stdDosingRatesVar = objectIn[SFP(HStr_Key_StdDosingRates)];
    commaStringToArray(stdDosingRatesVar, stdDosingRates, 3);
    totalFeedingsDay = objectIn[SFP(HStr_Key_TotalFeedingsDay)] | totalFeedingsDay;
    preFeedAeratorMins = objectIn[SFP(HStr_Key_PreFeedAeratorMins)] | preFeedAeratorMins;
    preLightSprayMins = objectIn[SFP(HStr_Key_PreLightSprayMins)] | preLightSprayMins;
    airReportInterval = objectIn[SFP(HStr_Key_AirReportInterval)] | airReportInterval;
}

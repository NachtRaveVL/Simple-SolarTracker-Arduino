/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Factory
*/

#include "Helioduino.h"

SharedPtr<HelioRelayActuator> HelioFactory::addPanelHeaterRelay(pintype_t outputPin)
{
    bool outputPinIsDigital = checkPinIsDigital(outputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_ActuatorType_PanelHeater));
    HELIO_HARD_ASSERT(outputPinIsDigital, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (outputPinIsDigital && isValidIndex(positionIndex)) {
        auto actuator = SharedPtr<HelioRelayActuator>(new HelioRelayActuator(
            Helio_ActuatorType_PanelHeater,
            positionIndex,
            HelioDigitalPin(outputPin, OUTPUT)
        ));
        if (getController()->registerObject(actuator)) { return actuator; }
    }

    return nullptr;
}

SharedPtr<HelioAnalogSensor> HelioFactory::addAnalogTemperatureSensor(pintype_t inputPin, uint8_t inputBitRes)
{
    bool inputPinIsAnalog = checkPinIsAnalogInput(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_TemperatureHumidity));
    HELIO_HARD_ASSERT(inputPinIsAnalog, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsAnalog && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioAnalogSensor>(new HelioAnalogSensor(
            Helio_SensorType_TemperatureHumidity,
            positionIndex,
            HelioAnalogPin(inputPin, INPUT, inputBitRes)
        ));
        if (getController()->registerObject(sensor)) { return sensor; }
    }

    return nullptr;
}

SharedPtr<HelioAnalogSensor> HelioFactory::addPowerUsageLevelMeter(pintype_t inputPin, bool isWattageBased, uint8_t inputBitRes)
{
    bool inputPinIsAnalog = checkPinIsAnalogInput(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_PowerProduction));
    HELIO_HARD_ASSERT(inputPinIsAnalog, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsAnalog && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioAnalogSensor>(new HelioAnalogSensor(
            Helio_SensorType_PowerProduction,
            positionIndex,
            HelioAnalogPin(inputPin, INPUT, inputBitRes)
        ));
        if (getController()->registerObject(sensor)) {
            if (!isWattageBased) { sensor->setMeasurementUnits(Helio_UnitsType_Power_Amperage); }
            return sensor;
        }
    }

    return nullptr;
}

SharedPtr<HelioSimpleRail> HelioFactory::addSimplePowerRail(Helio_RailType railType, int maxActiveAtOnce)
{
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(railType));
    HELIO_SOFT_ASSERT((int)railType >= 0 && railType <= Helio_RailType_Count, SFP(HStr_Err_InvalidParameter));
    HELIO_SOFT_ASSERT(maxActiveAtOnce > 0, SFP(HStr_Err_InvalidParameter));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if ((int)railType >= 0 && railType < Helio_RailType_Count && maxActiveAtOnce > 0 && isValidIndex(positionIndex)) {
        auto rail = SharedPtr<HelioSimpleRail>(new HelioSimpleRail(
            railType,
            positionIndex,
            maxActiveAtOnce
        ));
        if (getController()->registerObject(rail)) { return rail; }
    }

    return nullptr;
}

SharedPtr<HelioRegulatedRail> HelioFactory::addRegulatedPowerRail(Helio_RailType railType, float maxPower)
{
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(railType));
    HELIO_SOFT_ASSERT((int)railType >= 0 && railType <= Helio_RailType_Count, SFP(HStr_Err_InvalidParameter));
    HELIO_SOFT_ASSERT(maxPower > FLT_EPSILON, SFP(HStr_Err_InvalidParameter));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if ((int)railType >= 0 && railType < Helio_RailType_Count && maxPower > FLT_EPSILON && isValidIndex(positionIndex)) {
        auto rail = SharedPtr<HelioRegulatedRail>(new HelioRegulatedRail(
            railType,
            positionIndex,
            maxPower
        ));
        if (getController()->registerObject(rail)) { return rail; }
    }

    return nullptr;
}

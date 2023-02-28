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

SharedPtr<HelioRelayActuator> HelioFactory::addPanelSprayerRelay(pintype_t outputPin)
{
    bool outputPinIsDigital = checkPinIsDigital(outputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_ActuatorType_PanelSprayer));
    HELIO_HARD_ASSERT(outputPinIsDigital, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (outputPinIsDigital && isValidIndex(positionIndex)) {
        auto actuator = SharedPtr<HelioRelayActuator>(new HelioRelayActuator(
            Helio_ActuatorType_PanelSprayer,
            positionIndex,
            HelioDigitalPin(outputPin, OUTPUT)
        ));
        if (getController()->registerObject(actuator)) { return actuator; }
    }

    return nullptr;
}

SharedPtr<HelioRelayActuator> HelioFactory::addPanelBrakeRelay(pintype_t outputPin)
{
    bool outputPinIsDigital = checkPinIsDigital(outputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_ActuatorType_PanelBrake));
    HELIO_HARD_ASSERT(outputPinIsDigital, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (outputPinIsDigital && isValidIndex(positionIndex)) {
        auto actuator = SharedPtr<HelioRelayActuator>(new HelioRelayActuator(
            Helio_ActuatorType_PanelBrake,
            positionIndex,
            HelioDigitalPin(outputPin, OUTPUT)
        ));
        if (getController()->registerObject(actuator)) { return actuator; }
    }

    return nullptr;
}

SharedPtr<HelioVariableActuator> HelioFactory::addPositionalServo(pintype_t outputPin, float minDegrees, float maxDegrees, uint8_t outputBitRes
#ifdef ESP32
                                                      , uint8_t pwmChannel
#endif
#ifdef ESP_PLATFORM
                                                      , float pwmFrequency
#endif
)
{
    bool outputPinIsPWM = checkPinIsPWMOutput(outputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_ActuatorType_PositionalServo));
    HELIO_HARD_ASSERT(outputPinIsPWM, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (outputPinIsPWM && isValidIndex(positionIndex)) {
        auto actuator = SharedPtr<HelioVariableActuator>(new HelioVariableActuator(
            Helio_ActuatorType_PositionalServo,
            positionIndex,
            HelioAnalogPin(outputPin, OUTPUT, outputBitRes
#ifdef ESP32
                           , pwmChannel
#endif
#ifdef ESP_PLATFORM
                           , pwmFrequency
#endif
        )));
        if (getController()->registerObject(actuator)) {
            HelioCalibrationData userCalibData(actuator->getId(), Helio_UnitsType_Angle_Degrees_360);
            userCalibData.setFromServo(minDegrees, maxDegrees);
            actuator->setUserCalibrationData(&userCalibData);

            return actuator;
        }
    }

    return nullptr;
}

SharedPtr<HelioRelayMotorActuator> HelioFactory::addLinearActuatorRelay(pintype_t outputPinA, pintype_t outputPinB, float maxPosition, float minPosition)
{
    bool outputPinAIsDigital = checkPinIsDigital(outputPinA);
    bool outputPinBIsDigital = checkPinIsDigital(outputPinB);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_ActuatorType_LinearActuator));
    HELIO_HARD_ASSERT(outputPinAIsDigital && outputPinBIsDigital, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (outputPinAIsDigital && outputPinBIsDigital && isValidIndex(positionIndex)) {
        auto actuator = SharedPtr<HelioRelayMotorActuator>(new HelioRelayMotorActuator(
            Helio_ActuatorType_LinearActuator,
            positionIndex,
            HelioDigitalPin(outputPinA, OUTPUT),
            HelioDigitalPin(outputPinB, OUTPUT),
            make_pair(minPosition, maxPosition)
        ));
        if (getController()->registerObject(actuator)) { return actuator; }
    }

    return nullptr;
}

SharedPtr<HelioBinarySensor> HelioFactory::addEndstopIndicator(pintype_t inputPin)
{
    bool inputPinIsDigital = checkPinIsDigital(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_TravelPosition));
    HELIO_HARD_ASSERT(inputPinIsDigital, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsDigital && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioBinarySensor>(new HelioBinarySensor(
            Helio_SensorType_TravelPosition,
            positionIndex,
            HelioDigitalPin(inputPin, INPUT_PULLUP)
        ));
        if (getController()->registerObject(sensor)) { return sensor; }
    }

    return nullptr;
}

SharedPtr<HelioBinarySensor> HelioFactory::addIceIndicator(pintype_t inputPin)
{
    bool inputPinIsDigital = checkPinIsDigital(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_IceDetector));
    HELIO_HARD_ASSERT(inputPinIsDigital, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsDigital && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioBinarySensor>(new HelioBinarySensor(
            Helio_SensorType_IceDetector,
            positionIndex,
            HelioDigitalPin(inputPin, INPUT_PULLUP)
        ));
        if (getController()->registerObject(sensor)) { return sensor; }
    }

    return nullptr;
}

SharedPtr<HelioAnalogSensor> HelioFactory::addLightIntensitySensor(pintype_t inputPin, uint8_t inputBitRes)
{
    bool inputPinIsAnalog = checkPinIsAnalogInput(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_LightIntensity));
    HELIO_HARD_ASSERT(inputPinIsAnalog, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsAnalog && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioAnalogSensor>(new HelioAnalogSensor(
            Helio_SensorType_LightIntensity,
            positionIndex,
            HelioAnalogPin(inputPin, INPUT, inputBitRes)
        ));
        if (getController()->registerObject(sensor)) { return sensor; }
    }

    return nullptr;
}

SharedPtr<HelioAnalogSensor> HelioFactory::addPowerProductionMeter(pintype_t inputPin, bool isWattageBased, uint8_t inputBitRes)
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

SharedPtr<HelioAnalogSensor> HelioFactory::addPowerUsageLevelMeter(pintype_t inputPin, bool isWattageBased, uint8_t inputBitRes)
{
    bool inputPinIsAnalog = checkPinIsAnalogInput(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_PowerUsage));
    HELIO_HARD_ASSERT(inputPinIsAnalog, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsAnalog && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioAnalogSensor>(new HelioAnalogSensor(
            Helio_SensorType_PowerUsage,
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

SharedPtr<HelioAnalogSensor> HelioFactory::addAnalogPositionSensor(pintype_t inputPin, uint8_t inputBitRes)
{
    bool inputPinIsAnalog = checkPinIsAnalogInput(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_TravelPosition));
    HELIO_HARD_ASSERT(inputPinIsAnalog, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsAnalog && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioAnalogSensor>(new HelioAnalogSensor(
            Helio_SensorType_TravelPosition,
            positionIndex,
            HelioAnalogPin(inputPin, INPUT, inputBitRes)
        ));
        if (getController()->registerObject(sensor)) { return sensor; }
    }

    return nullptr;
}

SharedPtr<HelioAnalogSensor> HelioFactory::addAnalogTiltAngleSensor(pintype_t inputPin, uint8_t inputBitRes)
{
    bool inputPinIsAnalog = checkPinIsAnalogInput(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_TiltAngle));
    HELIO_HARD_ASSERT(inputPinIsAnalog, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsAnalog && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioAnalogSensor>(new HelioAnalogSensor(
            Helio_SensorType_TiltAngle,
            positionIndex,
            HelioAnalogPin(inputPin, INPUT, inputBitRes)
        ));
        if (getController()->registerObject(sensor)) { return sensor; }
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

SharedPtr<HelioAnalogSensor> HelioFactory::addAnalogWindSpeedSensor(pintype_t inputPin, uint8_t inputBitRes)
{
    bool inputPinIsAnalog = checkPinIsAnalogInput(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_WindSpeed));
    HELIO_HARD_ASSERT(inputPinIsAnalog, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsAnalog && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioAnalogSensor>(new HelioAnalogSensor(
            Helio_SensorType_WindSpeed,
            positionIndex,
            HelioAnalogPin(inputPin, INPUT, inputBitRes)
        ));
        if (getController()->registerObject(sensor)) { return sensor; }
    }

    return nullptr;
}

SharedPtr<HelioDHTTempHumiditySensor> HelioFactory::addDHTTempHumiditySensor(pintype_t inputPin, Helio_DHTType dhtType)
{
    bool inputPinIsDigital = checkPinIsDigital(inputPin);
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(Helio_SensorType_TemperatureHumidity));
    HELIO_HARD_ASSERT(inputPinIsDigital, SFP(HStr_Err_InvalidPinOrType));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if (inputPinIsDigital && isValidIndex(positionIndex)) {
        auto sensor = SharedPtr<HelioDHTTempHumiditySensor>(new HelioDHTTempHumiditySensor(
            positionIndex,
            HelioDigitalPin(inputPin, INPUT_PULLUP),
            dhtType
        ));
        if (getController()->registerObject(sensor)) { return sensor; }
    }

    return nullptr;
}

SharedPtr<HelioBalancingPanel> HelioFactory::addLDRBalancingPanel(Helio_PanelType panelType, float intTolerance, float minIntensity, float axisOffset[2], float homePosition[2])
{
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(panelType));
    HELIO_SOFT_ASSERT((int)panelType >= 0 && panelType <= Helio_PanelType_Count, SFP(HStr_Err_InvalidParameter));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if ((int)panelType >= 0 && panelType < Helio_PanelType_Count && isValidIndex(positionIndex)) {
        auto panel = SharedPtr<HelioBalancingPanel>(new HelioBalancingPanel(
            panelType,
            positionIndex,
            intTolerance,
            minIntensity
        ));
        if (getController()->registerObject(panel)) {
            panel->setAxisOffset(axisOffset);
            panel->setHomePosition(homePosition);

            return panel;
        }
    }

    return nullptr;
}

SharedPtr<HelioTrackingPanel> HelioFactory::addSolarTrackingPanel(Helio_PanelType panelType, float angleTolerance, float axisOffset[2], float homePosition[2])
{
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(panelType));
    HELIO_SOFT_ASSERT((int)panelType >= 0 && panelType <= Helio_PanelType_Count, SFP(HStr_Err_InvalidParameter));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if ((int)panelType >= 0 && panelType < Helio_PanelType_Count && isValidIndex(positionIndex)) {
        auto panel = SharedPtr<HelioTrackingPanel>(new HelioTrackingPanel(
            panelType,
            positionIndex,
            angleTolerance
        ));
        if (getController()->registerObject(panel)) {
            panel->setAxisOffset(axisOffset);
            panel->setHomePosition(homePosition);

            return panel;
        }
    }

    return nullptr;
}

SharedPtr<HelioReflectingPanel> HelioFactory::addSolarReflectingPanel(Helio_PanelType panelType, float angleTolerance, float reflectPosition[2], float axisOffset[2], float homePosition[2])
{
    hposi_t positionIndex = getController()->firstPositionOpen(HelioIdentity(panelType));
    HELIO_SOFT_ASSERT((int)panelType >= 0 && panelType <= Helio_PanelType_Count, SFP(HStr_Err_InvalidParameter));
    HELIO_SOFT_ASSERT(isValidIndex(positionIndex), SFP(HStr_Err_NoPositionsAvailable));

    if ((int)panelType >= 0 && panelType < Helio_PanelType_Count && isValidIndex(positionIndex)) {
        auto panel = SharedPtr<HelioReflectingPanel>(new HelioReflectingPanel(
            panelType,
            positionIndex,
            angleTolerance
        ));
        if (getController()->registerObject(panel)) {
            panel->setReflectPosition(reflectPosition);
            panel->setAxisOffset(axisOffset);
            panel->setHomePosition(homePosition);
            
            return panel;
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

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helio Attachment Points
*/

#include "Helioduino.h"

inline Helio_UnitsType HelioDistanceUnitsInterface::getSpeedUnits() const
{
    return rateUnits(getDistanceUnits());
}

inline Helio_UnitsType HelioMeasureUnitsInterface::getRateUnits(uint8_t measureRow) const
{
    return rateUnits(getMeasureUnits(measureRow));
}

inline Helio_UnitsType HelioMeasureUnitsInterface::getBaseUnits(uint8_t measureRow) const
{
    return baseUnits(getMeasureUnits(measureRow));
}


inline void HelioActuatorObjectInterface::setContinuousPowerUsage(float contPowerUsage, Helio_UnitsType contPowerUsageUnits)
{
    setContinuousPowerUsage(HelioSingleMeasurement(contPowerUsage, contPowerUsageUnits));
}


inline void HelioMotorObjectInterface::setContinuousSpeed(float contSpeed, Helio_UnitsType contSpeedUnits)
{
    setContinuousSpeed(HelioSingleMeasurement(contSpeed, contSpeedUnits));
}


template <class U>
inline void HelioParentActuatorAttachmentInterface::setActuator(U actuator)
{
    getParentActuator().setObject(actuator);
}

template <class U>
inline SharedPtr<U> HelioParentActuatorAttachmentInterface::getActuator()
{
    return getParentActuator().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioParentSensorAttachmentInterface::setSensor(U sensor)
{
    getParentSensor().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioParentSensorAttachmentInterface::getSensor()
{
    return getParentSensor().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioParentPanelAttachmentInterface::setPanel(U panel)
{
    getParentPanel().setObject(panel);
}

template <class U>
inline SharedPtr<U> HelioParentPanelAttachmentInterface::getPanel()
{
    return getParentPanel().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioParentRailAttachmentInterface::setRail(U rail)
{
    getParentRail().setObject(rail);
}

template <class U>
inline SharedPtr<U> HelioParentRailAttachmentInterface::getRail()
{
    return getParentRail().HelioAttachment::getObject<U>();
}


template <class U>
inline void HelioAngleSensorAttachmentInterface::setAngleSensor(U sensor)
{
    getAngle().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioAngleSensorAttachmentInterface::getAngleSensor(bool poll)
{
    getAngle().updateIfNeeded(poll);
    return getAngle().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioPositionSensorAttachmentInterface::setPositionSensor(U sensor)
{
    getPosition().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPositionSensorAttachmentInterface::getPositionSensor(bool poll)
{
    getPosition().updateIfNeeded(poll);
    return getPosition().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioPowerProductionSensorAttachmentInterface::setPowerProductionSensor(U sensor)
{
    getPowerProduction().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPowerProductionSensorAttachmentInterface::getPowerProductionSensor(bool poll)
{
    getPowerProduction().updateIfNeeded(poll);
    return getPowerProduction().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioPowerUsageSensorAttachmentInterface::setPowerUsageSensor(U sensor)
{
    getPowerUsage().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPowerUsageSensorAttachmentInterface::getPowerUsageSensor(bool poll)
{
    getPowerUsage().updateIfNeeded(poll);
    return getPowerUsage().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioSpeedSensorAttachmentInterface::setSpeedSensor(U sensor)
{
    getSpeed().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioSpeedSensorAttachmentInterface::getSpeedSensor(bool poll)
{
    getSpeed().updateIfNeeded(poll);
    return getSpeed().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioTemperatureSensorAttachmentInterface::setTemperatureSensor(U sensor)
{
    getTemperature().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioTemperatureSensorAttachmentInterface::getTemperatureSensor(bool poll)
{
    getTemperature().updateIfNeeded(poll);
    return getTemperature().HelioAttachment::getObject<U>();
}


template <class U>
inline void HelioMinTravelTriggerAttachmentInterface::setMinTravelTrigger(U trigger)
{
    getMinTravel().setObject(trigger);
}

template <class U>
inline SharedPtr<U> HelioMinTravelTriggerAttachmentInterface::getMinTravelTrigger(bool poll)
{
    getMinTravel().updateIfNeeded(poll);
    return getMinTravel().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioMaxTravelTriggerAttachmentInterface::setMaxTravelTrigger(U trigger)
{
    getMaxTravel().setObject(trigger);
}

template <class U>
inline SharedPtr<U> HelioMaxTravelTriggerAttachmentInterface::getMaxTravelTrigger(bool poll)
{
    getMaxTravel().updateIfNeeded(poll);
    return getMaxTravel().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioLimitTriggerAttachmentInterface::setLimitTrigger(U trigger)
{
    getLimit().setObject(trigger);
}

template <class U>
inline SharedPtr<U> HelioLimitTriggerAttachmentInterface::getLimitTrigger(bool poll)
{
    getLimit().updateIfNeeded(poll);
    return getLimit().HelioAttachment::getObject<U>();
}

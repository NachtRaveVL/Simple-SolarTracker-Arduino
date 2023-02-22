/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helio Attachment Points
*/

#include "Helioduino.h"

inline void HelioDistanceUnitsInterfaceStorage::setSpeedUnits(Helio_UnitsType speedUnits)
{
    setDistanceUnits(baseUnits(speedUnits));
}

inline Helio_UnitsType HelioDistanceUnitsInterfaceStorage::getSpeedUnits() const
{
    return rateUnits(getDistanceUnits());
}

inline Helio_UnitsType HelioMeasurementUnitsInterface::getRateUnits(uint8_t measurementRow) const
{
    return rateUnits(getMeasurementUnits(measurementRow));
}

inline Helio_UnitsType HelioMeasurementUnitsInterface::getBaseUnits(uint8_t measurementRow) const
{
    return baseUnits(getMeasurementUnits(measurementRow));
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
inline void HelioParentActuatorAttachmentInterface::setParentActuator(U actuator)
{
    getParentActuatorAttachment().setObject(actuator);
}

template <class U>
inline SharedPtr<U> HelioParentActuatorAttachmentInterface::getParentActuator()
{
    return getParentActuatorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioParentSensorAttachmentInterface::setParentSensor(U sensor)
{
    getParentSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioParentSensorAttachmentInterface::getParentSensor()
{
    return getParentSensorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioParentPanelAttachmentInterface::setParentPanel(U panel, hposi_t axisIndex)
{
    getParentPanelAttachment().setObject(panel);
    getParentPanelAttachment().setParentSubIndex(axisIndex);
}

template <class U>
inline SharedPtr<U> HelioParentPanelAttachmentInterface::getParentPanel()
{
    return getParentPanelAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioParentRailAttachmentInterface::setParentRail(U rail)
{
    getParentRailAttachment().setObject(rail);
}

template <class U>
inline SharedPtr<U> HelioParentRailAttachmentInterface::getParentRail()
{
    return getParentRailAttachment().HelioAttachment::getObject<U>();
}


template <class U>
inline void HelioSensorAttachmentInterface::setSensor(U sensor)
{
    getSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioSensorAttachmentInterface::getSensor(bool poll)
{
    getSensorAttachment().updateIfNeeded(poll);
    return getSensorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioAngleSensorAttachmentInterface::setAngleSensor(U sensor)
{
    getAngleSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioAngleSensorAttachmentInterface::getAngleSensor(bool poll)
{
    getAngleSensorAttachment().updateIfNeeded(poll);
    return getAngleSensorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioPositionSensorAttachmentInterface::setPositionSensor(U sensor)
{
    getPositionSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPositionSensorAttachmentInterface::getPositionSensor(bool poll)
{
    getPositionSensorAttachment().updateIfNeeded(poll);
    return getPositionSensorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioPowerProductionSensorAttachmentInterface::setPowerProductionSensor(U sensor)
{
    getPowerProductionSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPowerProductionSensorAttachmentInterface::getPowerProductionSensor(bool poll)
{
    getPowerProductionSensorAttachment().updateIfNeeded(poll);
    return getPowerProductionSensorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioPowerUsageSensorAttachmentInterface::setPowerUsageSensor(U sensor)
{
    getPowerUsageSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPowerUsageSensorAttachmentInterface::getPowerUsageSensor(bool poll)
{
    getPowerUsageSensorAttachment().updateIfNeeded(poll);
    return getPowerUsageSensorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioSpeedSensorAttachmentInterface::setSpeedSensor(U sensor)
{
    getSpeedSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioSpeedSensorAttachmentInterface::getSpeedSensor(bool poll)
{
    getSpeedSensorAttachment().updateIfNeeded(poll);
    return getSpeedSensorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioTemperatureSensorAttachmentInterface::setTemperatureSensor(U sensor)
{
    getTemperatureSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioTemperatureSensorAttachmentInterface::getTemperatureSensor(bool poll)
{
    getTemperatureSensorAttachment().updateIfNeeded(poll);
    return getTemperatureSensorAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioWindSpeedSensorAttachmentInterface::setWindSpeedSensor(U sensor)
{
    getWindSpeedSensorAttachment().setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioWindSpeedSensorAttachmentInterface::getWindSpeedSensor(bool poll)
{
    getWindSpeedSensorAttachment().updateIfNeeded(poll);
    return getWindSpeedSensorAttachment().HelioAttachment::getObject<U>();
}


template <class U>
inline void HelioTriggerAttachmentInterface::setTrigger(U trigger)
{
    getTriggerAttachment().setObject(trigger);
}

template <class U>
inline SharedPtr<U> HelioTriggerAttachmentInterface::getTrigger(bool poll)
{
    getTriggerAttachment().updateIfNeeded(poll);
    return getTriggerAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioMinimumTriggerAttachmentInterface::setMinimumTrigger(U trigger)
{
    getMinimumTriggerAttachment().setObject(trigger);
}

template <class U>
inline SharedPtr<U> HelioMinimumTriggerAttachmentInterface::getMinimumTrigger(bool poll)
{
    getMinimumTriggerAttachment().updateIfNeeded(poll);
    return getMinimumTriggerAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioMaximumTriggerAttachmentInterface::setMaximumTrigger(U trigger)
{
    getMaximumTriggerAttachment().setObject(trigger);
}

template <class U>
inline SharedPtr<U> HelioMaximumTriggerAttachmentInterface::getMaximumTrigger(bool poll)
{
    getMaximumTriggerAttachment().updateIfNeeded(poll);
    return getMaximumTriggerAttachment().HelioAttachment::getObject<U>();
}

template <class U>
inline void HelioLimitTriggerAttachmentInterface::setLimitTrigger(U trigger)
{
    getLimitTriggerAttachment().setObject(trigger);
}

template <class U>
inline SharedPtr<U> HelioLimitTriggerAttachmentInterface::getLimitTrigger(bool poll)
{
    getLimitTriggerAttachment().updateIfNeeded(poll);
    return getLimitTriggerAttachment().HelioAttachment::getObject<U>();
}

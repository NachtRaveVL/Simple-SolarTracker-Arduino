/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helio Attachment Points
*/

#include "Helioduino.h"

template <class U>
inline void HelioActuatorAttachmentInterface::setActuator(U actuator)
{
    getParentActuator(false).setObject(actuator);
}

template <class U>
inline SharedPtr<U> HelioActuatorAttachmentInterface::getActuator(bool resolve)
{
    return getParentActuator(resolve).template getObject<U>();
}

template <class U>
inline void HelioSensorAttachmentInterface::setSensor(U sensor)
{
    getParentSensor(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioSensorAttachmentInterface::getSensor(bool resolve)
{
    return getParentSensor(resolve).template getObject<U>();
}

template <class U>
inline void HelioPanelAttachmentInterface::setPanel(U panel)
{
    getParentPanel(false).setObject(panel);
}

template <class U>
inline SharedPtr<U> HelioPanelAttachmentInterface::getPanel(bool resolve)
{
    return getParentPanel(resolve).template getObject<U>();
}

template <class U>
inline void HelioRailAttachmentInterface::setRail(U rail)
{
    getParentRail(false).setObject(rail);
}

template <class U>
inline SharedPtr<U> HelioRailAttachmentInterface::getRail(bool resolve)
{
    return getParentRail(resolve).template getObject<U>();
}


inline void HelioActuatorObjectInterface::setContinuousPowerUsage(float contPowerUsage, Helio_UnitsType contPowerUsageUnits)
{
    setContinuousPowerUsage(HelioSingleMeasurement(contPowerUsage, contPowerUsageUnits));
}

inline bool HelioBalancerObjectInterface::isBalanced() const
{
    return getBalancerState() == Helio_BalancerState_Balanced;
}

inline void HelioMotorObjectInterface::setContinuousSpeed(float contSpeed, Helio_UnitsType contSpeedUnits)
{
    setContinuousSpeed(HelioSingleMeasurement(contSpeed, contSpeedUnits));
}


template <class U>
inline void HelioPositionSensorAttachmentInterface::setPositionSensor(U sensor)
{
    getPosition(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPositionSensorAttachmentInterface::getPositionSensor(bool poll)
{
    return static_pointer_cast<U>(getPosition(poll).getObject());
}

template <class U>
inline void HelioSpeedSensorAttachmentInterface::setSpeedSensor(U sensor)
{
    getSpeed(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioSpeedSensorAttachmentInterface::getSpeedSensor(bool poll)
{
    return static_pointer_cast<U>(getSpeed(poll).getObject());
}

template <class U>
inline void HelioPowerProductionSensorAttachmentInterface::setPowerProductionSensor(U sensor)
{
    getPowerProduction(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPowerProductionSensorAttachmentInterface::getPowerProductionSensor(bool poll)
{
    return static_pointer_cast<U>(getPowerProduction(poll).getObject());
}

template <class U>
inline void HelioPowerUsageSensorAttachmentInterface::setPowerUsageSensor(U sensor)
{
    getPowerUsage(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPowerUsageSensorAttachmentInterface::getPowerUsageSensor(bool poll)
{
    return static_pointer_cast<U>(getPowerUsage(poll).getObject());
}

template <class U>
inline void HelioAirTemperatureSensorAttachmentInterface::setAirTemperatureSensor(U sensor)
{
    getAirTemperature(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioAirTemperatureSensorAttachmentInterface::getAirTemperatureSensor(bool poll)
{
    return static_pointer_cast<U>(getAirTemperature(poll).getObject());
}

template <class U>
inline void HelioAirHumiditySensorAttachmentInterface::setAirHumiditySensor(U sensor)
{
    getAirHumidity(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioAirHumiditySensorAttachmentInterface::getAirHumiditySensor(bool poll)
{
    return static_pointer_cast<U>(getAirHumidity(poll).getObject());
}

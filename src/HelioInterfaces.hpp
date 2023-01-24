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

inline bool HelioBalancerObjectInterface::isBalanced() const
{
    return getBalancerState() == Helio_BalancerState_Balanced;
}

template <class U>
inline void HelioPumpObjectInterface::setInputPanel(U panel)
{
    getParentPanel(false).setObject(panel);
}

template <class U>
inline SharedPtr<U> HelioPumpObjectInterface::getInputPanel(bool resolve)
{
    return getParentPanel(resolve).template getObject<U>();
}

template <class U>
inline void HelioPumpObjectInterface::setOutputPanel(U panel)
{
    getDestinationPanel(false).setObject(panel);
}

template <class U>
inline SharedPtr<U> HelioPumpObjectInterface::getOutputPanel(bool resolve)
{
    return getDestinationPanel(resolve).template getObject<U>();
}

template <class U>
inline void HelioFeedPanelAttachmentInterface::setFeedPanel(U panel)
{
    getFeedingPanel(false).setObject(panel);
}

template <class U>
inline SharedPtr<U> HelioFeedPanelAttachmentInterface::getFeedPanel(bool resolve)
{
    return getFeedingPanel(resolve).template getObject<U>();
}

template <class U>
inline void HelioFlowSensorAttachmentInterface::setFlowRateSensor(U sensor)
{
    getFlowRate(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioFlowSensorAttachmentInterface::getFlowRateSensor(bool poll)
{
    return static_pointer_cast<U>(getFlowRate(poll).getObject());
}

template <class U>
inline void HelioVolumeSensorAttachmentInterface::setWaterVolumeSensor(U sensor)
{
    getWaterVolume(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioVolumeSensorAttachmentInterface::getWaterVolumeSensor(bool poll)
{
    return static_pointer_cast<U>(getWaterVolume(poll).getObject());
}

template <class U>
inline void HelioPowerSensorAttachmentInterface::setPowerUsageSensor(U sensor)
{
    getPowerUsage(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioPowerSensorAttachmentInterface::getPowerUsageSensor(bool poll)
{
    return static_pointer_cast<U>(getPowerUsage(poll).getObject());
}

template <class U>
inline void HelioWaterTemperatureSensorAttachmentInterface::setWaterTemperatureSensor(U sensor)
{
    getWaterTemperature(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioWaterTemperatureSensorAttachmentInterface::getWaterTemperatureSensor(bool poll)
{
    return static_pointer_cast<U>(getWaterTemperature(poll).getObject());
}

template <class U>
inline void HelioWaterPHSensorAttachmentInterface::setWaterPHSensor(U sensor)
{
    getWaterPH(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioWaterPHSensorAttachmentInterface::getWaterPHSensor(bool poll)
{
    return static_pointer_cast<U>(getWaterPH(poll).getObject());
}

template <class U>
inline void HelioWaterTDSSensorAttachmentInterface::setWaterTDSSensor(U sensor)
{
    getWaterTDS(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioWaterTDSSensorAttachmentInterface::getWaterTDSSensor(bool poll)
{
    return static_pointer_cast<U>(getWaterTDS(poll).getObject());
}

template <class U>
inline void HelioSoilMoistureSensorAttachmentInterface::setSoilMoistureSensor(U sensor)
{
    getSoilMoisture(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioSoilMoistureSensorAttachmentInterface::getSoilMoistureSensor(bool poll)
{
    return static_pointer_cast<U>(getSoilMoisture(poll).getObject());
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

template <class U>
inline void HelioAirCO2SensorAttachmentInterface::setAirCO2Sensor(U sensor)
{
    getAirCO2(false).setObject(sensor);
}

template <class U>
inline SharedPtr<U> HelioAirCO2SensorAttachmentInterface::getAirCO2Sensor(bool poll)
{
    return static_pointer_cast<U>(getAirCO2(poll).getObject());
}

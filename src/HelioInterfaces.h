/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Interfaces
*/

#ifndef HelioInterfaces_H
#define HelioInterfaces_H

struct HelioJSONSerializableInterface;

class HelioObjInterface;
class HelioUIInterface;

struct HelioDigitalInputPinInterface;
struct HelioDigitalOutputPinInterface;
struct HelioAnalogInputPinInterface;
struct HelioAnalogOutputPinInterface;
class HelioRTCInterface;

class HelioActuatorAttachmentInterface;
class HelioSensorAttachmentInterface;
class HelioPanelAttachmentInterface;
class HelioRailAttachmentInterface;

class HelioActuatorObjectInterface;
class HelioSensorObjectInterface;
class HelioPanelObjectInterface;
class HelioRailObjectInterface;

class HelioTriggerObjectInterface;

class HelioPowerSensorAttachmentInterface;
class HelioAirTemperatureSensorAttachmentInterface;
class HelioAirHumiditySensorAttachmentInterface;

#include "Helioduino.h"

// JSON Serializable Interface
struct HelioJSONSerializableInterface {
    // Given a JSON element to fill in, writes self to JSON format.
    virtual void toJSONObject(JsonObject &objectOut) const = 0;

    // Given a JSON element to read from, reads overtop self from JSON format.
    virtual void fromJSONObject(JsonObjectConst &objectIn) = 0;
};


// Object Interface
class HelioObjInterface {
public:
    virtual HelioIdentity getId() const = 0;
    virtual Helio_KeyType getKey() const = 0;
    virtual String getKeyString() const = 0;
    virtual SharedPtr<HelioObjInterface> getSharedPtr() const = 0;

    virtual bool addLinkage(HelioObject *obj) = 0;
    virtual bool removeLinkage(HelioObject *obj) = 0;
};

// UI Interface
class HelioUIInterface {
public:
    virtual void begin() = 0;

    virtual void setNeedsLayout() = 0;
};


// Digital Input Pin Interface
struct HelioDigitalInputPinInterface {
    virtual Arduino_PinStatusType digitalRead() = 0;
    inline int get() { return digitalRead(); }
};

// Digital Output Pin Interface
struct HelioDigitalOutputPinInterface {
    virtual void digitalWrite(Arduino_PinStatusType status) = 0;
    inline void set(Arduino_PinStatusType status) { digitalWrite(status); }
};

// Analog Input Pin Interface
struct HelioAnalogInputPinInterface {
    virtual float analogRead() = 0;
    virtual int analogRead_raw() = 0;
    inline float get() { return analogRead(); }
    inline int get_raw() { return analogRead_raw(); }
};

// Analog Output Pin Interface
struct HelioAnalogOutputPinInterface {
    virtual void analogWrite(float amount) = 0;
    virtual void analogWrite_raw(int amount) = 0;
    inline void set(float amount) { analogWrite(amount); }
    inline void set_raw(int amount) { analogWrite_raw(amount); }
};


// RTC Module Interface
class HelioRTCInterface {
public:
    virtual bool begin(TwoWire *wireInstance) = 0;
    virtual void adjust(const DateTime &dt) = 0;
    virtual bool lostPower(void) = 0;
    virtual DateTime now() = 0;
};



// Actuator Attachment Interface
class HelioActuatorAttachmentInterface {
public:
    virtual HelioAttachment &getParentActuator(bool resolve = true) = 0;

    template<class U> inline void setActuator(U actuator);
    template<class U = HelioActuator> inline SharedPtr<U> getActuator(bool resolve = true);
};

// Sensor Attachment Interface
class HelioSensorAttachmentInterface {
public:
    virtual HelioAttachment &getParentSensor(bool resolve = true) = 0;

    template<class U> inline void setSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getSensor(bool resolve = true);
};

// Panel Attachment Interface
class HelioPanelAttachmentInterface {
public:
    virtual HelioAttachment &getParentPanel(bool resolve = true) = 0;

    template<class U> inline void setPanel(U panel);
    template<class U = HelioPanel> inline SharedPtr<U> getPanel(bool resolve = true);
};

// Rail Attachment Interface
class HelioRailAttachmentInterface {
public:
    virtual HelioAttachment &getParentRail(bool resolve = true) = 0;

    template<class U> inline void setRail(U rail);
    template<class U = HelioRail> inline SharedPtr<U> getRail(bool resolve = true);
};


// Actuator Object Interface
class HelioActuatorObjectInterface {
public:
    virtual bool getCanEnable() = 0;
    virtual bool isEnabled(float tolerance = 0.0f) const = 0;

    virtual void setContinuousPowerUsage(float contPowerUsage, Helio_UnitsType contPowerUsageUnits = Helio_UnitsType_Undefined) = 0;
    virtual void setContinuousPowerUsage(HelioSingleMeasurement contPowerUsage) = 0;
    virtual const HelioSingleMeasurement &getContinuousPowerUsage() = 0;

protected:
    virtual bool _enableActuator(float intensity = 1.0) = 0;
    virtual void _disableActuator() = 0;
};

// Sensor Object Interface
class HelioSensorObjectInterface {
public:
    virtual bool takeMeasurement(bool force = false) = 0;
    virtual const HelioMeasurement *getLatestMeasurement() const = 0;
    virtual bool isTakingMeasurement() const = 0;
    virtual bool needsPolling(uint32_t allowance = 0) const = 0;
};

// Panel Object Interface
class HelioPanelObjectInterface {
public:
    virtual bool canActivate(HelioActuator *actuator) = 0;

    // todo
};

// Rail Object Interface
class HelioRailObjectInterface {
public:
    virtual bool canActivate(HelioActuator *actuator) = 0;
    virtual float getCapacity() = 0;

    virtual void setPowerUnits(Helio_UnitsType powerUnits) = 0;
    virtual Helio_UnitsType getPowerUnits() const = 0;

    virtual float getRailVoltage() const = 0;
};


// Balancer Object Interface
class HelioBalancerObjectInterface {
public:
    virtual void setTargetSetpoint(float targetSetpoint) = 0;
    virtual Helio_BalancerState getBalancerState() const = 0;
    inline bool isBalanced() const;
};

// Trigger Object Interface
class HelioTriggerObjectInterface {
public:
    virtual Helio_TriggerState getTriggerState() const = 0;
};


// Power Aware Interface
class HelioPowerSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPowerUsage(bool poll = false) = 0;

    template<class U> inline void setPowerUsageSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPowerUsageSensor(bool poll = false);
};

// Air Temperature Aware Interface
class HelioAirTemperatureSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getAirTemperature(bool poll = false) = 0;

    template<class U> inline void setAirTemperatureSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getAirTemperatureSensor(bool poll = false);
};

// Air Humidity Aware Interface
class HelioAirHumiditySensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getAirHumidity(bool poll = false) = 0;

    template<class U> inline void setAirHumiditySensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getAirHumiditySensor(bool poll = false);
};

#endif // /ifndef HelioInterfaces_H

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Interfaces
*/

#ifndef HelioInterfaces_H
#define HelioInterfaces_H

struct HelioJSONSerializableInterface;

class HelioObjInterface;
class HelioUIInterface;
class HelioRTCInterface;

struct HelioDigitalInputPinInterface;
struct HelioDigitalOutputPinInterface;
struct HelioAnalogInputPinInterface;
struct HelioAnalogOutputPinInterface;

class HelioAngleUnitsInterface;
class HelioDistanceUnitsInterface;
class HelioMeasureUnitsInterface;
class HelioPowerUnitsInterface;
class HelioTemperatureUnitsInterface;

class HelioActuatorObjectInterface;
class HelioSensorObjectInterface;
class HelioPanelObjectInterface;
class HelioRailObjectInterface;
class HelioDriverObjectInterface;
class HelioTriggerObjectInterface;
class HelioMotorObjectInterface;

class HelioActuatorAttachmentInterface;
class HelioSensorAttachmentInterface;
class HelioPanelAttachmentInterface;
class HelioRailAttachmentInterface;

class HelioAngleSensorAttachmentInterface;
class HelioPositionSensorAttachmentInterface;
class HelioSpeedSensorAttachmentInterface;
class HelioMinEndstopSensorAttachmentInterface;
class HelioMaxEndstopSensorAttachmentInterface;
class HelioPowerProductionSensorAttachmentInterface;
class HelioPowerUsageSensorAttachmentInterface;
class HelioTemperatureSensorAttachmentInterface;

class HelioMinTravelTriggerAttachmentInterface;
class HelioMaxTravelTriggerAttachmentInterface;
class HelioLimitTriggerAttachmentInterface;

#include "Helioduino.h"
#include "HelioUtils.h"

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
    virtual void unresolveAny(HelioObject *obj) = 0;

    virtual HelioIdentity getId() const = 0;
    virtual hkey_t getKey() const = 0;
    virtual String getKeyString() const = 0;
    virtual SharedPtr<HelioObjInterface> getSharedPtr() const = 0;

    virtual bool isObject() const = 0;
    inline bool isSubObject() const { return !isObject(); }
};

// UI Interface
class HelioUIInterface {
public:
    virtual void begin() = 0;

    virtual void setNeedsLayout() = 0;
};

// RTC Module Interface
class HelioRTCInterface {
public:
    virtual bool begin(TwoWire *wireInstance) = 0;
    virtual void adjust(const DateTime &dt) = 0;
    virtual bool lostPower(void) = 0;
    virtual DateTime now() = 0;
};


// Digital Input Pin Interface
struct HelioDigitalInputPinInterface {
    virtual ard_pinstatus_t digitalRead() = 0;
    inline int get() { return digitalRead(); }
};

// Digital Output Pin Interface
struct HelioDigitalOutputPinInterface {
    virtual void digitalWrite(ard_pinstatus_t status) = 0;
    inline void set(ard_pinstatus_t status) { digitalWrite(status); }
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


// Angle Units Interface
class HelioAngleUnitsInterface {
public:
    virtual void setAngleUnits(Helio_UnitsType angleUnits) = 0;
    inline Helio_UnitsType getAngleUnits() const { return _angleUnits; }

protected:
    Helio_UnitsType _angleUnits;
    inline HelioAngleUnitsInterface(Helio_UnitsType angleUnits = defaultAngleUnits()) : _angleUnits(angleUnits) { ; }
};

// Distance Units Interface
class HelioDistanceUnitsInterface {
public:
    virtual void setDistanceUnits(Helio_UnitsType distanceUnits) = 0;
    inline Helio_UnitsType getDistanceUnits() const { return _distanceUnits; }
    inline Helio_UnitsType getSpeedUnits() const { return rateUnits(_distanceUnits); }

protected:
    Helio_UnitsType _distanceUnits;
    inline HelioDistanceUnitsInterface(Helio_UnitsType distanceUnits = defaultDistanceUnits()) : _distanceUnits(distanceUnits) { ; }
};

// Measure Units Interface
class HelioMeasureUnitsInterface {
public:
    virtual void setMeasureUnits(Helio_UnitsType measureUnits) = 0;
    inline Helio_UnitsType getMeasureUnits() const { return _measureUnits; }
    inline Helio_UnitsType getRateUnits() const { rateUnits(_measureUnits); }
    inline Helio_UnitsType getBaseUnits() const { baseUnits(_measureUnits); }

protected:
    Helio_UnitsType _measureUnits;
    inline HelioMeasureUnitsInterface(Helio_UnitsType measureUnits = Helio_UnitsType_Undefined) : _measureUnits(measureUnits) { ; }
};

// Power Units Interface
class HelioPowerUnitsInterface {
public:
    virtual void setPowerUnits(Helio_UnitsType powerUnits) = 0;
    inline Helio_UnitsType getPowerUnits() const { return _powerUnits; }

protected:
    Helio_UnitsType _powerUnits;
    inline HelioPowerUnitsInterface(Helio_UnitsType powerUnits = defaultPowerUnits()) : _powerUnits(powerUnits) { ; }
};

// Temperature Units Interface
class HelioTemperatureUnitsInterface {
public:
    virtual void setTemperatureUnits(Helio_UnitsType temperatureUnits) = 0;
    inline Helio_UnitsType getTemperatureUnits() const { return _temperatureUnits; }

protected:
    Helio_UnitsType _temperatureUnits;
    inline HelioTemperatureUnitsInterface(Helio_UnitsType temperatureUnits = defaultTemperatureUnits()) : _temperatureUnits(temperatureUnits) { ; }
};


// Actuator Object Interface
class HelioActuatorObjectInterface {
public:
    virtual bool getCanEnable() = 0;
    virtual float getDriveIntensity() const = 0;
    virtual bool isEnabled(float tolerance = 0.0f) const = 0;

    virtual void setContinuousPowerUsage(HelioSingleMeasurement contPowerUsage) = 0;
    virtual const HelioSingleMeasurement &getContinuousPowerUsage() = 0;
    inline void setContinuousPowerUsage(float contPowerUsage, Helio_UnitsType contPowerUsageUnits = Helio_UnitsType_Undefined);

protected:
    virtual void _enableActuator(float intensity = 1.0) = 0;
    virtual void _disableActuator() = 0;
    virtual void handleActivation() = 0;
};

// Sensor Object Interface
class HelioSensorObjectInterface {
public:
    virtual bool takeMeasurement(bool force = false) = 0;
    virtual const HelioMeasurement *getLatestMeasurement() const = 0;
    virtual bool isTakingMeasurement() const = 0;
    virtual bool getNeedsPolling(hframe_t allowance = 0) const = 0;

protected:
    //virtual void handleMeasurement() = 0;
};

// Panel Object Interface
class HelioPanelObjectInterface {
public:
    virtual bool canActivate(HelioActuator *actuator) = 0;
    virtual bool isAligned(bool poll = false) = 0;

    virtual void setPowerUnits(Helio_UnitsType powerUnits) = 0;
    virtual Helio_UnitsType getPowerUnits() const = 0;
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

// Driver Object Interface
class HelioDriverObjectInterface {
public:
    virtual void setTargetSetpoint(float targetSetpoint) = 0;
    virtual void setTravelRate(float travelRate) = 0;

    virtual float getMaximumOffset(bool poll = false) = 0;
    virtual Helio_DrivingState getDrivingState() const = 0;

protected:
    virtual void handleOffset(float maximumOffset) = 0;
};

// Trigger Object Interface
class HelioTriggerObjectInterface {
public:
    virtual Helio_TriggerState getTriggerState() const = 0;

protected:
    virtual void handleMeasurement(const HelioMeasurement *measurement) = 0;
};


// Motor Object Interface
class HelioMotorObjectInterface {
public:
    virtual bool canTravel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) = 0;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) = 0;
    virtual bool canTravel(Helio_DirectionMode direction, millis_t time) = 0;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, millis_t time) = 0;

    virtual void setDistanceUnits(Helio_UnitsType distanceUnits) = 0;
    virtual Helio_UnitsType getDistanceUnits() const = 0;

    virtual void setContinuousSpeed(HelioSingleMeasurement contSpeed) = 0;
    virtual const HelioSingleMeasurement &getContinuousSpeed() = 0;
    inline void setContinuousSpeed(float contSpeed, Helio_UnitsType contSpeedUnits = Helio_UnitsType_Undefined);

protected:
    virtual void pollTravelingSensors() = 0;
    virtual void handleTravelTime(millis_t time) = 0;
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


// Angle Sensor Attachment Interface
class HelioAngleSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getAngle(bool poll = false) = 0;

    template<class U> inline void setAngleSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getAngleSensor(bool poll = false);
};

// Position Sensor Attachment Interface
class HelioPositionSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPosition(bool poll = false) = 0;

    template<class U> inline void setPositionSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPositionSensor(bool poll = false);
};

// Speed Sensor Attachment Interface
class HelioSpeedSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getSpeed(bool poll = false) = 0;

    template<class U> inline void setSpeedSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getSpeedSensor(bool poll = false);
};

// Min Endstop Sensor Attachment Interface
class HelioMinEndstopSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getMinimum(bool poll = false) = 0;

    template<class U> inline void setMinEndstop(U endstop);
    template<class U = HelioSensor> inline SharedPtr<U> getMinEndstop(bool poll = false);
};

// Max Endstop Sensor Attachment Interface
class HelioMaxEndstopSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getMaximum(bool poll = false) = 0;

    template<class U> inline void setMaxEndstop(U endstop);
    template<class U = HelioSensor> inline SharedPtr<U> getMaxEndstop(bool poll = false);
};

// Power Production Sensor Attachment Interface
class HelioPowerProductionSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPowerProduction(bool poll = false) = 0;

    template<class U> inline void setPowerProductionSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPowerProductionSensor(bool poll = false);
};

// Power Usage Sensor Attachment Interface
class HelioPowerUsageSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPowerUsage(bool poll = false) = 0;

    template<class U> inline void setPowerUsageSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPowerUsageSensor(bool poll = false);
};

// Temperature Sensor Attachment Interface
class HelioTemperatureSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getTemperature(bool poll = false) = 0;

    template<class U> inline void setTemperatureSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getTemperatureSensor(bool poll = false);
};

#endif // /ifndef HelioInterfaces_H

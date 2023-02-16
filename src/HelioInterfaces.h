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
template <size_t N = 1> class HelioMeasureUnitsStorage;
class HelioPowerUnitsInterface;
class HelioTemperatureUnitsInterface;

class HelioActuatorObjectInterface;
class HelioSensorObjectInterface;
class HelioPanelObjectInterface;
class HelioRailObjectInterface;
class HelioDriverObjectInterface;
class HelioTriggerObjectInterface;

class HelioMotorObjectInterface;

class HelioParentActuatorAttachmentInterface;
class HelioParentSensorAttachmentInterface;
class HelioParentPanelAttachmentInterface;
class HelioParentRailAttachmentInterface;

class HelioAngleSensorAttachmentInterface;
class HelioPositionSensorAttachmentInterface;
class HelioPowerProductionSensorAttachmentInterface;
class HelioPowerUsageSensorAttachmentInterface;
class HelioSpeedSensorAttachmentInterface;
class HelioTemperatureSensorAttachmentInterface;

class HelioMinTravelTriggerAttachmentInterface;
class HelioMaxTravelTriggerAttachmentInterface;
class HelioLimitTriggerAttachmentInterface;

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


// Angle Units Interface + Storage
class HelioAngleUnitsInterface {
public:
    virtual void setAngleUnits(Helio_UnitsType angleUnits) = 0;
    inline Helio_UnitsType getAngleUnits() const { return _angleUnits; }

protected:
    Helio_UnitsType _angleUnits;
    inline HelioAngleUnitsInterface(Helio_UnitsType angleUnits) : _angleUnits(angleUnits) { ; }
};

// Distance Units Interface + Storage
class HelioDistanceUnitsInterface {
public:
    virtual void setDistanceUnits(Helio_UnitsType distanceUnits) = 0;
    inline Helio_UnitsType getDistanceUnits() const { return _distUnits; }
    inline Helio_UnitsType getSpeedUnits() const;

protected:
    Helio_UnitsType _distUnits;
    inline HelioDistanceUnitsInterface(Helio_UnitsType distanceUnits) : _distUnits(distanceUnits) { ; }
};

// Measure Units Interface
// Uses virtual getMeasureUnits() so that units can be local or shadowed
class HelioMeasureUnitsInterface {
public:
    virtual void setMeasureUnits(Helio_UnitsType measureUnits, uint8_t measureRow = 0) = 0;
    virtual Helio_UnitsType getMeasureUnits(uint8_t measureRow = 0) const = 0;

    inline Helio_UnitsType getRateUnits(uint8_t measureRow = 0) const;
    inline Helio_UnitsType getBaseUnits(uint8_t measureRow = 0) const;
};

// Measure Units Storage
template <size_t N> class HelioMeasureUnitsStorage {
protected:
    Helio_UnitsType _measureUnits[N];
    inline HelioMeasureUnitsStorage(Helio_UnitsType measureUnits = Helio_UnitsType_Undefined) { for (hposi_t i = 0; i < N; ++i) { _measureUnits[i] = measureUnits; } }
};

// Power Units Interface + Storage
class HelioPowerUnitsInterface {
public:
    virtual void setPowerUnits(Helio_UnitsType powerUnits) = 0;
    inline Helio_UnitsType getPowerUnits() const { return _powerUnits; }

protected:
    Helio_UnitsType _powerUnits;
    inline HelioPowerUnitsInterface(Helio_UnitsType powerUnits) : _powerUnits(powerUnits) { ; }
};

// Temperature Units Interface + Storage
class HelioTemperatureUnitsInterface {
public:
    virtual void setTemperatureUnits(Helio_UnitsType temperatureUnits) = 0;
    inline Helio_UnitsType getTemperatureUnits() const { return _tempUnits; }

protected:
    Helio_UnitsType _tempUnits;
    inline HelioTemperatureUnitsInterface(Helio_UnitsType temperatureUnits) : _tempUnits(temperatureUnits) { ; }
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
};

// Panel Object Interface
class HelioPanelObjectInterface {
public:
    virtual bool canActivate(HelioActuator *actuator) = 0;
    virtual bool isAligned(bool poll = false) = 0;
};

// Rail Object Interface
class HelioRailObjectInterface {
public:
    virtual bool canActivate(HelioActuator *actuator) = 0;
    virtual float getCapacity(bool poll = false) = 0;
};

// Driver Object Interface
class HelioDriverObjectInterface {
public:
    virtual void setTargetSetpoint(float targetSetpoint) = 0;
    virtual float getMaximumOffset(bool poll = false) = 0;
    virtual Helio_DrivingState getDrivingState(bool poll = false) = 0;
    inline bool isOnTarget(bool poll = false);
};

// Trigger Object Interface
class HelioTriggerObjectInterface {
public:
    virtual Helio_TriggerState getTriggerState(bool poll = false) = 0;
    inline bool isTriggered(bool poll = false);
};


// Motor Object Interface
class HelioMotorObjectInterface {
public:
    virtual bool canTravel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) = 0;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) = 0;
    virtual bool canTravel(Helio_DirectionMode direction, millis_t time) = 0;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, millis_t time) = 0;

    virtual void setContinuousSpeed(HelioSingleMeasurement contSpeed) = 0;
    virtual const HelioSingleMeasurement &getContinuousSpeed() = 0;
    inline void setContinuousSpeed(float contSpeed, Helio_UnitsType contSpeedUnits = Helio_UnitsType_Undefined);

protected:
    virtual void pollTravelingSensors() = 0;
    virtual void handleTravelTime(millis_t time) = 0;
};


// Parent Actuator Attachment Interface
class HelioParentActuatorAttachmentInterface {
public:
    virtual HelioAttachment &getParentActuator() = 0;

    template<class U> inline void setActuator(U actuator);
    template<class U = HelioActuator> inline SharedPtr<U> getActuator();
};

// Parent Sensor Attachment Interface
class HelioParentSensorAttachmentInterface {
public:
    virtual HelioAttachment &getParentSensor() = 0;

    template<class U> inline void setSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getSensor();
};

// Parent Panel Attachment Interface
class HelioParentPanelAttachmentInterface {
public:
    virtual HelioAttachment &getParentPanel() = 0;

    template<class U> inline void setPanel(U panel);
    template<class U = HelioPanel> inline SharedPtr<U> getPanel();
};

// Parent Rail Attachment Interface
class HelioParentRailAttachmentInterface {
public:
    virtual HelioAttachment &getParentRail() = 0;

    template<class U> inline void setRail(U rail);
    template<class U = HelioRail> inline SharedPtr<U> getRail();
};


// Angle Sensor Attachment Interface
class HelioAngleSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getAngle() = 0;

    template<class U> inline void setAngleSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getAngleSensor(bool poll = false);
};

// Position Sensor Attachment Interface
class HelioPositionSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPosition() = 0;

    template<class U> inline void setPositionSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPositionSensor(bool poll = false);
};

// Power Production Sensor Attachment Interface
class HelioPowerProductionSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPowerProduction() = 0;

    template<class U> inline void setPowerProductionSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPowerProductionSensor(bool poll = false);
};

// Power Usage Sensor Attachment Interface
class HelioPowerUsageSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPowerUsage() = 0;

    template<class U> inline void setPowerUsageSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPowerUsageSensor(bool poll = false);
};

// Speed Sensor Attachment Interface
class HelioSpeedSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getSpeed() = 0;

    template<class U> inline void setSpeedSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getSpeedSensor(bool poll = false);
};

// Temperature Sensor Attachment Interface
class HelioTemperatureSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getTemperature() = 0;

    template<class U> inline void setTemperatureSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getTemperatureSensor(bool poll = false);
};


// Minimum Travel Trigger Attachment Interface
class HelioMinTravelTriggerAttachmentInterface {
public:
    virtual HelioTriggerAttachment &getMinTravel() = 0;

    template<class U> inline void setMinTravelTrigger(U trigger);
    template<class U = HelioTrigger> inline SharedPtr<U> getMinTravelTrigger(bool poll = false);
};

// Maximum Travel Trigger Attachment Interface
class HelioMaxTravelTriggerAttachmentInterface {
public:
    virtual HelioTriggerAttachment &getMaxTravel() = 0;

    template<class U> inline void setMaxTravelTrigger(U trigger);
    template<class U = HelioTrigger> inline SharedPtr<U> getMaxTravelTrigger(bool poll = false);
};

// Limit Trigger Attachment Interface
class HelioLimitTriggerAttachmentInterface {
public:
    virtual HelioTriggerAttachment &getLimit() = 0;

    template<class U> inline void setLimitTrigger(U trigger);
    template<class U = HelioTrigger> inline SharedPtr<U> getLimitTrigger(bool poll = false);
};

#endif // /ifndef HelioInterfaces_H

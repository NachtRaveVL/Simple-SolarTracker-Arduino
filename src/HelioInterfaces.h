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

class HelioAngleUnitsInterfaceStorage;
class HelioDistanceUnitsInterfaceStorage;
class HelioMeasurementUnitsInterface;
template <size_t N = 1> class HelioMeasurementUnitsStorage;
class HelioMeasurementUnitsInterfaceStorageSingle;
class HelioMeasurementUnitsInterfaceStorageDouble;
class HelioMeasurementUnitsInterfaceStorageTriple;
class HelioPowerUnitsInterfaceStorage;
class HelioTemperatureUnitsInterfaceStorage;

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

class HelioSensorAttachmentInterface;
class HelioAngleSensorAttachmentInterface;
class HelioPositionSensorAttachmentInterface;
class HelioPowerProductionSensorAttachmentInterface;
class HelioPowerUsageSensorAttachmentInterface;
class HelioSpeedSensorAttachmentInterface;
class HelioTemperatureSensorAttachmentInterface;
class HelioWindSpeedSensorAttachmentInterface;

class HelioTriggerAttachmentInterface;
class HelioMinimumTriggerAttachmentInterface;
class HelioMaximumTriggerAttachmentInterface;
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
    virtual bool begin() = 0;

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
class HelioAngleUnitsInterfaceStorage {
public:
    virtual void setAngleUnits(Helio_UnitsType angleUnits) = 0;
    inline Helio_UnitsType getAngleUnits() const { return _angleUnits; }

protected:
    Helio_UnitsType _angleUnits;
    inline HelioAngleUnitsInterfaceStorage(Helio_UnitsType angleUnits = Helio_UnitsType_Undefined) : _angleUnits(angleUnits) { ; }
};

// Distance Units Interface + Storage
class HelioDistanceUnitsInterfaceStorage {
public:
    virtual void setDistanceUnits(Helio_UnitsType distanceUnits) = 0;
    inline Helio_UnitsType getDistanceUnits() const { return _distUnits; }
    inline void setSpeedUnits(Helio_UnitsType speedUnits);
    inline Helio_UnitsType getSpeedUnits() const;

protected:
    Helio_UnitsType _distUnits;
    inline HelioDistanceUnitsInterfaceStorage(Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) : _distUnits(distanceUnits) { ; }
};

// Measure Units Interface
// Uses virtual getMeasurementUnits() so that units can be local or shadowed
class HelioMeasurementUnitsInterface {
public:
    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t measurementRow = 0) = 0;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t measurementRow = 0) const = 0;

    inline Helio_UnitsType getRateUnits(uint8_t measurementRow = 0) const;
    inline Helio_UnitsType getBaseUnits(uint8_t measurementRow = 0) const;
};

// Measure Units Storage
template <size_t N> class HelioMeasurementUnitsStorage {
protected:
    Helio_UnitsType _measurementUnits[N];
    inline HelioMeasurementUnitsStorage(Helio_UnitsType measurementUnits = Helio_UnitsType_Undefined) { for (hposi_t i = 0; i < N; ++i) { _measurementUnits[i] = measurementUnits; } }
};

// Single Measure Units Interface + Storage
class HelioMeasurementUnitsInterfaceStorageSingle : public HelioMeasurementUnitsInterface, public HelioMeasurementUnitsStorage<1> {
protected:
    inline HelioMeasurementUnitsInterfaceStorageSingle(Helio_UnitsType measurementUnits = Helio_UnitsType_Undefined) : HelioMeasurementUnitsStorage<1>(measurementUnits) { ; }
};

// Double Measure Units Interface + Storage
class HelioMeasurementUnitsInterfaceStorageDouble : public HelioMeasurementUnitsInterface, public HelioMeasurementUnitsStorage<2> {
protected:
    inline HelioMeasurementUnitsInterfaceStorageDouble(Helio_UnitsType measurementUnits = Helio_UnitsType_Undefined) : HelioMeasurementUnitsStorage<2>(measurementUnits) { ; }
};

// Triple Measure Units Interface + Storage
class HelioMeasurementUnitsInterfaceStorageTriple : public HelioMeasurementUnitsInterface, public HelioMeasurementUnitsStorage<3> {
protected:
    inline HelioMeasurementUnitsInterfaceStorageTriple(Helio_UnitsType measurementUnits = Helio_UnitsType_Undefined) : HelioMeasurementUnitsStorage<3>(measurementUnits) { ; }
};

// Power Units Interface + Storage
class HelioPowerUnitsInterfaceStorage {
public:
    virtual void setPowerUnits(Helio_UnitsType powerUnits) = 0;
    inline Helio_UnitsType getPowerUnits() const { return _powerUnits; }

protected:
    Helio_UnitsType _powerUnits;
    inline HelioPowerUnitsInterfaceStorage(Helio_UnitsType powerUnits = Helio_UnitsType_Undefined) : _powerUnits(powerUnits) { ; }
};

// Temperature Units Interface + Storage
class HelioTemperatureUnitsInterfaceStorage {
public:
    virtual void setTemperatureUnits(Helio_UnitsType temperatureUnits) = 0;
    inline Helio_UnitsType getTemperatureUnits() const { return _tempUnits; }

protected:
    Helio_UnitsType _tempUnits;
    inline HelioTemperatureUnitsInterfaceStorage(Helio_UnitsType temperatureUnits = Helio_UnitsType_Undefined) : _tempUnits(temperatureUnits) { ; }
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
};

// Sensor Object Interface
class HelioSensorObjectInterface {
public:
    virtual bool takeMeasurement(bool force = false) = 0;
    virtual const HelioMeasurement *getMeasurement(bool poll = false) = 0;
    virtual bool isTakingMeasurement() const = 0;
    virtual bool needsPolling(hframe_t allowance = 0) const = 0;
};

// Panel Object Interface
class HelioPanelObjectInterface {
public:
    virtual bool canActivate(HelioActuator *actuator) = 0;
    virtual bool isDaylight(bool poll = false) = 0;
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
    virtual float getMaxTargetOffset(bool poll = false) = 0;
    virtual Helio_DrivingState getDrivingState(bool poll = false) = 0;
    inline bool isAligned(bool poll = false) { return getDrivingState(poll) == Helio_DrivingState_AlignedTarget; }
};

// Trigger Object Interface
class HelioTriggerObjectInterface {
public:
    virtual Helio_TriggerState getTriggerState(bool poll = false) = 0;
    inline bool isTriggered(bool poll = false) { return getTriggerState(poll) == Helio_TriggerState_Triggered; }
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

    virtual bool isMinTravel(bool poll = false) = 0;
    virtual bool isMaxTravel(bool poll = false) = 0;

protected:
    virtual void handleTravelTime(millis_t time) = 0;
};


// Parent Actuator Attachment Interface
class HelioParentActuatorAttachmentInterface {
public:
    virtual HelioAttachment &getParentActuatorAttachment() = 0;

    template<class U> inline void setParentActuator(U actuator);
    template<class U = HelioActuator> inline SharedPtr<U> getParentActuator();
};

// Parent Sensor Attachment Interface
class HelioParentSensorAttachmentInterface {
public:
    virtual HelioAttachment &getParentSensorAttachment() = 0;

    template<class U> inline void setParentSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getParentSensor();
};

// Parent Panel Attachment Interface
class HelioParentPanelAttachmentInterface {
public:
    virtual HelioAttachment &getParentPanelAttachment() = 0;

    template<class U> inline void setParentPanel(U panel, hposi_t axisIndex = 0);
    template<class U = HelioPanel> inline SharedPtr<U> getParentPanel();
};

// Parent Rail Attachment Interface
class HelioParentRailAttachmentInterface {
public:
    virtual HelioAttachment &getParentRailAttachment() = 0;

    template<class U> inline void setParentRail(U rail);
    template<class U = HelioRail> inline SharedPtr<U> getParentRail();
};


// Abstract Sensor Attachment Interface
class HelioSensorAttachmentInterface {
    virtual HelioSensorAttachment &getSensorAttachment() = 0;

    template<class U> inline void setSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getSensor(bool poll = false);
};

// Angle Sensor Attachment Interface
class HelioAngleSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getAngleSensorAttachment() = 0;

    template<class U> inline void setAngleSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getAngleSensor(bool poll = false);
};

// Position Sensor Attachment Interface
class HelioPositionSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPositionSensorAttachment() = 0;

    template<class U> inline void setPositionSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPositionSensor(bool poll = false);
};

// Power Production Sensor Attachment Interface
class HelioPowerProductionSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPowerProductionSensorAttachment() = 0;

    template<class U> inline void setPowerProductionSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPowerProductionSensor(bool poll = false);
};

// Power Usage Sensor Attachment Interface
class HelioPowerUsageSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getPowerUsageSensorAttachment() = 0;

    template<class U> inline void setPowerUsageSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getPowerUsageSensor(bool poll = false);
};

// Speed Sensor Attachment Interface
class HelioSpeedSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getSpeedSensorAttachment() = 0;

    template<class U> inline void setSpeedSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getSpeedSensor(bool poll = false);
};

// Temperature Sensor Attachment Interface
class HelioTemperatureSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getTemperatureSensorAttachment() = 0;

    template<class U> inline void setTemperatureSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getTemperatureSensor(bool poll = false);
};

// Wind Speed Sensor Attachment Interface
class HelioWindSpeedSensorAttachmentInterface {
public:
    virtual HelioSensorAttachment &getWindSpeedSensorAttachment() = 0;

    template<class U> inline void setWindSpeedSensor(U sensor);
    template<class U = HelioSensor> inline SharedPtr<U> getWindSpeedSensor(bool poll = false);
};


// Abstract Trigger Attachment Interface
class HelioTriggerAttachmentInterface {
    virtual HelioTriggerAttachment &getTriggerAttachment() = 0;

    template<class U> inline void setTrigger(U trigger);
    template<class U = HelioTrigger> inline SharedPtr<U> getTrigger(bool poll = false);
};

// Minimum Trigger Attachment Interface
class HelioMinimumTriggerAttachmentInterface {
public:
    virtual HelioTriggerAttachment &getMinimumTriggerAttachment() = 0;

    template<class U> inline void setMinimumTrigger(U trigger);
    template<class U = HelioTrigger> inline SharedPtr<U> getMinimumTrigger(bool poll = false);
};

// Maximum Trigger Attachment Interface
class HelioMaximumTriggerAttachmentInterface {
public:
    virtual HelioTriggerAttachment &getMaximumTriggerAttachment() = 0;

    template<class U> inline void setMaximumTrigger(U trigger);
    template<class U = HelioTrigger> inline SharedPtr<U> getMaximumTrigger(bool poll = false);
};

// Limit Trigger Attachment Interface
class HelioLimitTriggerAttachmentInterface {
public:
    virtual HelioTriggerAttachment &getLimitTriggerAttachment() = 0;

    template<class U> inline void setLimitTrigger(U trigger);
    template<class U = HelioTrigger> inline SharedPtr<U> getLimitTrigger(bool poll = false);
};

#endif // /ifndef HelioInterfaces_H

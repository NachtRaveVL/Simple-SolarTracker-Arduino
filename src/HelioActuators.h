/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Actuators
*/

#ifndef HelioActuators_H
#define HelioActuators_H

class HelioActuator;
class HelioRelayActuator;
class HelioRelayMotorActuator;
class HelioVariableActuator;
//class HelioVariableMotorActuator;

struct HelioActuatorData;
struct HelioMotorActuatorData;

#include "Helioduino.h"
#include "HelioDatas.h"
#include "HelioActivation.h"

// Creates actuator object from passed actuator data (return ownership transfer - user code *must* delete returned object)
extern HelioActuator *newActuatorObjectFromData(const HelioActuatorData *dataIn);


// Actuator Base
// This is the base class for all actuators, which defines how the actuator is identified,
// where it lives, and what it's attached to.
class HelioActuator : public HelioObject,
                      public HelioActuatorObjectInterface,
                      public HelioParentRailAttachmentInterface,
                      public HelioParentPanelAttachmentInterface {
public:
    const enum : signed char { Relay, RelayMotor, Variable, VariableMotor, Unknown = -1 } classType; // Actuator class type (custom RTTI)
    inline bool isRelayClass() const { return classType == Relay; }
    inline bool isRelayMotorClass() const { return classType == RelayMotor; }
    inline bool isVariableClass() const { return classType == Variable; }
    inline bool isVariableMotorClass() const { return classType == VariableMotor; }
    inline bool isAnyBinaryClass() const { return isRelayClass() || isRelayMotorClass(); }
    inline bool isAnyVariableClass() const { return isVariableClass() || isVariableMotorClass(); }
    inline bool isAnyMotorClass() const { return isRelayMotorClass() || isVariableMotorClass(); }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioActuator(Helio_ActuatorType actuatorType,
                  hposi_t actuatorIndex,
                  int classType = Unknown);
    HelioActuator(const HelioActuatorData *dataIn);

    virtual void update() override;

    virtual bool getCanEnable() override;

    // Activating actuators is done through activation handles, which must stay memory
    // resident in order for the actuator to pick up and process it. Enablement mode
    // affects how handles are processed - in parallel, or in serial - and what the
    // applied output is. See HelioActuatorAttachment for an abstraction of this process.
    inline HelioActivationHandle enableActuator(Helio_DirectionMode direction, float intensity = 1.0f, millis_t duration = -1, bool force = false) { return HelioActivationHandle(::getSharedPtr<HelioActuator>(this), direction, intensity, duration, force); }
    inline HelioActivationHandle enableActuator(float value, millis_t duration = -1, bool force = false) { return enableActuator(Helio_DirectionMode_Forward, calibrationInvTransform(value), duration, force); }
    inline HelioActivationHandle enableActuator(millis_t duration = -1, bool force = false) { return enableActuator(Helio_DirectionMode_Forward, 1.0f, duration, force); }

    inline void setEnableMode(Helio_EnableMode enableMode) { _enableMode = enableMode; setNeedsUpdate(); bumpRevisionIfNeeded(); }
    inline Helio_EnableMode getEnableMode() const { return _enableMode; }

    inline bool isSerialMode() const { return getActuatorIsSerialFromMode(getEnableMode()); }
    inline bool isMotorType() const { return getActuatorIsMotorFromType(getActuatorType()) || isAnyMotorClass(); }
    inline bool isServoType() const { return getActuatorIsServoFromType(getActuatorType()); }
    inline bool isTravelType() const { return isMotorType() || isServoType(); }

    virtual void setContinuousPowerUsage(HelioSingleMeasurement contPowerUsage) override;
    virtual const HelioSingleMeasurement &getContinuousPowerUsage() override;

    virtual HelioAttachment &getParentRailAttachment() override;
    virtual HelioAttachment &getParentPanelAttachment() override;

    void setUserCalibrationData(HelioCalibrationData *userCalibrationData);
    inline const HelioCalibrationData *getUserCalibrationData() const { return _calibrationData; }

    virtual Pair<float,float> getTravelRange() const;

    // Transformation methods that convert from normalized driving intensity/driver value to calibration units
    inline float calibrationTransform(float value) const { return _calibrationData ? _calibrationData->transform(value) : value; }
    inline void calibrationTransform(float *valueInOut, Helio_UnitsType *unitsOut = nullptr) const { if (valueInOut && _calibrationData) { _calibrationData->transform(valueInOut, unitsOut); } }
    inline HelioSingleMeasurement calibrationTransform(HelioSingleMeasurement measurement) { return _calibrationData ? HelioSingleMeasurement(_calibrationData->transform(measurement.value), _calibrationData->calibrationUnits, measurement.timestamp, measurement.frame) : measurement; }
    inline void calibrationTransform(HelioSingleMeasurement *measurementInOut) const { if (measurementInOut && _calibrationData) { _calibrationData->transform(&measurementInOut->value, &measurementInOut->units); } }

    // Transformation methods that convert from calibration units to normalized driving intensity/driver value
    inline float calibrationInvTransform(float value) const { return _calibrationData ? _calibrationData->inverseTransform(value) : value; }
    inline void calibrationInvTransform(float *valueInOut, Helio_UnitsType *unitsOut = nullptr) const { if (valueInOut && _calibrationData) { _calibrationData->inverseTransform(valueInOut, unitsOut); } }
    inline HelioSingleMeasurement calibrationInvTransform(HelioSingleMeasurement measurement) { return _calibrationData ? HelioSingleMeasurement(_calibrationData->inverseTransform(measurement.value), _calibrationData->calibrationUnits, measurement.timestamp, measurement.frame) : measurement; }
    inline void calibrationInvTransform(HelioSingleMeasurement *measurementInOut) const { if (measurementInOut && _calibrationData) { _calibrationData->inverseTransform(&measurementInOut->value, &measurementInOut->units); } }

    inline float getCalibratedValue() const { return calibrationTransform(getDriveIntensity()); }

    inline Helio_ActuatorType getActuatorType() const { return _id.objTypeAs.actuatorType; }
    inline hposi_t getActuatorIndex() const { return _id.posIndex; }

    inline void setNeedsUpdate() { _needsUpdate = true; }
    inline bool needsUpdate() { return _needsUpdate; }

    Signal<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS> &getActivationSignal();

protected:
    bool _enabled;                                          // Enabled state flag
    bool _needsUpdate;                                      // Stale flag for handle updates
    Helio_EnableMode _enableMode;                           // Handle activation mode
    Vector<HelioActivationHandle *> _handles;               // Activation handles array
    HelioSingleMeasurement _contPowerUsage;                 // Continuous power draw
    HelioAttachment _parentRail;                            // Parent power rail attachment
    HelioAttachment _parentPanel;                           // Parent solar panel attachment
    const HelioCalibrationData *_calibrationData;           // Calibration data
    Signal<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS> _activateSignal; // Activation update signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    virtual void handleActivation();

    friend struct HelioActivationHandle;
};


// Binary Relay Actuator
// This actuator acts as a standard on/off switch, typically paired with a variety of
// different equipment from lights to heaters.
class HelioRelayActuator : public HelioActuator {
public:
    HelioRelayActuator(Helio_ActuatorType actuatorType,
                       hposi_t actuatorIndex,
                       HelioDigitalPin outputPin,
                       int classType = Relay);
    HelioRelayActuator(const HelioActuatorData *dataIn);
    virtual ~HelioRelayActuator();

    virtual bool getCanEnable() override;
    virtual float getDriveIntensity() const override;
    virtual bool isEnabled(float tolerance = 0.0f) const override;

    inline HelioActivationHandle enableActuator(float value, millis_t duration = -1, bool force = false) { return HelioActuator::enableActuator(calibrationInvTransform(value) > FLT_EPSILON ? Helio_DirectionMode_Forward : calibrationInvTransform(value) < -FLT_EPSILON ? Helio_DirectionMode_Reverse : Helio_DirectionMode_Stop, fabsf(calibrationInvTransform(value)), duration, force); }

    inline const HelioDigitalPin &getOutputPin() const { return _outputPin; }

protected:
    HelioDigitalPin _outputPin;                             // Digital output pin

    virtual void saveToData(HelioData *dataOut) override;

    virtual void _enableActuator(float intensity = 1.0) override;
    virtual void _disableActuator() override;
};


// Relay Motor Actuator
// This actuator acts as a motor and can control movement of attached objects. Motors using
// this class are either forward/stop/reverse and do not contain any variable speed control,
// but can be paired with a speed sensor for more precise running calculations.
class HelioRelayMotorActuator : public HelioRelayActuator,
                                public HelioMotorObjectInterface,
                                public HelioDistanceUnitsInterfaceStorage,
                                public HelioPositionSensorAttachmentInterface,
                                public HelioSpeedSensorAttachmentInterface,
                                public HelioMinimumTriggerAttachmentInterface,
                                public HelioMaximumTriggerAttachmentInterface {
public:
    HelioRelayMotorActuator(Helio_ActuatorType actuatorType,
                            hposi_t actuatorIndex,
                            HelioDigitalPin outputPinA,
                            HelioDigitalPin outputPinB,
                            Pair<float,float> travelRange = make_pair(0.0f, FLT_UNDEF),
                            int classType = RelayMotor);
    HelioRelayMotorActuator(const HelioMotorActuatorData *dataIn);

    virtual void update() override;
    virtual SharedPtr<HelioObjInterface> getSharedPtrFor(const HelioObjInterface *obj) const override;

    virtual bool getCanEnable() override;
    virtual float getDriveIntensity() const override;

    virtual bool canTravel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) override;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) override;
    virtual bool canTravel(Helio_DirectionMode direction, millis_t time) override;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, millis_t time) override;

    virtual void setDistanceUnits(Helio_UnitsType distanceUnits) override;
    inline void setSpeedUnits(Helio_UnitsType speedUnits) { setDistanceUnits(baseUnits(speedUnits)); }

    virtual void setContinuousSpeed(HelioSingleMeasurement contSpeed) override;
    virtual const HelioSingleMeasurement &getContinuousSpeed() override;

    virtual Pair<float,float> getTravelRange() const override;
    virtual bool isMinTravel(bool poll = false) override;
    virtual bool isMaxTravel(bool poll = false) override;

    virtual HelioSensorAttachment &getPositionSensorAttachment() override;

    virtual HelioSensorAttachment &getSpeedSensorAttachment() override;

    virtual HelioTriggerAttachment &getMinimumTriggerAttachment() override;

    virtual HelioTriggerAttachment &getMaximumTriggerAttachment() override;

protected:
    HelioDigitalPin _outputPin2;                            // Digital output pin 2 (reverse/H-bridge pin B)
    float _intensity;                                       // Current set intensity
    HelioSingleMeasurement _contSpeed;                      // Continuous speed (dist units per min)
    HelioSensorAttachment _position;                        // Position sensor attachment
    HelioSensorAttachment _speed;                           // Speed rate sensor attachment
    HelioTriggerAttachment _minimum;                        // Minimum travel trigger attachment
    HelioTriggerAttachment _maximum;                        // Maximum travel trigger attachment
    Pair<float,float> _travelRange;                         // Travel range, in distance units
    float _travelPosStart;                                  // Start for travel distance
    float _travelDistAccum;                                 // Accumulator for total distance traveled
    millis_t _travelTimeStart;                              // Time millis motor was activated at
    millis_t _travelTimeAccum;                              // Time millis motor has been accumulated up to

    virtual void saveToData(HelioData *dataOut) override;

    virtual void _enableActuator(float intensity = 1.0) override;
    virtual void _disableActuator() override;
    virtual void handleActivation() override;

    virtual void handleTravelTime(millis_t time) override;

    virtual void handleMinimumTrigger(Helio_TriggerState minimumTrigger);
    virtual void handleMaximumTrigger(Helio_TriggerState maximumTrigger);
};


// Variable Actuator
// This actuator acts as a simple variable ranged dial, typically paired with a variety of
// different equipment that allows analog throttle or position control.
class HelioVariableActuator : public HelioActuator {
public:
    HelioVariableActuator(Helio_ActuatorType actuatorType,
                          hposi_t actuatorIndex,
                          HelioAnalogPin outputPin,
                          int classType = Variable);
    HelioVariableActuator(const HelioActuatorData *dataIn);
    virtual ~HelioVariableActuator();

    virtual bool getCanEnable() override;
    virtual float getDriveIntensity() const override;
    virtual bool isEnabled(float tolerance = 0.0f) const override;

    inline int getDriveIntensity_raw() const { return _outputPin.bitRes.inverseTransform(_intensity); }

    inline const HelioAnalogPin &getOutputPin() const { return _outputPin; }

protected:
    HelioAnalogPin _outputPin;                              // Analog output pin
    float _intensity;                                       // Current set intensity

    virtual void saveToData(HelioData *dataOut) override;

    virtual void _enableActuator(float intensity = 1.0) override;
    virtual void _disableActuator() override;
};


// Variable/Throttled Motor Actuator
// This actuator acts as a throttleable motor and can control speed & displacement.
// Motors using this class have variable motor control but also can be paired with
// a speed sensor for more precise running calculations.
//class HelioVariableMotorActuator : public HelioVariableActuator, public HelioMotorObjectInterface, public HelioSpeedSensorAttachmentInterface {
// TODO: #9 in Helioduino.
//};


// Actuator Serialization Data
struct HelioActuatorData : public HelioObjectData
{
    HelioPinData outputPin;                                 // Output pin (1/A)
    Helio_EnableMode enableMode;                            // Activation enablement mode
    HelioMeasurementData contPowerUsage;                    // Continuous power usage
    char railName[HELIO_NAME_MAXSIZE];                      // Parent rail
    char panelName[HELIO_NAME_MAXSIZE];                     // Parent reservoir

    HelioActuatorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Motor Actuator Serialization Data
struct HelioMotorActuatorData : public HelioActuatorData
{
    HelioPinData outputPin2;                                // Output pin 2/B
    float travelRange[2];                                   // Travel range maximums
    Helio_UnitsType distanceUnits;                          // Distance units
    HelioMeasurementData contSpeed;                         // Continuous speed
    char positionSensor[HELIO_NAME_MAXSIZE];                // Position sensor
    char speedSensor[HELIO_NAME_MAXSIZE];                   // Speed sensor
    HelioTriggerSubData minTrigger;                         // Minimum travel trigger
    HelioTriggerSubData maxTrigger;                         // Maximum travel trigger

    HelioMotorActuatorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioActuators_H

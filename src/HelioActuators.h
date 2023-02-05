/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Actuators
*/

#ifndef HelioActuators_H
#define HelioActuators_H

struct HelioActivationHandle;

class HelioActuator;
class HelioRelayActuator;
class HelioRelayMotorActuator;
class HelioVariableActuator;
//class HelioVariableMotorActuator;

struct HelioActuatorData;
struct HelioMotorActuatorData;

#include "Helioduino.h"

// Creates actuator object from passed actuator data (return ownership transfer - user code *must* delete returned object)
extern HelioActuator *newActuatorObjectFromData(const HelioActuatorData *dataIn);


// Activation Handle
// Since actuators are shared objects, those wishing to enable any actuator must receive
// a valid handle. Actuators may customize how they handle multiple activation handles.
// Handles represent a driving intensity value ranged [0,1] or [-1,1] depending on the
// capabilities of the attached actuator. Handles do not guarantee activation unless their
// forced flag is set (also see Actuator activation signal), but can be set up to ensure
// actuators are enabled for a specified duration, which is able to be async updated.
struct HelioActivationHandle {
    SharedPtr<HelioActuator> actuator;  // Actuator owner, set only when activation requested (use operator= to set)
    float intensity;                    // Normalized driving intensity ([0.0,1.0])
    Helio_DirectionMode direction;      // Normalized driving direction
    millis_t checkTime;                 // Last check timestamp, in milliseconds, else 0 for not started
    millis_t duration;                  // Duration time remaining, in milliseconds, else -1 for non-diminishing/unlimited or 0 for finished
    millis_t elapsed;                   // Elapsed time accumulator, in milliseconds, else 0
    bool forced;                        // If activation should force enable and ignore cursory canEnable checks

    // Handle constructor that specifies a normalized enablement, ranged: normalized [0.0,1.0] for specified direction
    HelioActivationHandle(SharedPtr<HelioActuator> actuator, Helio_DirectionMode direction, float intensity = 1.0f, millis_t duration = -1, bool force = false);

    // Handle constructor that specifies a binary enablement, ranged: (=0=,!0!) for disable/enable or (<=-1,=0=,>=1) for reverse/stop/forward
    inline HelioActivationHandle(SharedPtr<HelioActuator> actuator, int enabled, millis_t duration = -1, bool force = false)
        : HelioActivationHandle(actuator, (enabled > 0 ? Helio_DirectionMode_Forward : enabled < 0 ? Helio_DirectionMode_Reverse : Helio_DirectionMode_Stop), (enabled ? 1.0f : 0.0f), duration, force) { ; }
    // Handle constructor that specifies a variable intensity enablement, ranged: [=0.0=,<=1.0] for disable/enable or [-1.0=>,=0.0=,<=1.0] for reverse/stop/forward
    inline HelioActivationHandle(SharedPtr<HelioActuator> actuator, float intensity, millis_t duration = -1, bool force = false)
        : HelioActivationHandle(actuator, (intensity > FLT_EPSILON ? Helio_DirectionMode_Forward : intensity < -FLT_EPSILON ? Helio_DirectionMode_Reverse : Helio_DirectionMode_Stop), fabsf(intensity), duration, force)  { ; }

    // Default constructor for empty handles
    inline HelioActivationHandle() : HelioActivationHandle(nullptr, Helio_DirectionMode_Undefined, 0.0f, 0, false) { ; }
    HelioActivationHandle(const HelioActivationHandle &handle);
    ~HelioActivationHandle();
    HelioActivationHandle &operator=(const HelioActivationHandle &handle);
    HelioActivationHandle &operator=(SharedPtr<HelioActuator> actuator);

    // Disconnects activation from an actuator (un-registers self from actuator)
    void unset();

    // Elapses time by delta, updating relevant operating values if needed
    void elapseBy(millis_t delta);

    inline bool isActive() const { return actuator && checkTime > 0; }
    inline bool isDone() const { return duration == 0; }
    inline bool isInfinite() const { return duration == -1; }
    inline millis_t getTimeElapsed(millis_t time = millis()) const { return isActive() ? (time - checkTime) + elapsed : elapsed; }

    // De-normalized driving intensity value [-1.0,1.0]
    inline float getDriveIntensity() const { return direction == Helio_DirectionMode_Forward ? intensity :
                                                    direction == Helio_DirectionMode_Reverse ? -intensity : 0.0f; }
};


// Actuator Base
// This is the base class for all actuators, which defines how the actuator is identified,
// where it lives, and what it's attached to.
class HelioActuator : public HelioObject, public HelioActuatorObjectInterface, public HelioRailAttachmentInterface, public HelioPanelAttachmentInterface {
public:
    const enum : signed char { Relay, RelayMotor, Variable, VariableMotor, Unknown = -1 } classType; // Actuator class type (custom RTTI)
    inline bool isRelayClass() const { return classType == Relay; }
    inline bool isRelayMotorClass() const { return classType == RelayMotor; }
    inline bool isVariableClass() const { return classType == Variable; }
    inline bool isVariableMotorClass() const { return classType == VariableMotor; }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioActuator(Helio_ActuatorType actuatorType,
                  hposi_t actuatorIndex,
                  int classType = Unknown);
    HelioActuator(const HelioActuatorData *dataIn);

    virtual void update() override;

    virtual bool getCanEnable() override;

    inline HelioActivationHandle enableActuator(Helio_DirectionMode direction, float intensity = 1.0f, millis_t duration = -1, bool force = false) { return HelioActivationHandle(::getSharedPtr<HelioActuator>(this), direction, intensity, duration, force); }
    inline HelioActivationHandle enableActuator(float intensity, millis_t duration = -1, bool force = false) { return HelioActivationHandle(::getSharedPtr<HelioActuator>(this), intensity, duration, force); }
    inline HelioActivationHandle enableActuator(millis_t duration, bool force = false) { return HelioActivationHandle(::getSharedPtr<HelioActuator>(this), 1, duration, force); }
    inline HelioActivationHandle enableActuator(bool force, millis_t duration = -1) { return HelioActivationHandle(::getSharedPtr<HelioActuator>(this), 1, duration, force); }

    inline void setEnableMode(Helio_EnableMode enableMode) { _enableMode = enableMode; setNeedsUpdate(); }
    inline Helio_EnableMode getEnableMode() { return _enableMode; }

    virtual void setContinuousPowerUsage(HelioSingleMeasurement contPowerUsage) override;
    virtual const HelioSingleMeasurement &getContinuousPowerUsage() override;

    virtual HelioAttachment &getParentRail(bool resolve = true) override;
    virtual HelioAttachment &getParentPanel(bool resolve = true) override;

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
    HelioAttachment _rail;                                  // Power rail attachment
    HelioAttachment _panel;                                 // Panel attachment
    Signal<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS> _activateSignal; // Activation update signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    virtual void handleActivation() override;

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
    virtual float getDriveIntensity() override;
    virtual bool isEnabled(float tolerance = 0.0f) const override;

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
class HelioRelayMotorActuator : public HelioRelayActuator, public HelioMotorObjectInterface, public HelioPositionSensorAttachmentInterface, public HelioSpeedSensorAttachmentInterface {
public:
    HelioRelayMotorActuator(Helio_ActuatorType actuatorType,
                            hposi_t actuatorIndex,
                            HelioDigitalPin forwardOutputPin,
                            HelioDigitalPin reverseOutputPin,
                            int classType = RelayMotor);
    HelioRelayMotorActuator(const HelioMotorActuatorData *dataIn);

    virtual void update() override;

    virtual bool getCanEnable() override;
    virtual float getDriveIntensity() override;

    virtual bool canTravel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) override;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) override;
    virtual bool canTravel(Helio_DirectionMode direction, millis_t time) override;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, millis_t time) override;

    virtual void setDistanceUnits(Helio_UnitsType distanceUnits) override;
    virtual Helio_UnitsType getDistanceUnits() const override;
    virtual void setSpeedUnits(Helio_UnitsType speedUnits) override;
    virtual Helio_UnitsType getSpeedUnits() const override;

    virtual void setContinuousSpeed(HelioSingleMeasurement contSpeed) override;
    virtual const HelioSingleMeasurement &getContinuousSpeed() override;

    virtual HelioSensorAttachment &getPosition(bool poll = false) override;

    virtual HelioSensorAttachment &getSpeed(bool poll = false) override;

protected:
    HelioDigitalPin _outputPin2;                            // Digital output pin 2 (reverse/H-bridge pin B)
    float _intensity;                                       // Current set intensity
    Helio_UnitsType _distanceUnits;                         // Distance units preferred
    Helio_UnitsType _speedUnits;                            // Speed units preferred
    HelioSingleMeasurement _contSpeed;                      // Continuous speed
    HelioSensorAttachment _position;                        // Position sensor attachment
    HelioSensorAttachment _speed;                           // Speed rate sensor attachment
    float _travelPositionStart;                             // Start for travel distance
    float _travelDistanceAccum;                             // Accumulator for total distance traveled
    millis_t _travelTimeStart;                              // Time millis motor was activated at
    millis_t _travelTimeAccum;                              // Time millis motor has been accumulated up to

    virtual void saveToData(HelioData *dataOut) override;

    virtual void _enableActuator(float intensity = 1.0) override;
    virtual void _disableActuator() override;
    virtual void handleActivation() override;

    virtual void pollTravelingSensors() override;
    virtual void handleTravelTime(millis_t time) override;
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
    virtual float getDriveIntensity() override;
    virtual bool isEnabled(float tolerance = 0.0f) const override;

    inline int getDriveIntensity_raw() const { return _outputPin.bitRes.inverseTransform(_intensity); }

    inline const HelioAnalogPin &getOutputPin() const { return _outputPin; }

protected:
    HelioAnalogPin _outputPin;                              // Analog output pin
    float _intensity;                                       // Current set intensity

    virtual void saveToData(HelioData *dataOut) override;

    virtual void _enableActuator(float intensity = 1.0) override;
    virtual void _disableActuator() override;
    virtual void handleActivation() override;
};


// Throttled Motor Actuator
// This actuator acts as a throttleable motor and can control speed & displacement.
// Motors using this class have variable motor control but also can be paired with
// a speed sensor for more precise running calculations.
//class HelioThrottledMotorActuator : public HelioVariableActuator, public HelioMotorObjectInterface, public HelioSpeedSensorAttachmentInterface {
// TODO
//};


// Actuator Serialization Data
struct HelioActuatorData : public HelioObjectData
{
    HelioPinData outputPin;
    Helio_EnableMode enableMode;
    HelioMeasurementData contPowerUsage;
    char railName[HELIO_NAME_MAXSIZE];
    char panelName[HELIO_NAME_MAXSIZE];

    HelioActuatorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Motor Actuator Serialization Data
struct HelioMotorActuatorData : public HelioActuatorData
{
    HelioPinData outputPin2;
    Helio_UnitsType distanceUnits;
    Helio_UnitsType speedUnits;
    HelioMeasurementData contSpeed;
    char positionSensor[HELIO_NAME_MAXSIZE];
    char speedSensor[HELIO_NAME_MAXSIZE];

    HelioMotorActuatorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioActuators_H

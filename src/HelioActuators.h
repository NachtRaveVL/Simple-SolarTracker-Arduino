/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Actuators
*/

#ifndef HelioActuators_H
#define HelioActuators_H

struct HelioActivationHandle;

class HelioActuator;
class HelioRelayActuator;
class HelioVariableActuator;

struct HelioActuatorData;

#include "Helioduino.h"

// Creates actuator object from passed actuator data (return ownership transfer - user code *must* delete returned object)
extern HelioActuator *newActuatorObjectFromData(const HelioActuatorData *dataIn);


// Activation Handle
// Since actuators are shared objects, those wishing to enable any actuator must receive
// a valid handle. Actuators may customize how they handle multiple activation handles.
// Handles set an intensity value that may be ranged [0,1] or [-1,1] depending on the
// capabilities of the attached actuator. Handles do not guarantee activation unless their
// forced flag is set (also see Actuator activation signal), but can be set up to ensure
// actuators are enabled for a specified duration, which is able to be updated asyncly.
// Finely timed activations should instead consider utilizing ActuatorTimedEnableTask.
struct HelioActivationHandle {
    HelioActuator *actuator;            // Actuator owner (strong)
    float intensity;                    // Intensity (normalized [0.0,1.0])
    Helio_DirectionMode direction;      // Direction setting
    millis_t startMillis;               // Enablement start timestamp, in milliseconds, else 0
    millis_t durationMillis;            // Duration time requested/remaining, in milliseconds, else 0 for unlimited/no expiry
    bool forced;                        // If activation should force enable and ignore cursory canEnable checks

    // Default constructor for empty handles
    inline HelioActivationHandle() : actuator(nullptr), intensity(0.0f), direction(Helio_DirectionMode_Undefined), startMillis(0), durationMillis(0), forced(false) { ; }
    // Handle constructor that specifies a normalized enablement, ranged: normalized [0.0,1.0] for specified direction
    HelioActivationHandle(HelioActuator *actuator, Helio_DirectionMode direction, float intensity = 1.0f, millis_t forMillis = 0, bool force = false);
    // Handle constructor that specifies a binary enablement, ranged: (=0=,!0!) for disable/enable or (<=-1,=0=,>=1) for reverse/stop/forward
    inline HelioActivationHandle(HelioActuator *actuator, int isEnabled, millis_t forMillis = 0, bool force = false)
        : HelioActivationHandle(actuator, (isEnabled > 0 ? Helio_DirectionMode_Forward : isEnabled < 0 ? Helio_DirectionMode_Reverse : Helio_DirectionMode_Stop), (isEnabled ? 1.0f : 0.0f), forMillis, force) { ; }
    // Handle constructor that specifies a variable intensity enablement, ranged: [=0.0=,<=1.0] for disable/enable or [-1.0=>,=0.0=,<=1.0] for reverse/stop/forward
    inline HelioActivationHandle(HelioActuator *actuator, float atIntensity, millis_t forMillis = 0, bool force = false)
        : HelioActivationHandle(actuator, (atIntensity > FLT_EPSILON ? Helio_DirectionMode_Forward : atIntensity < -FLT_EPSILON ? Helio_DirectionMode_Reverse : Helio_DirectionMode_Stop), fabsf(atIntensity), forMillis, force)  { ; }

    HelioActivationHandle(const HelioActivationHandle &handle);
    ~HelioActivationHandle();
    HelioActivationHandle &operator=(const HelioActivationHandle &handle);

    // Driving intensity value ([-1,1] non-normalized, [0,1] normalized)
    inline float getDriveIntensity(bool normalized = false) const { return direction == Helio_DirectionMode_Forward ? intensity :
                                                                           direction == Helio_DirectionMode_Reverse ? (normalized ? intensity : -intensity) : 0.0f; }
};


// Actuator Base
// This is the base class for all actuators, which defines how the actuator is identified,
// where it lives, and what it's attached to.
class HelioActuator : public HelioObject, public HelioActuatorObjectInterface, public HelioRailAttachmentInterface, public HelioPanelAttachmentInterface {
public:
    const enum : signed char { Relay, Variable, Unknown = -1 } classType; // Actuator class type (custom RTTI)
    inline bool isRelayClass() const { return classType == Relay; }
    inline bool isVariableClass() const { return classType == Variable; }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioActuator(Helio_ActuatorType actuatorType,
                  Helio_PositionIndex actuatorIndex,
                  int classType = Unknown);
    HelioActuator(const HelioActuatorData *dataIn);

    virtual void update() override;

    inline HelioActivationHandle enableActuator(Helio_DirectionMode direction, float normIntensity = 1.0f, millis_t forMillis = 0, bool force = false) { return HelioActivationHandle(this, direction, normIntensity, forMillis, force); }
    inline HelioActivationHandle enableActuator(float driveIntensity = 1.0f, millis_t forMillis = 0, bool force = false) { return HelioActivationHandle(this, driveIntensity, forMillis, force); }
    inline HelioActivationHandle enableActuator(float driveIntensity, millis_t forMillis = 0) { return HelioActivationHandle(this, driveIntensity, forMillis); }
    inline HelioActivationHandle enableActuator(float driveIntensity) { return HelioActivationHandle(this, driveIntensity); }
    inline HelioActivationHandle enableActuator(millis_t forMillis, bool force = false) { return HelioActivationHandle(this, 1.0f, forMillis, force); }
    inline HelioActivationHandle enableActuator(bool force, millis_t forMillis = 0) { return HelioActivationHandle(this, 1.0f, forMillis, force); }

    virtual bool getCanEnable() override;
    virtual bool isEnabled(float tolerance = 0.0f) const = 0;

    inline void setEnableMode(Helio_EnableMode enableMode) { _enableMode = enableMode; _needsUpdate = true; }
    inline Helio_EnableMode getEnableMode() { return _enableMode; }

    virtual void setContinuousPowerUsage(float contPowerUsage, Helio_UnitsType contPowerUsageUnits = Helio_UnitsType_Undefined) override;
    virtual void setContinuousPowerUsage(HelioSingleMeasurement contPowerUsage) override;
    virtual const HelioSingleMeasurement &getContinuousPowerUsage() override;

    virtual HelioAttachment &getParentRail(bool resolve = true) override;
    virtual HelioAttachment &getParentPanel(bool resolve = true) override;

    inline Helio_ActuatorType getActuatorType() const { return _id.objTypeAs.actuatorType; }
    inline Helio_PositionIndex getActuatorIndex() const { return _id.posIndex; }

    Signal<HelioActuator *> &getActivationSignal();

protected:
    bool _enabled;                                          // Enabled state flag
    bool _needsUpdate;                                      // Dirty flag for handle updates
    Helio_EnableMode _enableMode;                           // Handle activation mode
    Vector<HelioActivationHandle *> _handles;               // Activation handles array
    HelioSingleMeasurement _contPowerUsage;                 // Continuous power draw
    HelioAttachment _rail;                                  // Power rail attachment
    HelioAttachment _panel;                                 // Panel attachment
    Signal<HelioActuator *> _activateSignal;                // Activation update signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    friend struct HelioActivationHandle;
};


// Relay-based Binary Actuator
// This actuator acts as a standard on/off switch, typically paired with a variety of
// different equipment from pumps to grow lights and heaters.
class HelioRelayActuator : public HelioActuator {
public:
    HelioRelayActuator(Helio_ActuatorType actuatorType,
                       Helio_PositionIndex actuatorIndex,
                       HelioDigitalPin outputPin,
                       int classType = Relay);
    HelioRelayActuator(const HelioActuatorData *dataIn);
    virtual ~HelioRelayActuator();

    virtual bool getCanEnable() override;
    virtual bool isEnabled(float tolerance = 0.0f) const override;

    inline const HelioDigitalPin &getOutputPin() const { return _outputPin; }

protected:
    HelioDigitalPin _outputPin;                             // Digital output pin

    virtual void saveToData(HelioData *dataOut) override;

    virtual bool _enableActuator(float intensity = 1.0) override;
    virtual void _disableActuator() override;
};


// Variable Actuator
// This actuator acts as a variable range dial, typically paired with a device that supports
// throttling of some kind, such as a powered exhaust fan, or variable level LEDs.
class HelioVariableActuator : public HelioActuator {
public:
    HelioVariableActuator(Helio_ActuatorType actuatorType,
                          Helio_PositionIndex actuatorIndex,
                          HelioAnalogPin outputPin,
                          int classType = Variable);
    HelioVariableActuator(const HelioActuatorData *dataIn);
    virtual ~HelioVariableActuator();

    virtual bool isEnabled(float tolerance = 0.0f) const override;

    inline float getPWMAmount() const { return _pwmAmount; }
    int getPWMAmount(int) const;
    void setPWMAmount(float amount);
    void setPWMAmount(int amount);

    inline const HelioAnalogPin &getOutputPin() const { return _outputPin; }

protected:
    HelioAnalogPin _outputPin;                              // Analog output pin
    float _pwmAmount;                                       // Current set PWM amount

    virtual void saveToData(HelioData *dataOut) override;

    virtual bool _enableActuator(float intensity = 1.0) override;
    virtual void _disableActuator() override;
};


// Actuator Serialization Data
struct HelioActuatorData : public HelioObjectData
{
    HelioPinData outputPin;
    HelioMeasurementData contPowerUsage;
    char railName[HELIO_NAME_MAXSIZE];
    char panelName[HELIO_NAME_MAXSIZE];
    Helio_EnableMode enableMode;

    HelioActuatorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioActuators_H

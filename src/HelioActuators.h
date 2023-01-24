/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Actuators
*/

#ifndef HelioActuators_H
#define HelioActuators_H

class HelioActuator;
class HelioRelayActuator;
class HelioPWMActuator;

struct HelioActuatorData;

#include "Helioduino.h"

// Creates actuator object from passed actuator data (return ownership transfer - user code *must* delete returned object)
extern HelioActuator *newActuatorObjectFromData(const HelioActuatorData *dataIn);


// Actuator Base
// This is the base class for all actuators, which defines how the actuator is identified,
// where it lives, and what it's attached to.
class HelioActuator : public HelioObject, public HelioActuatorObjectInterface, public HelioRailAttachmentInterface, public HelioPanelAttachmentInterface {
public:
    const enum : signed char { Relay, VariablePWM, Unknown = -1 } classType; // Actuator class type (custom RTTI)
    inline bool isRelayClass() const { return classType == Relay; }
    inline bool isVariablePWMClass() const { return classType == VariablePWM; }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioActuator(Helio_ActuatorType actuatorType,
                  Helio_PositionIndex actuatorIndex,
                  int classType = Unknown);
    HelioActuator(const HelioActuatorData *dataIn);

    virtual void update() override;

    virtual bool enableActuator(float intensity = 1.0f, bool force = false) = 0;
    virtual bool getCanEnable() override;
    virtual bool isEnabled(float tolerance = 0.0f) const = 0;

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
    HelioSingleMeasurement _contPowerUsage;                 // Continuous power draw
    HelioAttachment _rail;                                  // Power rail attachment
    HelioAttachment _panel;                                 // Panel attachment
    Signal<HelioActuator *> _activateSignal;                // Activation update signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;
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

    virtual bool enableActuator(float intensity = 1.0f, bool force = false) override;
    virtual void disableActuator() override;
    virtual bool isEnabled(float tolerance = 0.0f) const override;

    inline const HelioDigitalPin &getOutputPin() const { return _outputPin; }

protected:
    HelioDigitalPin _outputPin;                             // Digital output pin

    virtual void saveToData(HelioData *dataOut) override;
};


// PWM-based Variable Actuator
// This actuator acts as a variable range dial, typically paired with a device that supports
// PWM throttling of some kind, such as a powered exhaust fan, or variable level LEDs.
class HelioPWMActuator : public HelioActuator {
public:
    HelioPWMActuator(Helio_ActuatorType actuatorType,
                     Helio_PositionIndex actuatorIndex,
                     HelioAnalogPin outputPin,
                     int classType = VariablePWM);
    HelioPWMActuator(const HelioActuatorData *dataIn);
    virtual ~HelioPWMActuator();

    virtual bool enableActuator(float intensity = 1.0f, bool force = false) override;
    virtual void disableActuator() override;
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
};


// Actuator Serialization Data
struct HelioActuatorData : public HelioObjectData
{
    HelioPinData outputPin;
    HelioMeasurementData contPowerUsage;
    char railName[HELIO_NAME_MAXSIZE];
    char panelName[HELIO_NAME_MAXSIZE];

    HelioActuatorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioActuators_H

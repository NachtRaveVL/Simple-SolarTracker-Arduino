/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Balancers
*/

#ifndef HelioBalancers_H
#define HelioBalancers_H

class HelioBalancer;
class HelioLinearEdgeBalancer;
class HelioTimedDosingBalancer;

#include "Helioduino.h"
#include "HelioObject.h"
#include "HelioTriggers.h"

// Balancer Base
// This is the base class for all balancers, which are used to modify the external
// environment via increment / decrement actuators that can increase or decrease a
// measured value. Balancers allow for a setpoint to be used to drive such devices.
class HelioBalancer : public HelioSubObject, public HelioBalancerObjectInterface {
public:
    const enum : signed char { LinearEdge, TimedDosing, Unknown = -1 } type; // Balancer type (custom RTTI)
    inline bool isLinearEdgeType() const { return type == LinearEdge; }
    inline bool isTimedDosingType() const { return type == TimedDosing; }
    inline bool isUnknownType() const { return type <= Unknown; }

    HelioBalancer(SharedPtr<HelioSensor> sensor,
                  float targetSetpoint,
                  float targetRange,
                  uint8_t measurementRow = 0,
                  int type = Unknown);
    virtual ~HelioBalancer();

    virtual void update();

    virtual void setTargetSetpoint(float targetSetpoint) override;
    virtual Helio_BalancerState getBalancerState() const override;

    void setTargetUnits(Helio_UnitsType targetUnits) { _sensor.setMeasurementUnits(targetUnits); }
    inline Helio_UnitsType getTargetUnits() const { return _sensor.getMeasurementUnits(); }

    void setIncrementActuators(const Vector<Pair<SharedPtr<HelioActuator>, float>, HELIO_BAL_INCACTUATORS_MAXSIZE> &incActuators);
    void setDecrementActuators(const Vector<Pair<SharedPtr<HelioActuator>, float>, HELIO_BAL_DECACTUATORS_MAXSIZE> &decActuators);
    inline const Vector<Pair<SharedPtr<HelioActuator>, float>, HELIO_BAL_INCACTUATORS_MAXSIZE> &getIncrementActuators() { return _incActuators; }
    inline const Vector<Pair<SharedPtr<HelioActuator>, float>, HELIO_BAL_DECACTUATORS_MAXSIZE> &getDecrementActuators() { return _decActuators; }

    inline void setEnabled(bool enabled) { _enabled = enabled; }
    inline bool isEnabled() const { return _enabled; }

    inline float getTargetSetpoint() const { return _targetSetpoint; }
    inline float getTargetRange() const { return _targetRange; }

    inline SharedPtr<HelioSensor> getSensor(bool poll = false) { _sensor.updateIfNeeded(poll); return _sensor.getObject(); }
    inline uint8_t getMeasurementRow() const { return _sensor.getMeasurementRow(); }

    Signal<Helio_BalancerState, HELIO_BALANCER_STATE_SLOTS> &getBalancerSignal();

protected:
    HelioSensorAttachment _sensor;                          // Sensor attachment
    Helio_BalancerState _balancerState;                     // Current balancer state
    float _targetSetpoint;                                  // Target setpoint value
    float _targetRange;                                     // Target range value
    bool _enabled;                                          // Enabled flag

    Signal<Helio_BalancerState, HELIO_BALANCER_STATE_SLOTS> _balancerSignal; // Balancer signal

    Vector<Pair<SharedPtr<HelioActuator>, float>, HELIO_BAL_INCACTUATORS_MAXSIZE> _incActuators; // Increment actuators
    Vector<Pair<SharedPtr<HelioActuator>, float>, HELIO_BAL_DECACTUATORS_MAXSIZE> _decActuators; // Decrement actuators

    void disableAllActuators();

    void handleMeasurement(const HelioMeasurement *measurement);
};


// Linear Edge Balancer
// A linear edge balancer is a balancer that provides the ability to form high and low
// areas of actuator control either by a vertical edge or a linear-gradient edge that
// interpolates along an edge's length. A vertical edge in this case can be thought of as
// an edge with zero length, which is the default. Useful for fans, heaters, and others.
class HelioLinearEdgeBalancer : public HelioBalancer {
public:
    HelioLinearEdgeBalancer(SharedPtr<HelioSensor> sensor,
                            float targetSetpoint,
                            float targetRange,
                            float edgeOffset = 0,
                            float edgeLength = 0,
                            uint8_t measurementRow = 0);

    virtual void update() override;

    inline float getEdgeOffset() const { return _edgeOffset; }
    inline float getEdgeLength() const { return _edgeLength; }

protected:
    float _edgeOffset;                                      // Edge offset
    float _edgeLength;                                      // Length of edge (0 for non-linear)
};

#endif // /ifndef HelioBalancers_H

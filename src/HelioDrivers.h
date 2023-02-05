/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Drivers
*/

#ifndef HelioDrivers_H
#define HelioDrivers_H

class HelioDriver;
class HelioServoDriver;
class HelioMotorDriver;

#include "Helioduino.h"
#include "HelioObject.h"
#include "HelioTriggers.h"

// Driver Base
// This is the base class for all driver objects, which are used to modify the external
// environment via a set of movement actuators along a specified track. Drivers allow for
// a set-point to be used to drive such tasks, with different drivers specializing the
// manner in which they operate.
class HelioDriver : public HelioSubObject, public HelioDriverObjectInterface {
public:
    const enum : signed char { Servo, Motor, Unknown = -1 } type; // Driver type (custom RTTI)
    inline bool isServoType() const { return type == Servo; }
    inline bool isMotorType() const { return type == Motor; }
    inline bool isUnknownType() const { return type <= Unknown; }

    HelioDriver(float trackMin, float trackMax,
                float targetSetpoint, float travelRate,
                int type = Unknown);
    virtual ~HelioDriver();

    virtual void update();

    virtual void setTargetSetpoint(float targetSetpoint) override;
    virtual void setTravelRate(float travelRate) override;
    virtual Helio_DrivingState getDrivingState() const override;

    virtual void setTargetUnits(Helio_UnitsType targetUnits);
    inline Helio_UnitsType getTargetUnits() const { return definedUnitsElse(_targetUnits, defaultDistanceUnits()); }

    void setActuators(const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &actuators);
    inline const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &getActuators() { return _actuators; }

    inline void setEnabled(bool enabled) { _enabled = enabled; }
    inline bool isEnabled() const { return _enabled; }

    inline float getTrackMinimum() const { return _trackMin; }
    inline float getTrackMaximum() const { return _trackMax; }
    inline float getTargetSetpoint() const { return _targetSetpoint; }
    inline float getTravelRate() const { return _travelRate; }

    Signal<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> &getDrivingSignal();

protected:
    float _trackMin;                                        // Track minimum value
    float _trackMax;                                        // Track maximum value
    float _targetSetpoint;                                  // Target set-point value
    float _travelRate;                                      // Travel rate
    Helio_UnitsType _targetUnits;                           // Target units
    Helio_DrivingState _drivingState;                       // Current driving state
    bool _enabled;                                          // Enabled flag

    Signal<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> _drivingSignal; // Driving signal

    Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> _actuators; // Actuator attachments

    void disableAllActivations();
};


// Servo/Direct Driver
// The servo/direct driver manages a rate-limited servo (or similar) that receives an
// intensity value that corresponds to the angle measurement it should position itself
// at. Typical servos operate on -90 to +90 degree range. Travel rate of FLT_UNDEF is
// considered instantaneous. Continuous servos should instead use HelioMotorDriver.
class HelioServoDriver : public HelioDriver {
public:
    HelioServoDriver(float minDegrees = -90.0f, float maxDegrees = 90.0f,
                     float targetSetpoint = 0.0f,
                     float travelRate = FLT_UNDEF,
                     int type = Servo);

    virtual void update() override;

protected:
};

// Motor Driver
// The motor driver manages a direction-controllable motor (or similar) that receives
// an intensity value that corresponds to travel direction and speed. Device will park
// within target range of target value, and will keep actuators within travel range of
// one another (else disable). Track sensors can be position, speed, or endstop based.
class HelioMotorDriver : public HelioDriver {
public:
    HelioMotorDriver(float trackMax = 100.0f,
                     float trackMin = 0.0f,
                     float targetSetpoint = 0.0f, float targetRange = 0.5f,
                     float travelRate = 1.0f, float travelRange = 2.5f,
                     int type = Motor);

    virtual void update() override;

protected:
    float _targetRange;                                     // Target range
    float _travelRange;                                     // Travel range 
};

#endif // /ifndef HelioDrivers_H

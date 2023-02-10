/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Drivers
*/

#ifndef HelioDrivers_H
#define HelioDrivers_H

class HelioDriver;
class HelioAbsoluteDriver;
class HelioIncrementalDriver;

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
    const enum : signed char { Absolute, Incremental, Unknown = -1 } type; // Driver type (custom RTTI)
    inline bool isAbsoluteType() const { return type == Absolute; }
    inline bool isIncrementalType() const { return type == Incremental; }
    inline bool isUnknownType() const { return type <= Unknown; }

    HelioDriver(float travelRate,
                int type = Unknown);
    virtual ~HelioDriver();

    virtual void update();

    void setActuators(const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &actuators);
    inline const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &getActuators() { return _actuators; }

    virtual void setTargetUnits(Helio_UnitsType targetUnits);
    inline Helio_UnitsType getTargetUnits() const { return definedUnitsElse(_targetUnits, defaultDistanceUnits()); }

    virtual void setTargetSetpoint(float targetSetpoint) override;
    inline float getTargetSetpoint() const { return _targetSetpoint; }
    virtual void setTravelRate(float travelRate) override;
    inline float getTravelRate() const { return _travelRate; }
    inline bool isInstantaneous() const { return _travelRate == FLT_UNDEF; }

    inline void setEnabled(bool enabled) { _enabled = enabled; }
    inline bool isEnabled() const { return _enabled; }

    inline Pair<float,float> getTrackExtents() const { return make_pair(_trackMin, _trackMax); }
    inline bool isWithinTrack(float value) const { return value + FLT_EPSILON >= _trackMin && value - FLT_EPSILON <= _trackMax; }

    virtual Helio_DrivingState getDrivingState() const override;
    virtual float getMaximumOffset(bool poll = false) override;

    Signal<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> &getDrivingSignal();

protected:
    float _trackMin;                                        // Track minimum value
    float _trackMax;                                        // Track maximum value
    float _targetSetpoint;                                  // Target set-point value
    float _travelRate;                                      // Travel rate (distance units / min)
    Helio_UnitsType _targetUnits;                           // Target units
    Helio_DrivingState _drivingState;                       // Current driving state
    bool _enabled;                                          // Enabled flag
    Signal<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> _drivingSignal; // Driving signal
    Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> _actuators; // Actuator attachments

    void disableAllActivations();
};


// Absolute Driver
// The absolute driver manages a rate-limited actuators list driven by an intensity value
// corresponding to a position offset. Travel rate of FLT_UNDEF is considered instantaneous.
class HelioAbsoluteDriver : public HelioDriver {
public:
    HelioAbsoluteDriver(float travelRate = FLT_UNDEF,
                        int type = Absolute);

protected:
    millis_t _lastUpdate;

    virtual void handleOffset(float travelDelta) override;
};

// Incremental Driver
// The incremental driver manages a rate-limited actuators list driven by an intensity
// value corresponding to a speed delta. Driver will park actuators within target range.
class HelioIncrementalDriver : public HelioDriver {
public:
    HelioIncrementalDriver(float targetRange = 0.5f,
                           float travelRate = 1.0f,
                           float travelRange = 2.5f,
                           int type = Incremental);

protected:
    float _targetRange;                                     // Target range
    float _travelRange;                                     // Travel range

    virtual void handleOffset(float travelDelta) override;
};

#endif // /ifndef HelioDrivers_H

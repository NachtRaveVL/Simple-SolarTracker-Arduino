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
class HelioDriver : public HelioSubObject,
                    public HelioDriverObjectInterface,
                    public HelioMeasurementUnitsInterfaceStorageSingle {
public:
    const enum : signed char { Absolute, Incremental, Unknown = -1 } type; // Driver type (custom RTTI)
    inline bool isAbsoluteType() const { return type == Absolute; }
    inline bool isIncrementalType() const { return type == Incremental; }
    inline bool isUnknownType() const { return type <= Unknown; }

    HelioDriver(float targetSetpoint,
                float travelRate,
                int type = Unknown);
    virtual ~HelioDriver();

    virtual void update();

    virtual float getMaxTargetOffset(bool poll = false) override;
    virtual Helio_DrivingState getDrivingState(bool poll = false) override;

    void setActuators(const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &actuators);
    inline const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &getActuators() { return _actuators; }

    virtual void setTargetSetpoint(float targetSetpoint) override;
    inline float getTargetSetpoint() const { return _targetSetpoint; }
    inline void setTravelRate(float travelRate) { _travelRate = travelRate; }
    inline float getTravelRate() const { return _travelRate; }
    inline bool isInstantaneous() const { return _travelRate == FLT_UNDEF; }
    inline Pair<float,float> getTrackRange() const { return _trackRange; }

    virtual void setEnabled(bool enabled);
    inline bool isEnabled() const { return _enabled; }

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t = 0) override;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t = 0) const override;

    Signal<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> &getDrivingSignal();

protected:
    Pair<float,float> _trackRange;                          // Track total range
    float _targetSetpoint;                                  // Target set-point value
    float _travelRate;                                      // Travel rate (distance units / min)
    Helio_DrivingState _drivingState;                       // Driving state (last handled)
    bool _enabled;                                          // Enabled flag
    Signal<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> _drivingSignal; // Driving signal
    Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> _actuators; // Actuator attachments

    void disableAllActivations();

    virtual void handleMaxOffset(float maxOffset) = 0;
};


// Absolute Driver
// The absolute driver manages a position-based actuator list driven by an intensity
// value corresponding to a positional offset. A travel rate of FLT_UNDEF is considered
// instantaneous. Suitable for driving simple positional servos.
class HelioAbsoluteDriver : public HelioDriver {
public:
    HelioAbsoluteDriver(float travelRate = FLT_UNDEF,
                        int type = Absolute);
    virtual ~HelioAbsoluteDriver();

    virtual void setEnabled(bool enabled) override;

protected:
    millis_t _lastUpdate;                                   // Last update millis (for delta-time rate application)

    virtual void handleMaxOffset(float maxOffset) override;
};


// Incremental Driver
// The incremental driver manages direction-based motors. Free-running motors are allowed
// to coast and settle rather than being driven toward exact zero positional error. Small
// overshoot after travel is held inside the nearby range, while larger errors are corrected.
// All actuators must have position data (be derived from HelioPositionSensorAttachmentInterface).
class HelioIncrementalDriver : public HelioDriver {
public:
    HelioIncrementalDriver(float nearbyRange = 0.5f,
                           float alignedRange = 0.05f,
                           float maxDifference = 2.5f,
                           float travelRate = 1.0f,
                           int type = Incremental);
    virtual ~HelioIncrementalDriver();

    virtual Helio_DrivingState getDrivingState(bool poll = false) override;

protected:
    float _nearbyRange;                                     // Nearby target range (for fine/coarse control and overshoot hold)
    float _alignedRange;                                    // Aligned to target range
    float _maxDifference;                                   // Maximum positional difference
    int8_t _lastTravelDirection;                            // Last commanded movement direction

    virtual void handleMaxOffset(float maxOffset) override;

    float getCoastDistance(HelioActuatorAttachment &attachment) const;
    bool shouldHold(HelioActuatorAttachment &attachment, float signedOffset) const;
};

#endif // /ifndef HelioDrivers_H

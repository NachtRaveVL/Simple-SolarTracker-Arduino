/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Triggers
*/

#ifndef HelioTriggers_H
#define HelioTriggers_H

class HelioTrigger;
class HelioMeasurementValueTrigger;
class HelioMeasurementRangeTrigger;

struct HelioTriggerSubData;

#include "Helioduino.h"
#include "HelioObject.h"
#include "HelioSensors.h"

// Creates trigger object from passed trigger sub data (return ownership transfer - user code *must* delete returned object)
extern HelioTrigger *newTriggerObjectFromSubData(const HelioTriggerSubData *dataIn);


// Trigger Base
// This is the base class for all triggers, which are used to alert the system
// to some change in a tracked property.
class HelioTrigger : public HelioSubObject, public HelioTriggerObjectInterface {
public:
    const enum : signed char { MeasureValue, MeasureRange, Unknown = -1 } type; // Trigger type (custom RTTI)
    inline bool isMeasureValueType() const { return type == MeasureValue; }
    inline bool isMeasureRangeType() const { return type == MeasureRange; }
    inline bool isUnknownType() const { return type <= Unknown; }

    HelioTrigger(HelioIdentity sensorId,
                 uint8_t measurementRow = 0,
                 int type = Unknown);
    HelioTrigger(SharedPtr<HelioSensor> sensor,
                 uint8_t measurementRow = 0,
                 int type = Unknown);
    HelioTrigger(const HelioTriggerSubData *dataIn);

    virtual void saveToData(HelioTriggerSubData *dataOut) const;

    virtual void update();
    virtual void handleLowMemory();

    virtual Helio_TriggerState getTriggerState() const override;

    inline void setToleranceUnits(Helio_UnitsType toleranceUnits) { _sensor.setMeasurementUnits(toleranceUnits); }
    inline Helio_UnitsType getToleranceUnits() const { return _sensor.getMeasurementUnits(); }

    inline SharedPtr<HelioSensor> getSensor(bool poll = false) { _sensor.updateIfNeeded(poll); return _sensor.getObject(); }
    inline uint8_t getMeasurementRow() const { return _sensor.getMeasurementRow(); }

    Signal<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS> &getTriggerSignal();

protected:
    HelioSensorAttachment _sensor;                          // Sensor attachment
    Helio_TriggerState _triggerState;                       // Current trigger state
    Signal<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS> _triggerSignal; // Trigger signal

    virtual void handleMeasurement(const HelioMeasurement *measurement) = 0;
};


// Sensor Data Measurement Value Trigger
// This trigger simply checks a measured value against a set tolerance value and is
// useful for simple comparisons that control triggering. Initializes as disabled
// until updated with first measurement, and with undefined units that compares
// directly to measured units, otherwise units can be explicitly set. Can also
// set an additive value that a measurement must go past in order to detrigger.
class HelioMeasurementValueTrigger : public HelioTrigger {
public:
    HelioMeasurementValueTrigger(HelioIdentity sensorId,
                                 float triggerTol,
                                 bool triggerBelow = true,
                                 float detriggerTol = 0,
                                 uint8_t measurementRow = 0);
    HelioMeasurementValueTrigger(SharedPtr<HelioSensor> sensor,
                                 float triggerTol,
                                 bool triggerBelow = true,
                                 float detriggerTol = 0,
                                 uint8_t measurementRow = 0);
    HelioMeasurementValueTrigger(const HelioTriggerSubData *dataIn);

    virtual void saveToData(HelioTriggerSubData *dataOut) const override;

    void setTriggerTolerance(float tolerance);

    inline float getTriggerTolerance() const { return _triggerTol; }
    inline float getDetriggerTolerance() const { return _detriggerTol; }
    inline bool getTriggerBelow() const { return _triggerBelow; }

protected:
    float _triggerTol;                                      // Trigger tolerance limit
    float _detriggerTol;                                    // Detrigger tolerance additive
    bool _triggerBelow;                                     // Trigger below flag

    virtual void handleMeasurement(const HelioMeasurement *measurement) override;
    friend class HelioTrigger;
};


// Sensor Data Measurement Range Trigger
// This trigger checks a measured value against a set tolerance range and is
// useful for ranged measurements that need to stay inside of (or outside of) a
// known range before triggering. Initializes as disabled until updated with
// first measurement, and with undefined units that compares directly to measured
// units, otherwise units can be explicitly set. Can also set an additive value
// that a measurement must go past in order to detrigger.
class HelioMeasurementRangeTrigger : public HelioTrigger {
public:
    HelioMeasurementRangeTrigger(HelioIdentity sensorId,
                                 float toleranceLow,
                                 float toleranceHigh,
                                 bool triggerOutside = true,
                                 float detriggerTol = 0,
                                 uint8_t measurementRow = 0);
    HelioMeasurementRangeTrigger(SharedPtr<HelioSensor> sensor,
                                 float toleranceLow,
                                 float toleranceHigh,
                                 bool triggerOutside = true,
                                 float detriggerTol = 0,
                                 uint8_t measurementRow = 0);
    HelioMeasurementRangeTrigger(const HelioTriggerSubData *dataIn);

    virtual void saveToData(HelioTriggerSubData *dataOut) const override;

    void updateTriggerMidpoint(float toleranceMid);

    inline float getTriggerToleranceLow() const { return _triggerTolLow; }
    inline float getTriggerToleranceHigh() const { return _triggerTolHigh; }
    inline float getDetriggerTolerance() const { return _detriggerTol; }
    inline bool getTriggerOutside() const { return _triggerOutside; }

protected:
    float _triggerTolLow;                                   // Low value tolerance
    float _triggerTolHigh;                                  // High value tolerance
    float _detriggerTol;                                    // Detrigger tolerance additive
    bool _triggerOutside;                                   // Trigger on outside flag

    virtual void handleMeasurement(const HelioMeasurement *measurement) override;
    friend class HelioTrigger;
};


// Combined Trigger Serialization Sub Data
struct HelioTriggerSubData : public HelioSubData {
    char sensorName[HELIO_NAME_MAXSIZE];
    int8_t measurementRow;
    union {
        struct {
            float tolerance;
            bool triggerBelow;
        } measureValue;
        struct {
            float toleranceLow;
            float toleranceHigh;
            bool triggerOutside;
        } measureRange;
    } dataAs;
    float detriggerTol;
    Helio_UnitsType toleranceUnits;

    HelioTriggerSubData();
    virtual void toJSONObject(JsonObject &objectOut) const;
    virtual void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioTriggers_H

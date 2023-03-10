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
class HelioTrigger : public HelioSubObject,
                     public HelioTriggerObjectInterface,
                     public HelioMeasurementUnitsInterfaceStorageSingle,
                     public HelioSensorAttachmentInterface {
public:
    const enum : signed char { MeasureValue, MeasureRange, Unknown = -1 } type; // Trigger type (custom RTTI)
    inline bool isMeasureValueType() const { return type == MeasureValue; }
    inline bool isMeasureRangeType() const { return type == MeasureRange; }
    inline bool isUnknownType() const { return type <= Unknown; }

    HelioTrigger(HelioIdentity sensorId,
                 uint8_t measurementRow,
                 float detriggerTol,
                 millis_t detriggerDelay,
                 int type = Unknown);
    HelioTrigger(SharedPtr<HelioSensor> sensor,
                 uint8_t measurementRow,
                 float detriggerTol,
                 millis_t detriggerDelay,
                 int type = Unknown);
    HelioTrigger(const HelioTriggerSubData *dataIn);

    virtual void saveToData(HelioTriggerSubData *dataOut) const;

    virtual void update();

    virtual Helio_TriggerState getTriggerState(bool poll = false) override;

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t = 0) override;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t = 0) const override;

    inline uint8_t getMeasurementRow() const { return _sensor.getMeasurementRow(); }
    inline float getMeasurementConvertParam() const { return _sensor.getMeasurementConvertParam(); }

    inline float getDetriggerTolerance() const { return _detriggerTol; }
    inline millis_t getDetriggerDelay() const { return _detriggerDelay; }
    inline bool isDetriggerDelayActive() const { return _lastTrigger; }

    virtual HelioSensorAttachment &getSensorAttachment() override;

    Signal<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS> &getTriggerSignal();

protected:
    HelioSensorAttachment _sensor;                          // Sensor attachment
    float _detriggerTol;                                    // De-trigger tolerance additive
    millis_t _detriggerDelay;                               // De-trigger timing delay, in milliseconds
    millis_t _lastTrigger;                                  // Last trigger millis, set to 0 when de-trigger delay met
    Helio_TriggerState _triggerState;                       // Trigger state (last handled)
    Signal<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS> _triggerSignal; // Trigger signal

    virtual void handleMeasurement(const HelioMeasurement *measurement) = 0;
};


// Sensor Data Measurement Value Trigger
// This trigger simply checks a measured value against a set tolerance value and is
// useful for simple comparisons that control triggering. Initializes as disabled
// until updated with first measurement, and with undefined units that compares
// directly to measured units, otherwise units can be explicitly set. Can also
// set an additive value that a measurement must go past in order to de-trigger.
class HelioMeasurementValueTrigger : public HelioTrigger {
public:
    HelioMeasurementValueTrigger(HelioIdentity sensorId,
                                 float triggerTol,
                                 bool triggerBelow = true,
                                 uint8_t measurementRow = 0,
                                 float detriggerTol = 0,
                                 millis_t detriggerDelay = 0);
    HelioMeasurementValueTrigger(SharedPtr<HelioSensor> sensor,
                                 float triggerTol,
                                 bool triggerBelow = true,
                                 uint8_t measurementRow = 0,
                                 float detriggerTol = 0,
                                 millis_t detriggerDelay = 0);
    HelioMeasurementValueTrigger(const HelioTriggerSubData *dataIn);

    virtual void saveToData(HelioTriggerSubData *dataOut) const override;

    // Used for making adjustments to the trigger tolerance.
    void setTriggerTolerance(float tolerance);

    inline float getTriggerTolerance() const { return _triggerTol; }
    inline bool getTriggerBelow() const { return _triggerBelow; }

protected:
    float _triggerTol;                                      // Trigger tolerance limit
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
// that a measurement must go past in order to de-trigger.
class HelioMeasurementRangeTrigger : public HelioTrigger {
public:
    HelioMeasurementRangeTrigger(HelioIdentity sensorId,
                                 float toleranceLow,
                                 float toleranceHigh,
                                 bool triggerOutside = true,
                                 uint8_t measurementRow = 0,
                                 float detriggerTol = 0,
                                 millis_t detriggerDelay = 0);
    HelioMeasurementRangeTrigger(SharedPtr<HelioSensor> sensor,
                                 float toleranceLow,
                                 float toleranceHigh,
                                 bool triggerOutside = true,
                                 uint8_t measurementRow = 0,
                                 float detriggerTol = 0,
                                 millis_t detriggerDelay = 0);
    HelioMeasurementRangeTrigger(const HelioTriggerSubData *dataIn);

    virtual void saveToData(HelioTriggerSubData *dataOut) const override;

    // Used for making adjustments to the trigger tolerance midpoint.
    void setTriggerMidpoint(float toleranceMid);

    inline float getTriggerToleranceLow() const { return _triggerTolLow; }
    inline float getTriggerToleranceHigh() const { return _triggerTolHigh; }
    inline bool getTriggerOutside() const { return _triggerOutside; }

protected:
    float _triggerTolLow;                                   // Low value tolerance
    float _triggerTolHigh;                                  // High value tolerance
    bool _triggerOutside;                                   // Trigger on outside flag

    virtual void handleMeasurement(const HelioMeasurement *measurement) override;
    friend class HelioTrigger;
};


// Combined Trigger Serialization Sub Data
struct HelioTriggerSubData : public HelioSubData {
    char sensorName[HELIO_NAME_MAXSIZE];                    // Sensor name
    int8_t measurementRow;                                  // Measurement row
    union {
        struct {
            float tolerance;                                // Value tolerance
            bool triggerBelow;                              // Trigger below flag
        } measureValue;                                     // Measure value type
        struct {
            float toleranceLow;                             // Low value tolerance
            float toleranceHigh;                            // High value tolerance
            bool triggerOutside;                            // Trigger outside flag
        } measureRange;
    } dataAs;                                               // Data type union
    float detriggerTol;                                     // De-trigger tolerance
    millis_t detriggerDelay;                                // De-trigger delay millis
    Helio_UnitsType measurementUnits;                       // Measurement units

    HelioTriggerSubData();
    virtual void toJSONObject(JsonObject &objectOut) const;
    virtual void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioTriggers_H

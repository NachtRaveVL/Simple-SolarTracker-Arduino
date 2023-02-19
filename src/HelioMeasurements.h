/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Sensor Measurements
*/

#ifndef HelioMeasurements_H
#define HelioMeasurements_H

struct HelioMeasurement;
struct HelioSingleMeasurement;
struct HelioBinaryMeasurement;
struct HelioDoubleMeasurement;
struct HelioTripleMeasurement;

struct HelioMeasurementData;

#include "Helioduino.h"
#include "HelioData.h"

// Creates measurement object from passed trigger sub data (return ownership transfer - user code *must* delete returned object)
extern HelioMeasurement *newMeasurementObjectFromSubData(const HelioMeasurementData *dataIn);

// Gets the value of a measurement at a specified row (with optional binary true scaling value).
extern float getMeasurementValue(const HelioMeasurement *measurement, uint8_t measurementRow = 0, float binScale = 1.0f);
// Gets the units of a measurement at a specified row (with optional binary units).
extern Helio_UnitsType getMeasurementUnits(const HelioMeasurement *measurement, uint8_t measurementRow = 0, Helio_UnitsType binUnits = Helio_UnitsType_Raw_1);
// Gets the number of rows of data that a measurement holds.
extern uint8_t getMeasurementRowCount(const HelioMeasurement *measurement);
// Gets the single measurement of a measurement (with optional binary true value / units).
extern HelioSingleMeasurement getAsSingleMeasurement(const HelioMeasurement *measurement, uint8_t measurementRow = 0, float binScale = 1.0f, Helio_UnitsType binUnits = Helio_UnitsType_Raw_1);

// Sensor Data Measurement Base
struct HelioMeasurement {
    enum : signed char { Binary, Single, Double, Triple, Unknown = -1 } type; // Measurement type (custom RTTI)
    inline bool isBinaryType() const { return type == Binary; }
    inline bool isSingleType() const { return type == Single; }
    inline bool isDoubleType() const { return type == Double; }
    inline bool isTripleType() const { return type == Triple; }
    inline bool isUnknownType() const { return type <= Unknown; }

    time_t timestamp;                                       // Time event recorded (UTC)
    hframe_t frame;                                         // Polling frame # measurement taken on, or 0 if not-set else 1 if user-set

    inline HelioMeasurement() : type(Unknown), frame(0), timestamp(unixNow()) { ; }
    inline HelioMeasurement(int classType, time_t timestampIn, hframe_t frameIn) : type((typeof(type))classType), timestamp(timestampIn), frame(frameIn) { ; }
    HelioMeasurement(int classType, time_t timestamp = 0);
    HelioMeasurement(const HelioMeasurementData *dataIn);

    void saveToData(HelioMeasurementData *dataOut, uint8_t measurementRow = 0, unsigned int additionalDecPlaces = 0) const;

    inline void updateTimestamp() { timestamp = unixNow(); }
    void updateFrame(hframe_t minFrame = 0);
    inline void setMinFrame(hframe_t minFrame = 0) { frame = max(minFrame, frame); }
};

// Single Value Sensor Data Measurement
struct HelioSingleMeasurement : public HelioMeasurement {
    float value;                                            // Polled value
    Helio_UnitsType units;                                  // Units of value

    HelioSingleMeasurement();
    HelioSingleMeasurement(float value, Helio_UnitsType units, time_t timestamp = unixNow());
    HelioSingleMeasurement(float value, Helio_UnitsType units, time_t timestamp, hframe_t frame);
    HelioSingleMeasurement(const HelioMeasurementData *dataIn);

    inline HelioSingleMeasurement asUnits(Helio_UnitsType outUnits, float convertParam = FLT_UNDEF) const; // in utils

    inline HelioSingleMeasurement wrappedBy(float range) const;
    inline HelioSingleMeasurement wrappedBySplit(float range) const;
    inline HelioSingleMeasurement wrappedBy360() const { return wrappedBy(360); }
    inline HelioSingleMeasurement wrappedBy180Neg180() const { return wrappedBySplit(360); }
    inline HelioSingleMeasurement wrappedBy2Pi() const { return wrappedBy(TWO_PI); }
    inline HelioSingleMeasurement wrappedByPiNegPi() const { return wrappedBySplit(TWO_PI); }
    inline HelioSingleMeasurement wrappedBy24Hr() const { return wrappedBy(MIN_PER_DAY); }
    inline HelioSingleMeasurement wrappedBy12HrNeg12Hr() const { return wrappedBySplit(MIN_PER_DAY); }

    void saveToData(HelioMeasurementData *dataOut, uint8_t measurementRow = 0, unsigned int additionalDecPlaces = 0) const;
};

// Binary Value Sensor Data Measurement
struct HelioBinaryMeasurement : public HelioMeasurement {
    bool state;                                             // Polled state

    HelioBinaryMeasurement();
    HelioBinaryMeasurement(bool state, time_t timestamp = unixNow());
    HelioBinaryMeasurement(bool state, time_t timestamp, hframe_t frame);
    HelioBinaryMeasurement(const HelioMeasurementData *dataIn);

    void saveToData(HelioMeasurementData *dataOut, uint8_t measurementRow = 0, unsigned int additionalDecPlaces = 0) const;

    inline HelioSingleMeasurement getAsSingleMeasurement(float binScale = 1.0f, Helio_UnitsType binUnits = Helio_UnitsType_Raw_1) { return HelioSingleMeasurement(state ? binScale : 0.0f, binUnits, timestamp, frame); }
};

// Double Value Sensor Data Measurement
struct HelioDoubleMeasurement : public HelioMeasurement {
    float value[2];                                         // Polled values
    Helio_UnitsType units[2];                               // Units of values

    HelioDoubleMeasurement();
    HelioDoubleMeasurement(float value1, Helio_UnitsType units1, 
                           float value2, Helio_UnitsType units2, 
                           time_t timestamp = unixNow());
    HelioDoubleMeasurement(float value1, Helio_UnitsType units1, 
                           float value2, Helio_UnitsType units2, 
                           time_t timestamp, hframe_t frame);
    HelioDoubleMeasurement(const HelioMeasurementData *dataIn);

    void saveToData(HelioMeasurementData *dataOut, uint8_t measurementRow = 0, unsigned int additionalDecPlaces = 0) const;

    inline HelioSingleMeasurement getAsSingleMeasurement(uint8_t measurementRow) { return HelioSingleMeasurement(value[measurementRow], units[measurementRow], timestamp, frame); }
};

// Triple Value Sensor Data Measurement
struct HelioTripleMeasurement : public HelioMeasurement {
    float value[3];                                         // Polled values
    Helio_UnitsType units[3];                               // Units of values

    HelioTripleMeasurement();
    HelioTripleMeasurement(float value1, Helio_UnitsType units1, 
                           float value2, Helio_UnitsType units2, 
                           float value3, Helio_UnitsType units3,
                           time_t timestamp = unixNow());
    HelioTripleMeasurement(float value1, Helio_UnitsType units1, 
                           float value2, Helio_UnitsType units2, 
                           float value3, Helio_UnitsType units3,
                           time_t timestamp, hframe_t frame);
    HelioTripleMeasurement(const HelioMeasurementData *dataIn);

    void saveToData(HelioMeasurementData *dataOut, uint8_t measurementRow = 0, unsigned int additionalDecPlaces = 0) const;

    inline HelioSingleMeasurement getAsSingleMeasurement(uint8_t measurementRow) { return HelioSingleMeasurement(value[measurementRow], units[measurementRow], timestamp, frame); }
    inline HelioDoubleMeasurement getAsDoubleMeasurement(uint8_t measurementRow1, uint8_t measurementRow2) { return HelioDoubleMeasurement(value[measurementRow1], units[measurementRow1], value[measurementRow2], units[measurementRow2], timestamp, frame); }
};


// Combined Measurement Serialization Sub Data
struct HelioMeasurementData : public HelioSubData {
    uint8_t measurementRow;                                 // Source measurement row index that data is from
    float value;                                            // Value
    Helio_UnitsType units;                                  // Units of value
    time_t timestamp;                                       // Timestamp

    HelioMeasurementData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
    void fromJSONVariant(JsonVariantConst &variantIn);
};

#endif // /ifndef HelioMeasurements_H

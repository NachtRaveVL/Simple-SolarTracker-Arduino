/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Datas
*/

#ifndef HelioDatas_H
#define HelioDatas_H

struct HelioSystemData;
struct HelioCalibrationData;

#include "Helioduino.h"
#include "HelioData.h"
#include "HelioScheduler.h"
#include "HelioPublisher.h"
#include "HelioLogger.h"

// Autosave Enumeration
enum Helio_Autosave : signed char {
    Helio_Autosave_EnabledToSDCardJson,                     // Autosave to SD card in Json
    Helio_Autosave_EnabledToSDCardRaw,                      // Autosave to SD card in binary
    Helio_Autosave_EnabledToEEPROMJson,                     // Autosave to EEPROM in Json
    Helio_Autosave_EnabledToEEPROMRaw,                      // Autosave to EEPROM in binary
    Helio_Autosave_EnabledToWiFiStorageJson,                // Autosave to WiFiStorage in Json
    Helio_Autosave_EnabledToWiFiStorageRaw,                 // Autosave to WiFiStorage in binary
    Helio_Autosave_Disabled = -1                            // Autosave disabled
};

// User System Setup Data
// id: HSYS. Helioduino user system setup data.
struct HelioSystemData : public HelioData {
    Helio_SystemMode systemMode;                            // System type mode
    Helio_MeasurementMode measureMode;                      // System measurement mode
    Helio_DisplayOutputMode dispOutMode;                    // System display output mode
    Helio_ControlInputMode ctrlInMode;                      // System control input mode 
    char systemName[HELIO_NAME_MAXSIZE];                    // System name
    int8_t timeZoneOffset;                                  // Timezone offset
    uint16_t pollingInterval;                               // Sensor polling interval, in milliseconds
    Helio_Autosave autosaveEnabled;                         // Autosave enabled
    Helio_Autosave autosaveFallback;                        // Autosave fallback
    uint16_t autosaveInterval;                              // Autosave interval, in minutes
    char wifiSSID[HELIO_NAME_MAXSIZE];                      // WiFi SSID
    uint8_t wifiPassword[HELIO_NAME_MAXSIZE];               // WiFi password (xor encrypted)
    uint32_t wifiPasswordSeed;                              // Seed for WiFi password one-time pad
    uint8_t macAddress[6];                                  // Ethernet MAC address

    HelioSchedulerSubData scheduler;                        // Scheduler subdata
    HelioLoggerSubData logger;                              // Logger subdata
    HelioPublisherSubData publisher;                        // Publisher subdata

    HelioSystemData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};


// Calibration Data
// id: HCAL. Helioduino linear calibration data.
// This class essentially controls a simple Ax+B linear transformation mapping, and is
// used to 'convert' values from one coordinate system into another, or in our case used
// for storing custom user curve/offset correction/mapping data.
// See setFrom* methods to set calibrated data in various formats.
struct HelioCalibrationData : public HelioData {
    char ownerName[HELIO_NAME_MAXSIZE];                     // Owner object name this calibration belongs to (actuator/sensor)
    Helio_UnitsType calibUnits;                             // Calibration output units
    float multiplier, offset;                               // Ax + B value transform coefficients

    HelioCalibrationData();
    HelioCalibrationData(HelioIdentity ownerId,
                         Helio_UnitsType calibUnits = Helio_UnitsType_Undefined);

    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;

    // Transforms value from raw (or initial) value into calibrated (or transformed) value.
    inline float transform(float value) const { return (value * multiplier) + offset; }
    // Transforms value in-place from raw (or initial) value into calibrated (or transformed) value, with optional units write out.
    inline void transform(float *valueInOut, Helio_UnitsType *unitsOut = nullptr) const { *valueInOut = transform(*valueInOut);
                                                                                          if (unitsOut) { *unitsOut = calibUnits; } }
    // Transforms measurement from raw (or initial) measurement into calibrated (or transformed) measurement.
    inline HelioSingleMeasurement transform(HelioSingleMeasurement measurement) { return HelioSingleMeasurement(transform(measurement.value), calibUnits, measurement.timestamp, measurement.frame); }
    // Transforms measurement in-place from raw (or initial) measurement into calibrated (or transformed) measurement.
    inline void transform(HelioSingleMeasurement *measurementInOut) const { transform(&measurementInOut->value, &measurementInOut->units); }

    // Inverse transforms value from calibrated (or transformed) value back into raw (or initial) value.
    inline float inverseTransform(float value) const { return (value - offset) / multiplier; }
    // Inverse transforms value in-place from calibrated (or transformed) value back into raw (or initial) value, with optional units write out.
    inline void inverseTransform(float *valueInOut, Helio_UnitsType *unitsOut = nullptr) const { *valueInOut = inverseTransform(*valueInOut);
                                                                                                 if (unitsOut) { *unitsOut = Helio_UnitsType_Raw_0_1; } }
    // Inverse transforms measurement from calibrated (or transformed) measurement back into raw (or initial) measurement.
    inline HelioSingleMeasurement inverseTransform(HelioSingleMeasurement measurement) { return HelioSingleMeasurement(inverseTransform(measurement.value), calibUnits, measurement.timestamp, measurement.frame); }
    // Inverse transforms measurement in-place from calibrated (or transformed) measurement back into raw (or initial) measurement.
    inline void inverseTransform(HelioSingleMeasurement *measurementInOut) const { inverseTransform(&measurementInOut->value, &measurementInOut->units); }

    // Sets linear calibration curvature from two points.
    // Measured normalized raw values should be between 0.0 and 1.0, and represents
    // the normalized voltage signal measurement from the analogRead() function (after
    // taking into account appropiate bit resolution conversion). Calibrated-to values
    // are what each measurement-at value should map out to.
    // For example, if your sensor should treat 0v (aka 0.0) as a value of 2 and treat 5v
    // (aka 1.0, or MCU max voltage) as a value of 10, you would pass 0.0, 2.0, 1.0, 10.0.
    // The final calculated curvature transform, for this example, would be y = 8x + 2.
    void setFromTwoPoints(float point1RawMeasuredAt,        // What normalized value point 1 measured in at [0.0,1.0]
                          float point1CalibratedTo,         // What value point 1 should be mapped to
                          float point2RawMeasuredAt,        // What normalized value point 2 measured in at [0.0,1.0]
                          float point2CalibratedTo);        // What value point 2 should be mapped to

    // Sets linear calibration curvature from two voltages.
    // Wrapper to setFromTwoPoints, used when raw voltage values are easier to work with.
    inline void setFromTwoVoltages(float point1VoltsAt,     // What raw voltage value point 1 measured in at [0.0,aRef]
                                   float point1CalibTo,     // What value point 1 should be mapped to
                                   float point2VoltsAt,     // What raw voltage value point 2 measured in at [0.0,aRef]
                                   float point2CalibTo,     // What value point 2 should be mapped to
                                   float analogRefVolts) {  // aRef: Value of aRef pin (use 5 for 5v MCUs, 3.3 for 3.3v MCUs)
        setFromTwoPoints(point1VoltsAt / analogRefVolts, point1CalibTo,
                         point2VoltsAt / analogRefVolts, point2CalibTo);
    }

    // Sets linear calibration curvature from known output range.
    // Wrapper to setFromTwoPoints, used when data uses the entire intensity range with a known min/max value at each end.
    // E.g. will map 0v (aka 0.0) to min value and 5v (aka 1.0, or MCU max voltage) to max value.
    inline void setFromRange(float min, float max) { setFromTwoPoints(0.0, min, 1.0, max); }

    // Sets linear calibration curvature from known output scale.
    // Similar to setFromTwoPoints, but when data has a known max intensity.
    // E.g. will map 0v to 0 and 5v (aka 1.0, or MCU max voltage) to scale value.
    inline void setFromScale(float scale) { setFromRange(0.0, scale); }

    // Sets linear calibration curvature from typical servo ranges.
    // Wrapper to setFromTwoPoints, used for specifying servo degree operation ranges using the typical 2.5% and 12.5% phase lengths that hobbyist servos operate at.
    // E.g. will map 2.5% (servo min/neg position/speed) to minDegrees and 12.5% (servo max/pos position/speed) to maxDegrees.
    inline void setFromServo(float minDegrees, float maxDegrees) { setFromTwoPoints(0.025f, minDegrees, 0.125f, maxDegrees); }
};


// Internal use, but must contain all ways for all data types to be new'ed
extern HelioData *_allocateDataFromBaseDecode(const HelioData &baseDecode);
extern HelioData *_allocateDataForObjType(int8_t idType, int8_t classType);

#endif // /ifndef HelioDatas_H

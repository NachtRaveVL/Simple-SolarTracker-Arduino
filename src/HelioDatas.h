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
// id: HSYS. Sol tracking user system setup data.
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


// Sensor Calibration Data
// id: HCAL. Sol tracking sensor calibration data.
// This class essentially controls a custom unit conversion mapping, and is used in
// converting raw sensor data to more useful value and units for doing science with.
// To convert from raw values to calibrated values, use transform(). To convert back to
// raw values from calibrated values, use inverseTransform(). The setFrom* methods
// allow you to easily set calibrated data in various formats.
struct HelioCalibrationData : public HelioData {
    char sensorName[HELIO_NAME_MAXSIZE];                    // Sensor name this calibration belongs to
    Helio_UnitsType calibUnits;                             // Calibration output units
    float multiplier, offset;                               // Ax + B value transform coefficients

    HelioCalibrationData();
    HelioCalibrationData(HelioIdentity sensorId,
                         Helio_UnitsType calibUnits = Helio_UnitsType_Undefined);

    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;

    // Transforms value from raw (or initial) value into calibrated (or transformed) value.
    inline float transform(float rawValue) const { return (rawValue * multiplier) + offset; }
    // Transforms value in-place from raw (or initial) value into calibrated (or transformed) value, with optional units write out.
    inline void transform(float *valueInOut, Helio_UnitsType *unitsOut = nullptr) const { *valueInOut = transform(*valueInOut);
                                                                                          if (unitsOut) { *unitsOut = calibUnits; } }
    // Transforms measurement from raw (or initial) measurement into calibrated (or transformed) measurement.
    inline HelioSingleMeasurement transform(HelioSingleMeasurement rawMeasurement) { return HelioSingleMeasurement(transform(rawMeasurement.value), calibUnits, rawMeasurement.timestamp, rawMeasurement.frame); }
    // Transforms measurement in-place from raw (or initial) measurement into calibrated (or transformed) measurement.
    inline void transform(HelioSingleMeasurement *measurementInOut) const { transform(&measurementInOut->value, &measurementInOut->units); }

    // Inverse transforms value from calibrated (or transformed) value back into raw (or initial) value.
    inline float inverseTransform(float calibratedValue) const { return (calibratedValue - offset) / multiplier; }
    // Inverse transforms value in-place from calibrated (or transformed) value back into raw (or initial) value, with optional units write out.
    inline void inverseTransform(float *valueInOut, Helio_UnitsType *unitsOut = nullptr) const { *valueInOut = inverseTransform(*valueInOut);
                                                                                                 if (unitsOut) { *unitsOut = Helio_UnitsType_Raw_0_1; } }
    // Inverse transforms measurement from calibrated (or transformed) measurement back into raw (or initial) measurement.
    inline HelioSingleMeasurement inverseTransform(HelioSingleMeasurement rawMeasurement) { return HelioSingleMeasurement(inverseTransform(rawMeasurement.value), calibUnits, rawMeasurement.timestamp, rawMeasurement.frame); }
    // Inverse transforms measurement in-place from calibrated (or transformed) measurement back into raw (or initial) measurement.
    inline void inverseTransform(HelioSingleMeasurement *measurementInOut) const { inverseTransform(&measurementInOut->value, &measurementInOut->units); }

    // Sets linear calibration curvature from two points.
    // Measured normalized raw values should be between 0.0 and 1.0, and represents
    // the normalized voltage signal measurement from the analogRead() function (after
    // taking into account appropiate bit resolution conversion). Calibrated-to values
    // are what each measurement-at value should map out to.
    // For example, if your sensor should treat 0v (aka 0.0) as a pH of 2 and treat 5v
    // (aka 1.0, or MCU max voltage) as a pH of 10, you would pass 0.0, 2.0, 1.0, 10.0.
    // The final calculated curvature transform, for this example, would be y = 8x + 2.
    void setFromTwoPoints(float point1RawMeasuredAt,        // What normalized value point 1 measured in at [0.0,1.0]
                          float point1CalibratedTo,         // What value point 1 should be mapped to
                          float point2RawMeasuredAt,        // What normalized value point 2 measured in at [0.0,1.0]
                          float point2CalibratedTo);        // What value point 2 should be mapped to

    // Sets linear calibration curvature from two voltages.
    // Wrapper to setFromTwoPoints, used when raw voltage values are easier to work with.
    inline void setFromTwoVoltages(float point1VoltageAt,   // What raw voltage value point 1 measured in at [0.0,aRef]
                                   float point1CalibratedTo, // What value point 1 should be mapped to
                                   float point2VoltageAt,   // What raw voltage value point 2 measured in at [0.0,aRef]
                                   float point2CalibratedTo, // What value point 2 should be mapped to
                                   float analogRefVoltage) { // aRef: Value of aRef pin (if not connected uses default of 5 for 5v MCUs, 3.3 for 3.3v MCUs)
        setFromTwoPoints(point1VoltageAt / analogRefVoltage, point1CalibratedTo,
                         point2VoltageAt / analogRefVoltage, point2CalibratedTo);
    }

    // Sets linear calibration curvature from known output range.
    // Wrapper to setFromTwoPoints, used when the sensor uses the entire voltage range
    // with a known min/max value at each end. This will map 0v (aka 0.0) to min value
    // and 5v (aka 1.0, or MCU max voltage) to max value.
    inline void setFromRange(float min, float max) { setFromTwoPoints(0.0, min, 1.0, max); }

    // Sets linear calibration curvature from known output scale.
    // Similar to setFromTwoPoints, but when the sensor has a known max scale.
    // This will map 0v to 0 and 5v (or MCU max voltage) to scale value.
    inline void setFromScale(float scale) { setFromRange(0.0, scale); }
};


// Internal use, but must contain all ways for all data types to be new'ed
extern HelioData *_allocateDataFromBaseDecode(const HelioData &baseDecode);
extern HelioData *_allocateDataForObjType(int8_t idType, int8_t classType);

#endif // /ifndef HelioDatas_H
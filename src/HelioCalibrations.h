/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Calibrations Storage
*/

#ifndef HelioCalibrations_H
#define HelioCalibrations_H

class HelioCalibrations;

#include "Helioduino.h"

// Calibrations Storage
// Stores user calibration data, which calibrates the various sensors output to
// an usable input value.
class HelioCalibrations {
public:
    // Adds/updates user calibration data to storage, returning success flag
    bool setUserCalibrationData(const HelioCalibrationData *calibrationData);

    // Drops/removes user calibration data from storage, returning success flag
    bool dropUserCalibrationData(const HelioCalibrationData *calibrationData);

    // Returns user calibration data instance in storage
    const HelioCalibrationData *getUserCalibrationData(hkey_t key) const;

    // Returns if there are any user calibrations in storage
    inline bool hasUserCalibrations() const { return _calibrationData.size(); };

protected:
    Map<hkey_t, HelioCalibrationData *, HELIO_CAL_CALIBSTORE_MAXSIZE> _calibrationData; // Loaded user calibration data
};

#endif // /ifndef HelioCalibrations_H

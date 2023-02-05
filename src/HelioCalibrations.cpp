/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Calibrations Storage
*/

#include "Helioduino.h"

const HelioCalibrationData *HelioCalibrations::getUserCalibrationData(hkey_t key) const
{
    auto iter = _calibrationData.find(key);
    if (iter != _calibrationData.end()) {
        return iter->second;
    }
    return nullptr;
}

bool HelioCalibrations::setUserCalibrationData(const HelioCalibrationData *calibrationData)
{
    HELIO_SOFT_ASSERT(calibrationData, SFP(HStr_Err_InvalidParameter));

    if (calibrationData) {
        hkey_t key = stringHash(calibrationData->ownerName);
        auto iter = _calibrationData.find(key);
        bool retVal = false;

        if (iter == _calibrationData.end()) {
            auto calibData = new HelioCalibrationData();

            HELIO_SOFT_ASSERT(calibData, SFP(HStr_Err_AllocationFailure));
            if (calibData) {
                *calibData = *calibrationData;
                _calibrationData[key] = calibData;
                retVal = (_calibrationData.find(key) != _calibrationData.end());
            }
        } else {
            *(iter->second) = *calibrationData;
            retVal = true;
        }

        return retVal;
    }
    return false;
}

bool HelioCalibrations::dropUserCalibrationData(const HelioCalibrationData *calibrationData)
{
    HELIO_HARD_ASSERT(calibrationData, SFP(HStr_Err_InvalidParameter));
    hkey_t key = stringHash(calibrationData->ownerName);
    auto iter = _calibrationData.find(key);

    if (iter != _calibrationData.end()) {
        if (iter->second) { delete iter->second; }
        _calibrationData.erase(iter);

        return true;
    }

    return false;
}

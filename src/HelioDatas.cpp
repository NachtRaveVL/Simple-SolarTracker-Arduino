/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Datas
*/

#include "Helioduino.h"

HelioData *_allocateDataFromBaseDecode(const HelioData &baseDecode)
{
    HelioData *retVal = nullptr;

    if (baseDecode.isStandardData()) {
        if (baseDecode.isSystemData()) {
            retVal = new HelioSystemData();
        } else if (baseDecode.isCalibrationData()) {
            retVal = new HelioCalibrationData();
        }
    } else if (baseDecode.isObjectData()) {
        retVal = _allocateDataForObjType(baseDecode.id.object.idType, baseDecode.id.object.classType);
    }

    HELIO_SOFT_ASSERT(retVal, F("Unknown data decode"));
    if (retVal) {
        retVal->id = baseDecode.id;
        HELIO_SOFT_ASSERT(retVal->_version == baseDecode._version, F("Data version mismatch")); // TODO: Version updaters
        retVal->_revision = baseDecode._revision;
        return retVal;
    }
    return new HelioData(baseDecode);
}

HelioData *_allocateDataForObjType(int8_t idType, int8_t classType)
{
    switch (idType) {
        case (int8_t)HelioIdentity::Actuator:
            switch (classType) {
                case (int8_t)HelioActuator::Relay:
                    return new HelioActuatorData();
                case (int8_t)HelioActuator::Variable:
                    return new HelioActuatorData();
                default: break;
            }
            break;

        case (int8_t)HelioIdentity::Sensor:
            switch (classType) {
                case (int8_t)HelioSensor::Binary:
                    return new HelioBinarySensorData();
                case (int8_t)HelioSensor::Analog:
                    return new HelioAnalogSensorData();
                //case 2: // Digital (not instance-able)
                case (int8_t)HelioSensor::DHT1W:
                    return new HelioDHTTempHumiditySensorData();
                default: break;
            }
            break;

        case (int8_t)HelioIdentity::Panel:
            switch (classType) {
                // todo
                //case (int8_t)HelioPanel::SingleAxis:
                //    return new HelioSingleAxisPanelData();
                //case (int8_t)HelioPanel::DualAxis:
                //    return new HelioDualAxisPanelData();
                default: break;
            }
            break;

        case (int8_t)HelioIdentity::Rail:
            switch (classType) {
                case (int8_t)HelioRail::Simple:
                    return new HelioSimpleRailData();
                case (int8_t)HelioRail::Regulated:
                    return new HelioRegulatedRailData();
                default: break;
            }
            break;

        default: break;
    }

    return nullptr;
}

HelioSystemData::HelioSystemData()
    : HelioData('H','S','Y','S', 1),
      systemMode(Helio_SystemMode_Undefined), measureMode(Helio_MeasurementMode_Undefined),
      dispOutMode(Helio_DisplayOutputMode_Undefined), ctrlInMode(Helio_ControlInputMode_Undefined),
      systemName{0}, timeZoneOffset(0), pollingInterval(HELIO_DATA_LOOP_INTERVAL),
      autosaveEnabled(Helio_Autosave_Disabled), autosaveFallback(Helio_Autosave_Disabled), autosaveInterval(HELIO_SYS_AUTOSAVE_INTERVAL),
      wifiSSID{0}, wifiPassword{0}, wifiPasswordSeed(0),
      macAddress{0}
{
    _size = sizeof(*this);
    HELIO_HARD_ASSERT(isSystemData(), SFP(HStr_Err_OperationFailure));
    strncpy(systemName, SFP(HStr_Default_SystemName).c_str(), HELIO_NAME_MAXSIZE);
}

void HelioSystemData::toJSONObject(JsonObject &objectOut) const
{
    HelioData::toJSONObject(objectOut);

    objectOut[SFP(HStr_Key_SystemMode)] = systemModeToString(systemMode);
    objectOut[SFP(HStr_Key_MeasureMode)] = measurementModeToString(measureMode);
    #ifdef HELIO_USE_GUI
        objectOut[SFP(HStr_Key_DispOutMode)] = displayOutputModeToString(dispOutMode);
        objectOut[SFP(HStr_Key_CtrlInMode)] = controlInputModeToString(ctrlInMode);
    #else
        objectOut[SFP(HStr_Key_DispOutMode)] = displayOutputModeToString(Helio_DisplayOutputMode_Disabled);
        objectOut[SFP(HStr_Key_CtrlInMode)] = controlInputModeToString(Helio_ControlInputMode_Disabled);
    #endif
    if (systemName[0]) { objectOut[SFP(HStr_Key_SystemName)] = charsToString(systemName, HELIO_NAME_MAXSIZE); }
    if (timeZoneOffset != 0) { objectOut[SFP(HStr_Key_TimeZoneOffset)] = timeZoneOffset; }
    if (pollingInterval && pollingInterval != HELIO_DATA_LOOP_INTERVAL) { objectOut[SFP(HStr_Key_PollingInterval)] = pollingInterval; }
    if (autosaveEnabled != Helio_Autosave_Disabled) { objectOut[SFP(HStr_Key_AutosaveEnabled)] = autosaveEnabled; }
    if (autosaveFallback != Helio_Autosave_Disabled) { objectOut[SFP(HStr_Key_AutosaveFallback)] = autosaveFallback; }
    if (autosaveInterval && autosaveInterval != HELIO_SYS_AUTOSAVE_INTERVAL) { objectOut[SFP(HStr_Key_AutosaveInterval)] = autosaveInterval; }
    if (wifiSSID[0]) { objectOut[SFP(HStr_Key_WiFiSSID)] = charsToString(wifiSSID, HELIO_NAME_MAXSIZE); }
    if (wifiPasswordSeed) {
        objectOut[SFP(HStr_Key_WiFiPassword)] = hexStringFromBytes(wifiPassword, HELIO_NAME_MAXSIZE);
        objectOut[SFP(HStr_Key_WiFiPasswordSeed)] = wifiPasswordSeed;
    } else if (wifiPassword[0]) {
        objectOut[SFP(HStr_Key_WiFiPassword)] = charsToString((const char *)wifiPassword, HELIO_NAME_MAXSIZE);
    }
    if (!arrayElementsEqual<uint8_t>(macAddress, 6, 0)) {
        objectOut[SFP(HStr_Key_MACAddress)] = commaStringFromArray(macAddress, 6);
    }

    JsonObject schedulerObj = objectOut.createNestedObject(SFP(HStr_Key_Scheduler));
    scheduler.toJSONObject(schedulerObj); if (!schedulerObj.size()) { objectOut.remove(SFP(HStr_Key_Scheduler)); }
    JsonObject loggerObj = objectOut.createNestedObject(SFP(HStr_Key_Logger));
    logger.toJSONObject(loggerObj); if (!loggerObj.size()) { objectOut.remove(SFP(HStr_Key_Logger)); }
    JsonObject publisherObj = objectOut.createNestedObject(SFP(HStr_Key_Publisher));
    publisher.toJSONObject(publisherObj); if (!publisherObj.size()) { objectOut.remove(SFP(HStr_Key_Publisher)); }
}

void HelioSystemData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioData::fromJSONObject(objectIn);

    systemMode = systemModeFromString(objectIn[SFP(HStr_Key_SystemMode)]);
    measureMode = measurementModeFromString(objectIn[SFP(HStr_Key_MeasureMode)]);
    #ifdef HELIO_USE_GUI
        dispOutMode = displayOutputModeFromString(objectIn[SFP(HStr_Key_DispOutMode)]);
        ctrlInMode = controlInputModeFromString(objectIn[SFP(HStr_Key_CtrlInMode)]);
    #else
        dispOutMode = Helio_DisplayOutputMode_Disabled;
        ctrlInMode = Helio_ControlInputMode_Disabled;
    #endif
    const char *systemNameStr = objectIn[SFP(HStr_Key_SystemName)];
    if (systemNameStr && systemNameStr[0]) { strncpy(systemName, systemNameStr, HELIO_NAME_MAXSIZE); }
    timeZoneOffset = objectIn[SFP(HStr_Key_TimeZoneOffset)] | timeZoneOffset;
    pollingInterval = objectIn[SFP(HStr_Key_PollingInterval)] | pollingInterval;
    autosaveEnabled = objectIn[SFP(HStr_Key_AutosaveEnabled)] | autosaveEnabled;
    autosaveFallback = objectIn[SFP(HStr_Key_AutosaveFallback)] | autosaveFallback;
    autosaveInterval = objectIn[SFP(HStr_Key_AutosaveInterval)] | autosaveInterval;
    const char *wifiSSIDStr = objectIn[SFP(HStr_Key_WiFiSSID)];
    if (wifiSSIDStr && wifiSSIDStr[0]) { strncpy(wifiSSID, wifiSSIDStr, HELIO_NAME_MAXSIZE); }
    const char *wifiPasswordStr = objectIn[SFP(HStr_Key_WiFiPassword)];
    wifiPasswordSeed = objectIn[SFP(HStr_Key_WiFiPasswordSeed)] | wifiPasswordSeed;
    if (wifiPasswordStr && wifiPasswordSeed) { hexStringToBytes(String(wifiPasswordStr), wifiPassword, HELIO_NAME_MAXSIZE); }
    else if (wifiPasswordStr && wifiPasswordStr[0]) { strncpy((char *)wifiPassword, wifiPasswordStr, HELIO_NAME_MAXSIZE); wifiPasswordSeed = 0; }
    JsonVariantConst macAddressVar = objectIn[SFP(HStr_Key_MACAddress)];
    commaStringToArray(macAddressVar, macAddress, 6);

    JsonObjectConst schedulerObj = objectIn[SFP(HStr_Key_Scheduler)];
    if (!schedulerObj.isNull()) { scheduler.fromJSONObject(schedulerObj); }
    JsonObjectConst loggerObj = objectIn[SFP(HStr_Key_Logger)];
    if (!loggerObj.isNull()) { logger.fromJSONObject(loggerObj); }
    JsonObjectConst publisherObj = objectIn[SFP(HStr_Key_Publisher)];
    if (!publisherObj.isNull()) { publisher.fromJSONObject(publisherObj); }
}


HelioCalibrationData::HelioCalibrationData()
    : HelioData('H','C','A','L', 1),
      ownerName{0}, calibUnits(Helio_UnitsType_Undefined),
      multiplier(1.0f), offset(0.0f)
{
    _size = sizeof(*this);
    HELIO_HARD_ASSERT(isCalibrationData(), SFP(HStr_Err_OperationFailure));
}

HelioCalibrationData::HelioCalibrationData(HelioIdentity ownerId, Helio_UnitsType calibUnitsIn)
    : HelioData('H','C','A','L', 1),
      ownerName{0}, calibUnits(calibUnitsIn),
      multiplier(1.0f), offset(0.0f)
{
    _size = sizeof(*this);
    HELIO_HARD_ASSERT(isCalibrationData(), SFP(HStr_Err_OperationFailure));
    if (ownerId) {
        strncpy(ownerName, ownerId.keyString.c_str(), HELIO_NAME_MAXSIZE);
    }
}

void HelioCalibrationData::toJSONObject(JsonObject &objectOut) const
{
    HelioData::toJSONObject(objectOut);

    if (ownerName[0]) { objectOut[SFP(HStr_Key_SensorName)] = charsToString(ownerName, HELIO_NAME_MAXSIZE); }
    if (calibUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_CalibUnits)] = unitsTypeToSymbol(calibUnits); }
    objectOut[SFP(HStr_Key_Multiplier)] = multiplier;
    objectOut[SFP(HStr_Key_Offset)] = offset;
}

void HelioCalibrationData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioData::fromJSONObject(objectIn);

    const char *ownerNameStr = objectIn[SFP(HStr_Key_SensorName)];
    if (ownerNameStr && ownerNameStr[0]) { strncpy(ownerName, ownerNameStr, HELIO_NAME_MAXSIZE); }
    calibUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_CalibUnits)]);
    multiplier = objectIn[SFP(HStr_Key_Multiplier)] | multiplier;
    offset = objectIn[SFP(HStr_Key_Offset)] | offset;
}

void HelioCalibrationData::setFromTwoPoints(float point1MeasuredAt, float point1CalibratedTo,
                                                  float point2MeasuredAt, float point2CalibratedTo)
{
    float aTerm = point2CalibratedTo - point1CalibratedTo;
    float bTerm = point2MeasuredAt - point1MeasuredAt;
    HELIO_SOFT_ASSERT(!isFPEqual(bTerm, 0.0f), SFP(HStr_Err_InvalidParameter));
    if (!isFPEqual(bTerm, 0.0f)) {
        _bumpRevIfNotAlreadyModded();
        multiplier = aTerm / bTerm;
        offset = ((aTerm * point2MeasuredAt) + (bTerm * point1CalibratedTo)) / bTerm;
    }
}

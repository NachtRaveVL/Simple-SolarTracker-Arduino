/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Controller Modules
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


bool HelioObjectRegistration::registerObject(SharedPtr<HelioObject> obj)
{
    HELIO_SOFT_ASSERT(obj->getId().posIndex >= 0 && obj->getId().posIndex < HELIO_POS_MAXSIZE, SFP(HStr_Err_InvalidParameter));
    if (obj && _objects.find(obj->getKey()) == _objects.end()) {
        _objects[obj->getKey()] = obj;

        if (obj->isActuatorType() || obj->isPanelType()) {
            if (getScheduler()) {
                getScheduler()->setNeedsScheduling();
            }
        }
        if (obj->isSensorType()) {
            if (getPublisher()) {
                getPublisher()->setNeedsTabulation();
            }
        }

        return true;
    }
    return false;
}

bool HelioObjectRegistration::unregisterObject(SharedPtr<HelioObject> obj)
{
    auto iter = _objects.find(obj->getKey());
    if (iter != _objects.end()) {
        _objects.erase(iter);

        if (obj->isActuatorType() || obj->isPanelType()) {
            if (getScheduler()) {
                getScheduler()->setNeedsScheduling();
            }
        }
        if (obj->isSensorType()) {
            if (getPublisher()) {
                getPublisher()->setNeedsTabulation();
            }
        }

        return true;
    }
    return false;
}

SharedPtr<HelioObject> HelioObjectRegistration::objectById(HelioIdentity id) const
{
    if (id.posIndex == HELIO_POS_SEARCH_FROMBEG) {
        while (++id.posIndex < HELIO_POS_MAXSIZE) {
            auto iter = _objects.find(id.regenKey());
            if (iter != _objects.end()) {
                if (id.keyString == iter->second->getKeyString()) {
                    return iter->second;
                } else {
                    objectById_Col(id);
                }
            }
        }
    } else if (id.posIndex == HELIO_POS_SEARCH_FROMEND) {
        while (--id.posIndex >= 0) {
            auto iter = _objects.find(id.regenKey());
            if (iter != _objects.end()) {
                if (id.keyString == iter->second->getKeyString()) {
                    return iter->second;
                } else {
                    objectById_Col(id);
                }
            }
        }
    } else {
        auto iter = _objects.find(id.key);
        if (iter != _objects.end()) {
            if (id.keyString == iter->second->getKeyString()) {
                return iter->second;
            } else {
                objectById_Col(id);
            }
        }
    }

    return nullptr;
}

SharedPtr<HelioObject> HelioObjectRegistration::objectById_Col(const HelioIdentity &id) const
{
    HELIO_SOFT_ASSERT(false, F("Hashing collision")); // exhaustive search must be performed, publishing may miss values

    for (auto iter = _objects.begin(); iter != _objects.end(); ++iter) {
        if (id.keyString == iter->second->getKeyString()) {
            return iter->second;
        }
    }

    return nullptr;
}

hposi_t HelioObjectRegistration::firstPosition(HelioIdentity id, bool taken)
{
    if (id.posIndex != HELIO_POS_SEARCH_FROMEND) {
        id.posIndex = HELIO_POS_SEARCH_FROMBEG;
        while (++id.posIndex < HELIO_POS_MAXSIZE) {
            auto iter = _objects.find(id.regenKey());
            if (taken == (iter != _objects.end())) {
                return id.posIndex;
            }
        }
    } else {
        id.posIndex = HELIO_POS_SEARCH_FROMEND;
        while (--id.posIndex >= 0) {
            auto iter = _objects.find(id.regenKey());
            if (taken == (iter != _objects.end())) {
                return id.posIndex;
            }
        }
    }

    return -1;
}


bool HelioPinHandlers::tryGetPinLock(pintype_t pin, millis_t wait)
{
    millis_t start = millis();
    while (1) {
        auto iter = _pinLocks.find(pin);
        if (iter == _pinLocks.end()) {
            _pinLocks[pin] = true;
            return (_pinLocks.find(pin) != _pinLocks.end());
        }
        else if (millis() - start >= wait) { return false; }
        else { yield(); }
    }
}

void HelioPinHandlers::deactivatePinMuxers()
{
    for (auto iter = _pinMuxers.begin(); iter != _pinMuxers.end(); ++iter) {
        iter->second->deactivate();
    }
}

OneWire *HelioPinHandlers::getOneWireForPin(pintype_t pin)
{
    auto wireIter = _pinOneWire.find(pin);
    if (wireIter != _pinOneWire.end()) {
        return wireIter->second;
    } else {
        OneWire *oneWire = new OneWire(pin);
        if (oneWire) {
            _pinOneWire[pin] = oneWire;
            if (_pinOneWire.find(pin) != _pinOneWire.end()) { return oneWire; }
            else if (oneWire) { delete oneWire; }
        } else if (oneWire) { delete oneWire; }
    }
    return nullptr;
}

void HelioPinHandlers::dropOneWireForPin(pintype_t pin)
{
    auto wireIter = _pinOneWire.find(pin);
    if (wireIter != _pinOneWire.end()) {
        if (wireIter->second) {
            wireIter->second->depower();
            delete wireIter->second;
        }
        _pinOneWire.erase(wireIter);
    }
}

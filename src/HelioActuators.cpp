/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Actuators
*/

#include "Helioduino.h"

HelioActuator *newActuatorObjectFromData(const HelioActuatorData *dataIn)
{
    if (dataIn && dataIn->id.object.idType == -1) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && dataIn->isObjectData(), SFP(HStr_Err_InvalidParameter));

    if (dataIn && dataIn->isObjectData()) {
        switch (dataIn->id.object.classType) {
            case (int8_t)HelioActuator::Relay:
                return new HelioRelayActuator((const HelioActuatorData *)dataIn);
            case (int8_t)HelioActuator::Variable:
                return new HelioVariableActuator((const HelioActuatorData *)dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioActivationHandle::HelioActivationHandle(HelioActuator *actuator, Helio_DirectionMode directionIn, float intensityIn, millis_t forMillis, bool force)
    : actuator(actuator), direction(directionIn), intensity(constrain(intensityIn, 0.0f, 1.0f)), startMillis(0), durationMillis(forMillis), forced(force)
{
    if (actuator) { actuator->_handles.push_back(this); actuator->_needsUpdate = true; }
}

HelioActivationHandle::HelioActivationHandle(const HelioActivationHandle &handle)
    : actuator(handle.actuator), direction(handle.direction), intensity(constrain(handle.intensity, 0.0f, 1.0f)), forced(handle.forced),
      startMillis(handle.startMillis), durationMillis(handle.durationMillis)
{
    if (actuator) { actuator->_handles.push_back(this); }
}

HelioActivationHandle::~HelioActivationHandle()
{
    if (actuator) { actuator->_needsUpdate = true;
        for (auto handleIter = actuator->_handles.end() - 1; handleIter != actuator->_handles.begin() - 1; --handleIter) {
            if ((*handleIter) == this) {
                actuator->_handles.erase(handleIter);
                break;
            }
        }
        actuator = nullptr;
    }
}

HelioActivationHandle &HelioActivationHandle::operator=(const HelioActivationHandle &handle)
{
    if (handle.actuator != actuator) {
        if (actuator) { actuator->_needsUpdate = true;
            for (auto handleIter = actuator->_handles.end() - 1; handleIter != actuator->_handles.begin() - 1; --handleIter) {
                if ((*handleIter) == this) {
                    actuator->_handles.erase(handleIter);
                    break;
                }
            }
        }
        actuator = handle.actuator;
        if (actuator) { actuator->_handles.push_back(this); actuator->_needsUpdate = true; }
    }
    direction = handle.direction;
    intensity = constrain(handle.intensity, 0.0f, 1.0f);
    startMillis = 0;
    durationMillis = handle.durationMillis;
    forced = handle.forced;
    return *this;
}


HelioActuator::HelioActuator(Helio_ActuatorType actuatorType,
                             Helio_PositionIndex actuatorIndex,
                             int classTypeIn)
    : HelioObject(HelioIdentity(actuatorType, actuatorIndex)), classType((typeof(classType))classTypeIn),
      _enabled(false), _enableMode(Helio_EnableMode_Undefined), _rail(this), _panel(this)
{ ; }

HelioActuator::HelioActuator(const HelioActuatorData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))dataIn->id.object.classType),
      _enabled(false),
      _contPowerUsage(&(dataIn->contPowerUsage)),
      _rail(this), _panel(this)
{
    _rail.setObject(dataIn->railName);
    _panel.setObject(dataIn->panelName);
}

void HelioActuator::update()
{
    HelioObject::update();

    _rail.resolve();
    _panel.resolve();

    // Enablement checking
    bool canEnable = _handles.size() && getCanEnable();
    for (auto handleIter = _handles.begin(); handleIter != _handles.end() && !canEnable; ++handleIter) {
        canEnable = (*handleIter)->forced;
    }

    // If enabled and shouldn't be (unless force enabled)
    if ((_enabled || _needsUpdate) && !canEnable) {
        _disableActuator();
        _needsUpdate = false;
    } else if (canEnable && (!_enabled || _needsUpdate)) {
        float drivingIntensity = 0.0f; // Determine what driving intensity [-1,1] actuator should use

        switch (_enableMode) {
            case Helio_EnableMode_Highest:
            case Helio_EnableMode_DesOrder: {
                drivingIntensity = -__FLT_MAX__;
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    auto handleIntensity = (*handleIter)->getDriveIntensity();
                    if (handleIntensity > drivingIntensity) { drivingIntensity = handleIntensity; }
                }
            } break;

            case Helio_EnableMode_Lowest:
            case Helio_EnableMode_AscOrder: {
                drivingIntensity = __FLT_MAX__;
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    auto handleIntensity = (*handleIter)->getDriveIntensity();
                    if (handleIntensity < drivingIntensity) { drivingIntensity = handleIntensity; }
                }
            } break;

            case Helio_EnableMode_Average: {
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    drivingIntensity += (*handleIter)->getDriveIntensity();
                }
                drivingIntensity /= _handles.size();
            } break;

            case Helio_EnableMode_Multiply: {
                drivingIntensity = (*_handles.begin())->getDriveIntensity();
                for (auto handleIter = _handles.begin() + 1; handleIter != _handles.end(); ++handleIter) {
                    drivingIntensity *= (*handleIter)->getDriveIntensity();
                }
            } break;

            case Helio_EnableMode_InOrder: {
                drivingIntensity = (*_handles.begin())->getDriveIntensity();
            } break;

            case Helio_EnableMode_RevOrder: {
                drivingIntensity = (*(_handles.end() - 1))->getDriveIntensity();
            } break;

            default:
                break;
        }

        _enableActuator(constrain(drivingIntensity, -1.0f, 1.0f));
        _needsUpdate = false;
    }
}

bool HelioActuator::getCanEnable()
{
    if (getRail() && !getRail()->canActivate(this)) { return false; }
    if (getPanel() && !getPanel()->canActivate(this)) { return false; }
    return true;
}

void HelioActuator::setContinuousPowerUsage(float contPowerUsage, Helio_UnitsType contPowerUsageUnits)
{
    _contPowerUsage.value = contPowerUsage;
    _contPowerUsage.units = contPowerUsageUnits;
    _contPowerUsage.updateTimestamp();
    _contPowerUsage.updateFrame(1);
}

void HelioActuator::setContinuousPowerUsage(HelioSingleMeasurement contPowerUsage)
{
    _contPowerUsage = contPowerUsage;
    _contPowerUsage.setMinFrame(1);
}

const HelioSingleMeasurement &HelioActuator::getContinuousPowerUsage()
{
    return _contPowerUsage;
}

HelioAttachment &HelioActuator::getParentRail(bool resolve)
{
    if (resolve) { _rail.resolve(); }
    return _rail;
}

HelioAttachment &HelioActuator::getParentPanel(bool resolve)
{
    if (resolve) { _panel.resolve(); }
    return _panel;
}

Signal<HelioActuator *> &HelioActuator::getActivationSignal()
{
    return _activateSignal;
}

HelioData *HelioActuator::allocateData() const
{
    return _allocateDataForObjType((int8_t)_id.type, (int8_t)classType);
}

void HelioActuator::saveToData(HelioData *dataOut)
{
    HelioObject::saveToData(dataOut);

    dataOut->id.object.classType = (int8_t)classType;
    if (_contPowerUsage.frame) {
        _contPowerUsage.saveToData(&(((HelioActuatorData *)dataOut)->contPowerUsage));
    }
    if (_panel.getId()) {
        strncpy(((HelioActuatorData *)dataOut)->panelName, _panel.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_rail.getId()) {
        strncpy(((HelioActuatorData *)dataOut)->railName, _rail.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    ((HelioActuatorData *)dataOut)->enableMode = _enableMode;
}


HelioRelayActuator::HelioRelayActuator(Helio_ActuatorType actuatorType,
                                       Helio_PositionIndex actuatorIndex,
                                       HelioDigitalPin outputPin,
                                       int classType)
    : HelioActuator(actuatorType, actuatorIndex, classType),
      _outputPin(outputPin)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.deactivate();
}

HelioRelayActuator::HelioRelayActuator(const HelioActuatorData *dataIn)
    : HelioActuator(dataIn), _outputPin(&dataIn->outputPin)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.deactivate();
}

HelioRelayActuator::~HelioRelayActuator()
{
    if (_enabled) {
        _enabled = false;
        _outputPin.deactivate();
    }
}

bool HelioRelayActuator::_enableActuator(float intensity)
{
    if (_outputPin.isValid()) {
        bool enabledBefore = _enabled;

        if (!_enabled && !isFPEqual(intensity, 0.0f)) {
            _enabled = true;
            _outputPin.activate();
        }

        if (_enabled && !enabledBefore) {
            millis_t timeMillis = max(1, millis());
            for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                if ((*handleIter) && (*handleIter)->startMillis == 0) {
                    (*handleIter)->startMillis = timeMillis;
                }
            }

            getLoggerInstance()->logActivation(this);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
            #else
                _activateSignal.fire(this);
            #endif
        }
    }

    return _enabled;
}

void HelioRelayActuator::_disableActuator()
{
    if (_outputPin.isValid()) {
        bool enabledBefore = _enabled;

        if (_enabled) {
            _enabled = false;
            _outputPin.deactivate();
        }

        if (!_enabled && enabledBefore) {
            millis_t timeMillis = millis();
            for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                if ((*handleIter) && (*handleIter)->durationMillis) {
                    millis_t durationMillis = timeMillis - (*handleIter)->startMillis;
                    if (durationMillis >= (*handleIter)->durationMillis) {
                        (*handleIter)->durationMillis = 0;
                        (*handleIter)->actuator = nullptr;
                        handleIter = _handles.erase(handleIter) - 1;
                    } else {
                        (*handleIter)->durationMillis -= durationMillis;
                    }
                }
            }

            getLoggerInstance()->logDeactivation(this);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
            #else
                _activateSignal.fire(this);
            #endif
        }
    }
}

bool HelioRelayActuator::getCanEnable()
{
    if (HelioActuator::getCanEnable()) {
        // todo: adapt for panels
        return true;
    }
    return false;
}

bool HelioRelayActuator::isEnabled(float tolerance) const
{
    return _enabled;
}

void HelioRelayActuator::saveToData(HelioData *dataOut)
{
    HelioActuator::saveToData(dataOut);

    _outputPin.saveToData(&((HelioActuatorData *)dataOut)->outputPin);
}


HelioVariableActuator::HelioVariableActuator(Helio_ActuatorType actuatorType,
                                             Helio_PositionIndex actuatorIndex,
                                             HelioAnalogPin outputPin,
                                             int classType)
    : HelioActuator(actuatorType, actuatorIndex, classType),
      _outputPin(outputPin), _pwmAmount(0.0f)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.analogWrite_raw(0);
}

HelioVariableActuator::HelioVariableActuator(const HelioActuatorData *dataIn)
    : HelioActuator(dataIn),
      _outputPin(&dataIn->outputPin), _pwmAmount(0.0f)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.analogWrite_raw(0);
}

HelioVariableActuator::~HelioVariableActuator()
{
    if (_enabled) {
        _enabled = false;
        _outputPin.analogWrite_raw(0);
    }
}

bool HelioVariableActuator::_enableActuator(float intensity)
{
    if (_outputPin.isValid()) {
        bool enabledBefore = _enabled;

        if (!_enabled || (_enabled && !isFPEqual(_pwmAmount, intensity))) {
            _enabled = true;
            _pwmAmount = constrain(intensity, 0.0f, 1.0f);
            _outputPin.analogWrite(_pwmAmount);
        }

        if (_enabled && !enabledBefore) {
            getLoggerInstance()->logActivation(this);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
            #else
                _activateSignal.fire(this);
            #endif
        }
    }

    return _enabled;
}

void HelioVariableActuator::_disableActuator()
{
    if (_outputPin.isValid()) {
        bool enabledBefore = _enabled;

        if (_enabled) {
            _enabled = false;
            _outputPin.analogWrite_raw(0);
        }

        if (!_enabled && enabledBefore) {
            getLoggerInstance()->logDeactivation(this);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
            #else
                _activateSignal.fire(this);
            #endif
        }
    }
}

bool HelioVariableActuator::isEnabled(float tolerance) const
{
    return _enabled && _pwmAmount >= tolerance - FLT_EPSILON;
}

int HelioVariableActuator::getPWMAmount(int) const
{
    return _outputPin.bitRes.inverseTransform(_pwmAmount);
}

void HelioVariableActuator::setPWMAmount(float amount)
{
    _pwmAmount = constrain(amount, 0.0f, 1.0f);

    if (_enabled) { _outputPin.analogWrite(_pwmAmount); }
}

void HelioVariableActuator::setPWMAmount(int amount)
{
    _pwmAmount = _outputPin.bitRes.transform(amount);

    if (_enabled) { _outputPin.analogWrite(_pwmAmount); }
}

void HelioVariableActuator::saveToData(HelioData *dataOut)
{
    HelioActuator::saveToData(dataOut);

    _outputPin.saveToData(&((HelioActuatorData *)dataOut)->outputPin);
}


HelioActuatorData::HelioActuatorData()
    : HelioObjectData(), outputPin(), contPowerUsage(), railName{0}, panelName{0}, enableMode(Helio_EnableMode_Undefined)
{
    _size = sizeof(*this);
}

void HelioActuatorData::toJSONObject(JsonObject &objectOut) const
{
    HelioObjectData::toJSONObject(objectOut);

    if (isValidPin(outputPin.pin)) {
        JsonObject outputPinObj = objectOut.createNestedObject(SFP(HStr_Key_OutputPin));
        outputPin.toJSONObject(outputPinObj);
    }
    if (contPowerUsage.value > FLT_EPSILON) {
        JsonObject contPowerUsageObj = objectOut.createNestedObject(SFP(HStr_Key_ContPowerUsage));
        contPowerUsage.toJSONObject(contPowerUsageObj);
    }
    if (railName[0]) { objectOut[SFP(HStr_Key_RailName)] = charsToString(railName, HELIO_NAME_MAXSIZE); }
    if (panelName[0]) { objectOut[SFP(HStr_Key_PanelName)] = charsToString(panelName, HELIO_NAME_MAXSIZE); }
    if (enableMode != Helio_EnableMode_Undefined) { objectOut[SFP(HStr_Key_EnableMode)] = enableMode; }
}

void HelioActuatorData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioObjectData::fromJSONObject(objectIn);

    JsonObjectConst outputPinObj = objectIn[SFP(HStr_Key_OutputPin)];
    if (!outputPinObj.isNull()) { outputPin.fromJSONObject(outputPinObj); }
    JsonVariantConst contPowerUsageVar = objectIn[SFP(HStr_Key_ContPowerUsage)];
    if (!contPowerUsageVar.isNull()) { contPowerUsage.fromJSONVariant(contPowerUsageVar); }
    const char *railNameStr = objectIn[SFP(HStr_Key_RailName)];
    if (railNameStr && railNameStr[0]) { strncpy(railName, railNameStr, HELIO_NAME_MAXSIZE); }
    const char *panelNameStr = objectIn[SFP(HStr_Key_PanelName)];
    if (panelNameStr && panelNameStr[0]) { strncpy(panelName, panelNameStr, HELIO_NAME_MAXSIZE); }
    enableMode = objectIn[SFP(HStr_Key_EnableMode)] | enableMode;
}

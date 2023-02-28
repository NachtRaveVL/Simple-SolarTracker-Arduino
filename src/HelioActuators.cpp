/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Actuators
*/

#include "Helioduino.h"

HelioActuator *newActuatorObjectFromData(const HelioActuatorData *dataIn)
{
    if (dataIn && isValidType(dataIn->id.object.idType)) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && dataIn->isObjectData(), SFP(HStr_Err_InvalidParameter));

    if (dataIn && dataIn->isObjectData()) {
        switch (dataIn->id.object.classType) {
            case (hid_t)HelioActuator::Relay:
                return new HelioRelayActuator((const HelioActuatorData *)dataIn);
            case (hid_t)HelioActuator::RelayMotor:
                return new HelioRelayMotorActuator((const HelioMotorActuatorData *)dataIn);
            case (hid_t)HelioActuator::Variable:
                return new HelioVariableActuator((const HelioActuatorData *)dataIn);
            case (hid_t)HelioActuator::VariableMotor:
                //return new HelioVariableMotorActuator((const HelioMotorActuatorData *)dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioActuator::HelioActuator(Helio_ActuatorType actuatorType, hposi_t actuatorIndex, int classTypeIn)
    : HelioObject(HelioIdentity(actuatorType, actuatorIndex)), classType((typeof(classType))classTypeIn),
      _enabled(false), _enableMode(Helio_EnableMode_Undefined), _parentRail(this), _parentPanel(this), _needsUpdate(false)
{ ; }

HelioActuator::HelioActuator(const HelioActuatorData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))dataIn->id.object.classType),
      _enabled(false), _enableMode(dataIn->enableMode),
      _contPowerUsage(&(dataIn->contPowerUsage)),
      _parentRail(this), _parentPanel(this), _needsUpdate(false)
{
    _parentRail.initObject(dataIn->railName);
    _parentPanel.initObject(dataIn->panelName);
}

void HelioActuator::update()
{
    HelioObject::update();

    _parentRail.resolve();
    _parentPanel.resolve();

    millis_t time = nzMillis();

    // Update running handles and elapse them as needed, determine forced status, and remove invalid/finished handles
    bool forced = false;
    if (_handles.size()) {
        for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
            if (_enabled && (*handleIter)->isActive()) {
                (*handleIter)->elapseTo(time);
            }
            if ((*handleIter)->actuator.get() != this || !(*handleIter)->isValid() || (*handleIter)->isDone()) {
                if ((*handleIter)->actuator.get() == this) { (*handleIter)->actuator = nullptr; }
                handleIter = _handles.erase(handleIter) - 1;
                setNeedsUpdate();
                continue;
            }
            forced |= (*handleIter)->isForced();
        }
    }

    // Enablement checking
    bool canEnable = _handles.size() && (forced || getCanEnable());

    if (!canEnable && (_enabled || _needsUpdate)) { // If enabled and shouldn't be (unless force enabled)
        _disableActuator();
    } else if (canEnable && (!_enabled || _needsUpdate)) { // If can enable and isn't (maybe force enabled)
        float drivingIntensity = 0.0f;

        // Determine what driving intensity [-1,1] actuator should use
        switch (_enableMode) {
            case Helio_EnableMode_Highest:
            case Helio_EnableMode_DescOrder: {
                drivingIntensity = -__FLT_MAX__;
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    if ((*handleIter)->isValid() && !(*handleIter)->isDone()) {
                        auto handleIntensity = (*handleIter)->getDriveIntensity();
                        if (handleIntensity > drivingIntensity) { drivingIntensity = handleIntensity; }
                    }
                }
            } break;

            case Helio_EnableMode_Lowest:
            case Helio_EnableMode_AscOrder: {
                drivingIntensity = __FLT_MAX__;
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    if ((*handleIter)->isValid() && !(*handleIter)->isDone()) {
                        auto handleIntensity = (*handleIter)->getDriveIntensity();
                        if (handleIntensity < drivingIntensity) { drivingIntensity = handleIntensity; }
                    }
                }
            } break;

            case Helio_EnableMode_Average: {
                int handleCount = 0;
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    if ((*handleIter)->isValid() && !(*handleIter)->isDone()) {
                        drivingIntensity += (*handleIter)->getDriveIntensity();
                        ++handleCount;
                    }
                }
                if (handleCount) { drivingIntensity /= handleCount; }
            } break;

            case Helio_EnableMode_Multiply: {
                drivingIntensity = (*_handles.begin())->getDriveIntensity();
                for (auto handleIter = _handles.begin() + 1; handleIter != _handles.end(); ++handleIter) {
                    if ((*handleIter)->isValid() && !(*handleIter)->isDone()) {
                        drivingIntensity *= (*handleIter)->getDriveIntensity();
                    }
                }
            } break;

            case Helio_EnableMode_InOrder: {
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    if ((*handleIter)->isValid() && !(*handleIter)->isDone()) {
                        drivingIntensity += (*handleIter)->getDriveIntensity();
                        break;
                    }
                }
            } break;

            case Helio_EnableMode_RevOrder: {
                for (auto handleIter = _handles.end() - 1; handleIter != _handles.begin() - 1; --handleIter) {
                    if ((*handleIter)->isValid() && !(*handleIter)->isDone()) {
                        drivingIntensity += (*handleIter)->getDriveIntensity();
                        break;
                    }
                }
            } break;

            default:
                break;
        }

        // Enable/disable activation handles as needed (serial modes only select 1 at a time)
        switch (_enableMode) {
            case Helio_EnableMode_InOrder:
            case Helio_EnableMode_DescOrder: {
                bool selected = false;
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    if (!selected && (*handleIter)->isValid() && !(*handleIter)->isDone() && isFPEqual((*handleIter)->activation.intensity, getDriveIntensity())) {
                        selected = true; (*handleIter)->checkTime = time;
                    } else if ((*handleIter)->checkTime != 0) {
                        (*handleIter)->checkTime = 0;
                    }
                }
            } break;

            case Helio_EnableMode_RevOrder:
            case Helio_EnableMode_AscOrder: {
                bool selected = false;
                for (auto handleIter = _handles.end() - 1; handleIter != _handles.begin() - 1; --handleIter) {
                    if (!selected && (*handleIter)->isValid() && !(*handleIter)->isDone() && isFPEqual((*handleIter)->activation.intensity, getDriveIntensity())) {
                        selected = true; (*handleIter)->checkTime = time;
                    } else if ((*handleIter)->checkTime != 0) {
                        (*handleIter)->checkTime = 0;
                    }
                }
            } break;

            default: {
                for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
                    if ((*handleIter)->isValid() && !(*handleIter)->isDone() && (*handleIter)->checkTime == 0) {
                        (*handleIter)->checkTime = time;
                    }
                }
            } break;
        }

        _enableActuator(drivingIntensity);
    }
    _needsUpdate = false;
}

bool HelioActuator::getCanEnable()
{
    if (getParentRail() && !getParentRail()->canActivate(this)) { return false; }
    if (getParentPanel() && !getParentPanel()->canActivate(this)) { return false; }
    return true;
}

void HelioActuator::setContinuousPowerUsage(HelioSingleMeasurement contPowerUsage)
{
    _contPowerUsage = contPowerUsage;
    _contPowerUsage.setMinFrame(1);
    bumpRevisionIfNeeded();
}

const HelioSingleMeasurement &HelioActuator::getContinuousPowerUsage()
{
    return _contPowerUsage;
}

HelioAttachment &HelioActuator::getParentRailAttachment()
{
    return _parentRail;
}

HelioAttachment &HelioActuator::getParentPanelAttachment()
{
    return _parentPanel;
}

void HelioActuator::setUserCalibrationData(HelioCalibrationData *userCalibrationData)
{
    if (_calibrationData && _calibrationData != userCalibrationData) { bumpRevisionIfNeeded(); }
    if (getController()) {
        if (userCalibrationData && getController()->setUserCalibrationData(userCalibrationData)) {
            _calibrationData = getController()->getUserCalibrationData(_id.key);
        } else if (!userCalibrationData && _calibrationData && getController()->dropUserCalibrationData(_calibrationData)) {
            _calibrationData = nullptr;
        }
    } else {
        _calibrationData = userCalibrationData;
    }
}

Pair<float,float> HelioActuator::getTravelRange() const
{
    if (isServoType()) {
        return make_pair<float,float>(calibrationTransform(0.025f), calibrationTransform(0.125f));
    } else if (isMotorType()) {
        return make_pair<float,float>(0.0f, FLT_UNDEF);
    } else {
        return make_pair<float,float>(calibrationTransform(0.0f), calibrationTransform(1.0f));
    }
}

Signal<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS> &HelioActuator::getActivationSignal()
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
    if (_contPowerUsage.isSet()) {
        _contPowerUsage.saveToData(&(((HelioActuatorData *)dataOut)->contPowerUsage));
    }
    if (_parentPanel.isSet()) {
        strncpy(((HelioActuatorData *)dataOut)->panelName, _parentPanel.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_parentRail.isSet()) {
        strncpy(((HelioActuatorData *)dataOut)->railName, _parentRail.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    ((HelioActuatorData *)dataOut)->enableMode = _enableMode;
}

void HelioActuator::handleActivation()
{
    if (_enabled) {
        getLogger()->logActivation(this);
    } else {
        for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
            if ((*handleIter)->checkTime) { (*handleIter)->checkTime = 0; }
        }

        getLogger()->logDeactivation(this);
    }

    #ifdef HELIO_USE_MULTITASKING
        scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
    #else
        _activateSignal.fire(this);
    #endif
}


HelioRelayActuator::HelioRelayActuator(Helio_ActuatorType actuatorType, hposi_t actuatorIndex, HelioDigitalPin outputPin, int classType)
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

bool HelioRelayActuator::getCanEnable()
{
    return _outputPin.isValid() && HelioActuator::getCanEnable();
}

float HelioRelayActuator::getDriveIntensity() const
{
    return _enabled ? 1.0f : 0.0f;
}

bool HelioRelayActuator::isEnabled(float tolerance) const
{
    return _enabled;
}

void HelioRelayActuator::_enableActuator(float intensity)
{
    bool wasEnabled = _enabled;
    //intensity = roundf(intensity); // unnecessary frn

    if (_outputPin.isValid()) {
        if (intensity > FLT_EPSILON) {
            _enabled = true;
            _outputPin.activate();
        } else {
            _outputPin.deactivate();
        }

        if (!wasEnabled) { handleActivation(); }
    }
}

void HelioRelayActuator::_disableActuator()
{
    bool wasEnabled = _enabled;

    if (_outputPin.isValid()) {
        _enabled = false;
        _outputPin.deactivate();

        if (wasEnabled) { handleActivation(); }
    }
}

void HelioRelayActuator::saveToData(HelioData *dataOut)
{
    HelioActuator::saveToData(dataOut);

    _outputPin.saveToData(&((HelioActuatorData *)dataOut)->outputPin);
}


HelioRelayMotorActuator::HelioRelayMotorActuator(Helio_ActuatorType actuatorType, hposi_t actuatorIndex, HelioDigitalPin outputPinA, HelioDigitalPin outputPinB, Pair<float,float> travelRange, int classType)
    : HelioRelayActuator(actuatorType, actuatorIndex, outputPinA, classType),
      HelioDistanceUnitsInterfaceStorage(defaultDistanceUnits()),
      _outputPin2(outputPinB), _travelRange(travelRange), _position(this), _speed(this), _minimum(this), _maximum(this),
      _travelPosStart(0.0f), _travelDistAccum(0.0f), _travelTimeStart(0), _travelTimeAccum(0)
{
    _position.setMeasurementUnits(getDistanceUnits());
    _speed.setMeasurementUnits(getSpeedUnits());

    _minimum.setHandleMethod(&HelioRelayMotorActuator::handleMinimumTrigger);
    _maximum.setHandleMethod(&HelioRelayMotorActuator::handleMaximumTrigger);
}

HelioRelayMotorActuator::HelioRelayMotorActuator(const HelioMotorActuatorData *dataIn)
    : HelioRelayActuator(dataIn),
      HelioDistanceUnitsInterfaceStorage(definedUnitsElse(dataIn->distanceUnits, defaultDistanceUnits())),
      _outputPin2(&dataIn->outputPin2), _travelRange(make_pair(dataIn->travelRange[0], dataIn->travelRange[1])),
      _position(this), _speed(this), _minimum(this), _maximum(this),
      _contSpeed(&(dataIn->contSpeed)),
      _travelPosStart(0.0f), _travelDistAccum(0.0f), _travelTimeStart(0), _travelTimeAccum(0)
{
    _position.setMeasurementUnits(getDistanceUnits());
    _position.initObject(dataIn->positionSensor);

    _speed.setMeasurementUnits(getSpeedUnits());
    _speed.initObject(dataIn->speedSensor);

    _minimum.setHandleMethod(&HelioRelayMotorActuator::handleMinimumTrigger);
    _maximum.setHandleMethod(&HelioRelayMotorActuator::handleMaximumTrigger);

    _minimum = newTriggerObjectFromSubData(&(dataIn->minTrigger));
    HELIO_SOFT_ASSERT(_minimum, SFP(HStr_Err_AllocationFailure));
    _maximum = newTriggerObjectFromSubData(&(dataIn->maxTrigger));
    HELIO_SOFT_ASSERT(_maximum, SFP(HStr_Err_AllocationFailure));
}

void HelioRelayMotorActuator::update()
{
    HelioActuator::update();

    _position.updateIfNeeded(true);
    _speed.updateIfNeeded(true);
    _minimum.updateIfNeeded(true);
    _maximum.updateIfNeeded(true);

    if (_travelTimeStart) {
        millis_t time = nzMillis();
        millis_t duration = time - _travelTimeStart;
        if (duration >= HELIO_ACT_TRAVELCALC_UPDATEMS) {
            handleTravelTime(time);
        }
    }
}

SharedPtr<HelioObjInterface> HelioRelayMotorActuator::getSharedPtrFor(const HelioObjInterface *obj) const
{
    return obj->getKey() == _minimum.getKey() ? _minimum.getSharedPtrFor(obj) :
           obj->getKey() == _maximum.getKey() ? _maximum.getSharedPtrFor(obj) :
           HelioObject::getSharedPtrFor(obj);
}

bool HelioRelayMotorActuator::getCanEnable()
{
    if (_outputPin2.isValid() && HelioRelayActuator::getCanEnable()) {
        return (!isMinTravel(true) || _intensity >= 0) &&
               (!isMaxTravel(true) || _intensity <= 0);
    }
    return false;
}

float HelioRelayMotorActuator::getDriveIntensity() const
{
    return _intensity;
}

void HelioRelayMotorActuator::_enableActuator(float intensity)
{
    bool wasEnabled = _enabled;
    intensity = constrain(intensity, -1.0f, 1.0f);

    if (_outputPin.isValid() && _outputPin2.isValid()) {
        _intensity = intensity;
        if (!isFPEqual(_intensity, 0.0f)) {
            _enabled = true;
            if (_intensity > 0) {
                _outputPin.activate();
                _outputPin2.deactivate();
            } else {
                _outputPin.deactivate();
                _outputPin2.activate();
            }
        } else {
            _outputPin.deactivate();
            _outputPin2.deactivate();
        }

        if (!wasEnabled) { handleActivation(); }
    }
}

void HelioRelayMotorActuator::_disableActuator()
{
    bool wasEnabled = _enabled;

    if (_outputPin.isValid() && _outputPin2.isValid()) {
        _enabled = false;
        _outputPin.deactivate();
        _outputPin2.deactivate();

        if (wasEnabled) { handleActivation(); }
    }
}

void HelioRelayMotorActuator::handleActivation()
{
    millis_t time = nzMillis();
    HelioActuator::handleActivation();

    if (_enabled) {
        auto position = _position.getMeasurement(true);

        _travelDistAccum = 0;
        _travelTimeStart = _travelTimeAccum = time;
        _travelPosStart = position.value;
    } else {
        if (_travelTimeAccum < time) { handleTravelTime(time); }
        _travelTimeAccum = 0;
        float duration = time - _travelTimeStart;

        getLogger()->logStatus(this, SFP(HStr_Log_MeasuredTravel));
        if (getParentPanel()) { getLogger()->logMessage(SFP(HStr_Log_Field_Solar_Panel), getParentPanel()->getKeyString()); }
        getLogger()->logMessage(SFP(HStr_Log_Field_Travel_Measured), measurementToString(_travelDistAccum, baseUnits(getSpeedUnits()), 1));
        getLogger()->logMessage(SFP(HStr_Log_Field_Time_Measured), roundToString(duration / 1000.0f, 1), String('s'));
    }
}

bool HelioRelayMotorActuator::canTravel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits)
{
    if (getParentPanel() && _contSpeed.value > FLT_EPSILON) {
        convertUnits(&distance, &distanceUnits, getDistanceUnits());
        HelioSingleMeasurement position = getPositionSensorAttachment().getMeasurement();
        position.value += (direction != Helio_DirectionMode_Reverse ? distance : -distance);
        return position.value >= _travelRange.first - FLT_EPSILON && position.value <= _travelRange.second + FLT_EPSILON;
    }
    return false;
}

HelioActivationHandle HelioRelayMotorActuator::travel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits)
{
    if (getParentPanel() && _contSpeed.value > FLT_EPSILON) {
        convertUnits(&distance, &distanceUnits, getDistanceUnits());
        return travel(direction, (millis_t)((distance / _contSpeed.value) * secondsToMillis(SECS_PER_MIN)));
    }
    return HelioActivationHandle();
}

bool HelioRelayMotorActuator::canTravel(Helio_DirectionMode direction, millis_t time)
{
    if (getParentPanel() && _contSpeed.value > FLT_EPSILON) {
        return canTravel(direction, _contSpeed.value * (time / (float)secondsToMillis(SECS_PER_MIN)), getDistanceUnits());
    }
    return false;
}

HelioActivationHandle HelioRelayMotorActuator::travel(Helio_DirectionMode direction, millis_t time)
{
    if (getParentPanel()) {
        #ifdef HELIO_USE_MULTITASKING
            getLogger()->logStatus(this, SFP(HStr_Log_CalculatedTravel));
            if (getParentPanel()) { getLogger()->logMessage(SFP(HStr_Log_Field_Solar_Panel), getParentPanel()->getKeyString()); }
            if (_contSpeed.value > FLT_EPSILON) {
                getLogger()->logMessage(SFP(HStr_Log_Field_Travel_Calculated), measurementToString(_contSpeed.value * (time / (float)secondsToMillis(SECS_PER_MIN)), baseUnits(getSpeedUnits()), 1));
            }
            getLogger()->logMessage(SFP(HStr_Log_Field_Time_Calculated), roundToString(time / 1000.0f, 1), String('s'));
            return enableActuator(direction, 1.0f, time);
        #else
            getLogger()->logStatus(this, SFP(HStr_Log_CalculatedTravel));
            if (getPanel()) { getLogger()->logMessage(SFP(HStr_Log_Field_Solar_Panel), getPanel()->getKeyString()); }
            if (_contSpeed.value > FLT_EPSILON) {
                getLogger()->logMessage(SFP(HStr_Log_Field_Travel_Calculated), measurementToString(_contSpeed.value * (time / (float)secondsToMillis(SECS_PER_MIN)), baseUnits(getSpeedUnits()), 1));
            }
            getLogger()->logMessage(SFP(HStr_Log_Field_Time_Calculated), roundToString(time / 1000.0f, 1), String('s'));
            return enableActuator(direction, 1.0f, time);
        #endif
    }
    return HelioActivationHandle();
}

void HelioRelayMotorActuator::setDistanceUnits(Helio_UnitsType distanceUnits)
{
    if (_distUnits != distanceUnits) {
        _distUnits = distanceUnits;

        convertUnits(&_contSpeed, getSpeedUnits());
        _position.setMeasurementUnits(getDistanceUnits());
        _speed.setMeasurementUnits(getSpeedUnits());
        bumpRevisionIfNeeded();
    }
}

void HelioRelayMotorActuator::setContinuousSpeed(HelioSingleMeasurement contSpeed)
{
    _contSpeed = contSpeed;
    _contSpeed.setMinFrame(1);

    convertUnits(&_contSpeed, getSpeedUnits());
    bumpRevisionIfNeeded();
}

const HelioSingleMeasurement &HelioRelayMotorActuator::getContinuousSpeed()
{
    return _contSpeed;
}

Pair<float,float> HelioRelayMotorActuator::getTravelRange() const
{
    return _travelRange;
}

bool HelioRelayMotorActuator::isMinTravel(bool poll)
{
    if (triggerStateToBool(_minimum.getTriggerState(poll))) { return true; }
    return _position.getMeasurementValue(poll) <= _travelRange.first + FLT_EPSILON;
}

bool HelioRelayMotorActuator::isMaxTravel(bool poll)
{
    if (triggerStateToBool(_maximum.getTriggerState(poll))) { return true; }
    return _position.getMeasurementValue(poll) >= _travelRange.second - FLT_EPSILON;
}

HelioSensorAttachment &HelioRelayMotorActuator::getPositionSensorAttachment()
{
    return _position;
}

HelioSensorAttachment &HelioRelayMotorActuator::getSpeedSensorAttachment()
{
    return _speed;
}

HelioTriggerAttachment &HelioRelayMotorActuator::getMinimumTriggerAttachment()
{
    return _minimum;
}

HelioTriggerAttachment &HelioRelayMotorActuator::getMaximumTriggerAttachment()
{
    return _maximum;
}

void HelioRelayMotorActuator::saveToData(HelioData *dataOut)
{
    HelioRelayActuator::saveToData(dataOut);

    _outputPin2.saveToData(&((HelioMotorActuatorData *)dataOut)->outputPin2);
    ((HelioMotorActuatorData *)dataOut)->distanceUnits = _distUnits;
    if (_contSpeed.isSet()) {
        _contSpeed.saveToData(&(((HelioMotorActuatorData *)dataOut)->contSpeed));
    }
    if (_position.isSet()) {
        strncpy(((HelioMotorActuatorData *)dataOut)->positionSensor, _position.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_speed.isSet()) {
        strncpy(((HelioMotorActuatorData *)dataOut)->speedSensor, _speed.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
}

void HelioRelayMotorActuator::handleTravelTime(millis_t time)
{
    if (getPositionSensor(true)) {
        auto position = _position.getMeasurement();
        auto travelDistTotal = (position.value - _travelPosStart);

        if (!getSpeedSensor()) {
            auto timeDelta = (time - _travelTimeAccum) / (float)secondsToMillis(SECS_PER_MIN);
            auto distDelta = travelDistTotal - _travelDistAccum;
            _speed.setMeasurement(distDelta / timeDelta);
        }

        _travelDistAccum = travelDistTotal;
    } else {
        auto speed = getSpeedSensor(true) ? _speed.getMeasurement() : _contSpeed;

        if (speed.value >= (_contSpeed.value * HELIO_ACT_TRAVELCALC_MINSPEED) - FLT_EPSILON) {        
            auto timeDelta = (time - _travelTimeAccum) / (float)secondsToMillis(SECS_PER_MIN);
            auto distDelta = speed.value * timeDelta;
            _travelDistAccum += distDelta;
            _position.setMeasurement(_travelPosStart + _travelDistAccum);
        }
    }

    _travelTimeAccum = time;
}

void HelioRelayMotorActuator::handleMinimumTrigger(Helio_TriggerState minimumTrigger)
{
    if (triggerStateToBool(minimumTrigger) && !getPositionSensor()) {
        getPositionSensorAttachment().setMeasurement(_travelRange.first, getDistanceUnits());
    }
}

void HelioRelayMotorActuator::handleMaximumTrigger(Helio_TriggerState maximumTrigger)
{
    if (triggerStateToBool(maximumTrigger) && !getPositionSensor()) {
        getPositionSensorAttachment().setMeasurement(_travelRange.second, getDistanceUnits());
    }
}


HelioVariableActuator::HelioVariableActuator(Helio_ActuatorType actuatorType, hposi_t actuatorIndex, HelioAnalogPin outputPin, int classType)
    : HelioActuator(actuatorType, actuatorIndex, classType),
      _outputPin(outputPin), _intensity(0.0f)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.analogWrite_raw(0);
}

HelioVariableActuator::HelioVariableActuator(const HelioActuatorData *dataIn)
    : HelioActuator(dataIn),
      _outputPin(&dataIn->outputPin), _intensity(0.0f)
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

bool HelioVariableActuator::getCanEnable()
{
    return _outputPin.isValid() && HelioActuator::getCanEnable();
}

float HelioVariableActuator::getDriveIntensity() const
{
    return _intensity;
}

bool HelioVariableActuator::isEnabled(float tolerance) const
{
    return _enabled && _intensity >= tolerance - FLT_EPSILON;
}

void HelioVariableActuator::_enableActuator(float intensity)
{
    bool wasEnabled = _enabled;
    intensity = constrain(intensity, 0.0f, 1.0f);

    if (_outputPin.isValid()) {
        _enabled = true;
        _outputPin.analogWrite((_intensity = intensity));

        if (!wasEnabled) { handleActivation(); }
    }
}

void HelioVariableActuator::_disableActuator()
{
    bool wasEnabled = _enabled;

    if (_outputPin.isValid()) {
        _enabled = false;
        _outputPin.analogWrite_raw(0);

        if (wasEnabled) { handleActivation(); }
    }
}

void HelioVariableActuator::saveToData(HelioData *dataOut)
{
    HelioActuator::saveToData(dataOut);

    _outputPin.saveToData(&((HelioActuatorData *)dataOut)->outputPin);
}


HelioActuatorData::HelioActuatorData()
    : HelioObjectData(), outputPin(), enableMode(Helio_EnableMode_Undefined), contPowerUsage(), railName{0}, panelName{0}
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
    if (enableMode != Helio_EnableMode_Undefined) { objectOut[SFP(HStr_Key_EnableMode)] = enableModeToString(enableMode); }
    if (contPowerUsage.value > FLT_EPSILON) {
        JsonObject contPowerUsageObj = objectOut.createNestedObject(SFP(HStr_Key_ContinuousPowerUsage));
        contPowerUsage.toJSONObject(contPowerUsageObj);
    }
    if (railName[0]) { objectOut[SFP(HStr_Key_RailName)] = charsToString(railName, HELIO_NAME_MAXSIZE); }
    if (panelName[0]) { objectOut[SFP(HStr_Key_PanelName)] = charsToString(panelName, HELIO_NAME_MAXSIZE); }
}

void HelioActuatorData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioObjectData::fromJSONObject(objectIn);

    JsonObjectConst outputPinObj = objectIn[SFP(HStr_Key_OutputPin)];
    if (!outputPinObj.isNull()) { outputPin.fromJSONObject(outputPinObj); }
    enableMode = enableModeFromString(objectIn[SFP(HStr_Key_EnableMode)]);
    JsonVariantConst contPowerUsageVar = objectIn[SFP(HStr_Key_ContinuousPowerUsage)];
    if (!contPowerUsageVar.isNull()) { contPowerUsage.fromJSONVariant(contPowerUsageVar); }
    const char *railNameStr = objectIn[SFP(HStr_Key_RailName)];
    if (railNameStr && railNameStr[0]) { strncpy(railName, railNameStr, HELIO_NAME_MAXSIZE); }
    const char *panelNameStr = objectIn[SFP(HStr_Key_PanelName)];
    if (panelNameStr && panelNameStr[0]) { strncpy(panelName, panelNameStr, HELIO_NAME_MAXSIZE); }
}

HelioMotorActuatorData::HelioMotorActuatorData()
    : HelioActuatorData(), outputPin2(), distanceUnits(Helio_UnitsType_Undefined), contSpeed(), positionSensor{0}, speedSensor{0}
{
    _size = sizeof(*this);
}

void HelioMotorActuatorData::toJSONObject(JsonObject &objectOut) const
{
    HelioActuatorData::toJSONObject(objectOut);

    if (isValidPin(outputPin2.pin)) {
        JsonObject outputPin2Obj = objectOut.createNestedObject(SFP(HStr_Key_OutputPin2));
        outputPin2.toJSONObject(outputPin2Obj);
    }
    if (distanceUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_DistanceUnits)] = unitsTypeToSymbol(distanceUnits); }
    if (contSpeed.value > FLT_EPSILON) {
        JsonObject contSpeedObj = objectOut.createNestedObject(SFP(HStr_Key_ContinuousSpeed));
        contSpeed.toJSONObject(contSpeedObj);
    }
    if (positionSensor[0]) { objectOut[SFP(HStr_Key_PositionSensor)] = charsToString(positionSensor, HELIO_NAME_MAXSIZE); }
    if (speedSensor[0]) { objectOut[SFP(HStr_Key_SpeedSensor)] = charsToString(speedSensor, HELIO_NAME_MAXSIZE); }
}

void HelioMotorActuatorData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioActuatorData::fromJSONObject(objectIn);

    JsonObjectConst outputPinObj = objectIn[SFP(HStr_Key_OutputPin)];
    if (!outputPinObj.isNull()) { outputPin.fromJSONObject(outputPinObj); }
    distanceUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_DistanceUnits)]);
    JsonVariantConst contSpeedVar = objectIn[SFP(HStr_Key_ContinuousSpeed)];
    if (!contSpeedVar.isNull()) { contSpeed.fromJSONVariant(contSpeedVar); }
    const char *positionSensorStr = objectIn[SFP(HStr_Key_PositionSensor)];
    if (positionSensorStr && positionSensorStr[0]) { strncpy(positionSensor, positionSensorStr, HELIO_NAME_MAXSIZE); }
    const char *speedSensorStr = objectIn[SFP(HStr_Key_SpeedSensor)];
    if (speedSensorStr && speedSensorStr[0]) { strncpy(speedSensor, speedSensorStr, HELIO_NAME_MAXSIZE); }
}

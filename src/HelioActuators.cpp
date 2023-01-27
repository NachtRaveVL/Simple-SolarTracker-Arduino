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


HelioActivationHandle::HelioActivationHandle(HelioActuator *actuator, Helio_DirectionMode directionIn, float intensityIn, millis_t duration, bool force)
    : actuator(actuator), direction(directionIn), intensity(constrain(intensityIn, 0.0f, 1.0f)), start(0), duration(duration), forced(force)
{
    if (actuator) { actuator->_handles.push_back(this); actuator->_needsUpdate = true; }
}

HelioActivationHandle::HelioActivationHandle(const HelioActivationHandle &handle)
    : actuator(handle.actuator), direction(handle.direction), intensity(constrain(handle.intensity, 0.0f, 1.0f)), forced(handle.forced),
      start(handle.start), duration(handle.duration)
{
    if (actuator) { actuator->_handles.push_back(this); actuator->_needsUpdate = true; }
}

HelioActivationHandle::~HelioActivationHandle()
{
    unset();
}

HelioActivationHandle &HelioActivationHandle::operator=(const HelioActivationHandle &handle)
{
    if (handle.actuator != actuator) {
        unset();
        actuator = handle.actuator;
        if (actuator) { actuator->_handles.push_back(this); actuator->_needsUpdate = true; }
    } else { start = 0; }
    direction = handle.direction;
    intensity = constrain(handle.intensity, 0.0f, 1.0f);
    duration = handle.duration;
    forced = handle.forced;
    return *this;
}

void HelioActivationHandle::unset()
{
    if (actuator) {
        for (auto handleIter = actuator->_handles.end() - 1; handleIter != actuator->_handles.begin() - 1; --handleIter) {
            if ((*handleIter) == this) {
                actuator->_handles.erase(handleIter); actuator->_needsUpdate = true;
                break;
            }
        }
        actuator = nullptr;
    }
    start = duration = 0;
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

        _enableActuator(drivingIntensity);
        _needsUpdate = false;
    }
}

bool HelioActuator::getCanEnable()
{
    if (getRail() && !getRail()->canActivate(this)) { return false; }
    if (getPanel() && !getPanel()->canActivate(this)) { return false; }
    return true;
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

void HelioActuator::handleActivation()
{
    millis_t time = millis();

    if (_enabled) {
        for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
            if ((*handleIter)->start == 0) {
                (*handleIter)->start = max(1, time);
            }
        }

        getLoggerInstance()->logActivation(this);
    } else {
        for (auto handleIter = _handles.begin(); handleIter != _handles.end(); ++handleIter) {
            if ((*handleIter)->duration) {
                millis_t duration = time - (*handleIter)->start;

                if (duration >= (*handleIter)->duration) {
                    (*handleIter)->duration = 0;
                    (*handleIter)->actuator = nullptr;
                    handleIter = _handles.erase(handleIter) - 1;
                } else {
                    (*handleIter)->duration -= duration;
                }
            }
        }

        getLoggerInstance()->logDeactivation(this);
    }

    #ifdef HELIO_USE_MULTITASKING
        scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
    #else
        _activateSignal.fire(this);
    #endif
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

bool HelioRelayActuator::getCanEnable()
{
    return _outputPin.isValid() && HelioActuator::getCanEnable();
}

bool HelioRelayActuator::isEnabled(float tolerance) const
{
    return _enabled;
}

bool HelioRelayActuator::_enableActuator(float intensity)
{
    bool wasEnabled = _enabled;

    if (_outputPin.isValid() && !_enabled) {
        if (!isFPEqual(intensity, 0.0f)) {
            _enabled = true;
            _outputPin.activate();
            if (!wasEnabled) { handleActivation(); }
        } else {
            _outputPin.deactivate();
        }
    }

    return _enabled;
}

void HelioRelayActuator::_disableActuator()
{
    if (_outputPin.isValid() && _enabled) {
        _enabled = false;
        _outputPin.deactivate();
        handleActivation();
    }
}

void HelioRelayActuator::saveToData(HelioData *dataOut)
{
    HelioActuator::saveToData(dataOut);

    _outputPin.saveToData(&((HelioActuatorData *)dataOut)->outputPin);
}


HelioRelayMotorActuator::HelioRelayMotorActuator(Helio_ActuatorType actuatorType,
                                                 Helio_PositionIndex actuatorIndex,
                                                 HelioDigitalPin forwardOutputPin,
                                                 HelioDigitalPin reverseOutputPin,
                                                 int classType)
    :  HelioRelayActuator(actuatorType, actuatorIndex, forwardOutputPin, classType), _outputPin2(reverseOutputPin),
       _distanceUnits(defaultDistanceUnits()), _speedUnits(defaultSpeedUnits()), _position(this), _speed(this),
       _travelDistanceAccum(0.0f), _travelTimeStart(0), _travelTimeAccum(0)
{
    _position.setMeasurementUnits(getDistanceUnits());
    _speed.setMeasurementUnits(getSpeedUnits());
}

HelioRelayMotorActuator::HelioRelayMotorActuator(const HelioMotorActuatorData *dataIn)
    : HelioRelayActuator(dataIn), _travelDistanceAccum(0.0f), _travelTimeStart(0), _travelTimeAccum(0),
      _distanceUnits(definedUnitsElse(dataIn->distanceUnits, defaultDistanceUnits())),
      _speedUnits(definedUnitsElse(dataIn->speedUnits, defaultSpeedUnits())),
      _contSpeed(&(dataIn->contSpeed)),
      _position(this), _speed(this)
{
    _position.setMeasurementUnits(getDistanceUnits());
    _speed.setMeasurementUnits(getSpeedUnits());

    _position.setObject(dataIn->positionSensor);
    _speed.setObject(dataIn->speedSensor);
}

void HelioRelayMotorActuator::update()
{
    HelioActuator::update();

    _position.updateIfNeeded(true);

    _speed.updateIfNeeded(true);

    if (_travelTimeAccum) {
        millis_t time = millis();
        millis_t motor = time - _travelTimeAccum;
        if (motor >= HELIO_ACT_TRAVELCALC_MINWRTMILLIS) {
            handleTravelTime(motor);
            _travelTimeAccum = max(1, time);
        }
    }

    if (_enabled) { pollTravelingSensors(); }
}

bool HelioRelayMotorActuator::getCanEnable()
{
    if (_outputPin2.isValid() && HelioRelayActuator::getCanEnable()) {
        return true;
    }
    return false;
}

bool HelioRelayMotorActuator::_enableActuator(float intensity)
{
    bool wasEnabled = _enabled;
    intensity = constrain(intensity, -1.0f, 1.0f);

    if (_outputPin.isValid() && _outputPin2.isValid() && (!_enabled || !isFPEqual(_intensity, intensity))) {
        _intensity = intensity;
        if (!isFPEqual(_intensity, 0.0f)) {
            _enabled = true;
            if (!signbit(_intensity)) {
                _outputPin.activate();
                _outputPin2.deactivate();
            } else {
                _outputPin.deactivate();
                _outputPin2.activate();
            }
            if (!wasEnabled) { handleActivation(); }
        } else {
            _outputPin.deactivate();
            _outputPin2.deactivate();
        }
    }

    return _enabled;
}

void HelioRelayMotorActuator::_disableActuator()
{
    if (_outputPin.isValid() && _outputPin2.isValid() && _enabled) {
        _enabled = false;
        _outputPin.deactivate();
        _outputPin2.deactivate();
        handleActivation();
    }
}

void HelioRelayMotorActuator::handleActivation()
{
    millis_t time = millis();
    HelioActuator::handleActivation();

    if (_enabled) {
        _travelDistanceAccum = 0;
        _travelTimeStart = _travelTimeAccum = max(1, time);
    } else {
        millis_t duration = time - _travelTimeAccum;
        if (duration) { handleTravelTime(duration); }
        _travelTimeAccum = 0;
        duration = time - _travelTimeStart;

        getLoggerInstance()->logStatus(this, SFP(HStr_Log_MeasuredTravel));
        if (getPanel()) { getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Panel), getPanel()->getKeyString()); }
        getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Travel_Measured), measurementToString(_travelDistanceAccum, baseUnitsFromRate(getSpeedUnits()), 1));
        getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Time_Measured), roundToString(duration / 1000.0f, 1), String('s'));
    }
}

bool HelioRelayMotorActuator::canTravel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits)
{
    if (getPanel() && _contSpeed.value > FLT_EPSILON) {
        convertUnits(&distance, &distanceUnits, getDistanceUnits());
        HelioSingleMeasurement position = getPosition().getMeasurement();
        position.value += (direction != Helio_DirectionMode_Reverse ? distance : -distance);
        return position.value > 0.0f && position.value <= 1000.0f; // todo: actual track bounds
    }
    return false;
}

HelioActivationHandle HelioRelayMotorActuator::travel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits)
{
    if (getPanel() && _contSpeed.value > FLT_EPSILON) {
        convertUnits(&distance, &distanceUnits, baseUnitsFromRate(getSpeedUnits()));
        return travel(direction, (millis_t)((distance / _contSpeed.value) * secondsToMillis(SECS_PER_MIN)));
    }
    return HelioActivationHandle();
}

bool HelioRelayMotorActuator::canTravel(Helio_DirectionMode direction, millis_t time)
{
    if (getPanel() && _contSpeed.value > FLT_EPSILON) {
        return canTravel(direction, _contSpeed.value * (time / (float)secondsToMillis(SECS_PER_MIN)), baseUnitsFromRate(getSpeedUnits()));
    }
    return false;
}

HelioActivationHandle HelioRelayMotorActuator::travel(Helio_DirectionMode direction, millis_t time)
{
    if (getPanel()) {
        #ifdef HELIO_USE_MULTITASKING
            getLoggerInstance()->logStatus(this, SFP(HStr_Log_CalculatedTravel));
            if (getPanel()) { getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Panel), getPanel()->getKeyString()); }
            if (_contSpeed.value > FLT_EPSILON) {
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Travel_Calculated), measurementToString(_contSpeed.value * (time / (float)secondsToMillis(SECS_PER_MIN)), baseUnitsFromRate(getSpeedUnits()), 1));
            }
            getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Time_Calculated), roundToString(time / 1000.0f, 1), String('s'));
            return enableActuator(direction, 1.0f, time);
        #else
            getLoggerInstance()->logStatus(this, SFP(HStr_Log_CalculatedTravel));
            if (getPanel()) { getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Panel), getPanel()->getKeyString()); }
            if (_contSpeed.value > FLT_EPSILON) {
                getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Travel_Calculated), measurementToString(_contSpeed.value * (time / (float)secondsToMillis(SECS_PER_MIN)), baseUnitsFromRate(getSpeedUnits()), 1));
            }
            getLoggerInstance()->logMessage(SFP(HStr_Log_Field_Time_Calculated), roundToString(time / 1000.0f, 1), String('s'));
            return enableActuator(direction, 1.0f, time);
        #endif
    }
}

void HelioRelayMotorActuator::setDistanceUnits(Helio_UnitsType distanceUnits)
{
    if (_distanceUnits != distanceUnits) {
        _distanceUnits = distanceUnits;

        _position.setMeasurementUnits(getDistanceUnits());
    }
}

Helio_UnitsType HelioRelayMotorActuator::getDistanceUnits() const
{
    return definedUnitsElse(_distanceUnits, defaultDistanceUnits());
}

void HelioRelayMotorActuator::setSpeedUnits(Helio_UnitsType speedUnits)
{
    if (_speedUnits != speedUnits) {
        _speedUnits = speedUnits;

        convertUnits(&_contSpeed, getSpeedUnits());
        _speed.setMeasurementUnits(getSpeedUnits());
    }
}

Helio_UnitsType HelioRelayMotorActuator::getSpeedUnits() const
{
    return definedUnitsElse(_speedUnits, defaultDistanceUnits());
}

void HelioRelayMotorActuator::setContinuousSpeed(HelioSingleMeasurement contSpeed)
{
    _contSpeed = contSpeed;
    _contSpeed.setMinFrame(1);

    convertUnits(&_contSpeed, getSpeedUnits());
}

const HelioSingleMeasurement &HelioRelayMotorActuator::getContinuousSpeed()
{
    return _contSpeed;
}

HelioSensorAttachment &HelioRelayMotorActuator::getPosition(bool poll)
{
    _position.updateIfNeeded(poll);
    return _position;
}

HelioSensorAttachment &HelioRelayMotorActuator::getSpeed(bool poll)
{
    _speed.updateIfNeeded(poll);
    return _speed;
}

void HelioRelayMotorActuator::saveToData(HelioData *dataOut)
{
    HelioRelayActuator::saveToData(dataOut);

    _outputPin2.saveToData(&((HelioMotorActuatorData *)dataOut)->outputPin2);
    ((HelioMotorActuatorData *)dataOut)->distanceUnits = _distanceUnits;
    ((HelioMotorActuatorData *)dataOut)->speedUnits = _speedUnits;
    if (_contSpeed.frame) {
        _contSpeed.saveToData(&(((HelioMotorActuatorData *)dataOut)->contSpeed));
    }
    if (_position.getId()) {
        strncpy(((HelioMotorActuatorData *)dataOut)->positionSensor, _position.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_speed.getId()) {
        strncpy(((HelioMotorActuatorData *)dataOut)->speedSensor, _speed.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
}

void HelioRelayMotorActuator::pollTravelingSensors()
{
    if (getPositionSensor()) {
        _position->takeMeasurement(true);
    }
    if (getSpeedSensor()) {
        _speed->takeMeasurement(true);
    }
}

void HelioRelayMotorActuator::handleTravelTime(millis_t time)
{
    float speedVal = _speed.getMeasurementFrame() && getSpeedSensor() && !_speed->needsPolling(HELIO_ACT_TRAVELCALC_MAXFRAMEDIFF) &&
                     _speed.getMeasurementValue() >= (_contSpeed.value * HELIO_ACT_TRAVELCALC_MINSPEED) - FLT_EPSILON ? _speed.getMeasurementValue() : _contSpeed.value;
    float distanceTraveled = speedVal * (time / (float)secondsToMillis(SECS_PER_MIN));
    _travelDistanceAccum += distanceTraveled;

    // todo: speed/pos sensor variance
}


HelioVariableActuator::HelioVariableActuator(Helio_ActuatorType actuatorType,
                                             Helio_PositionIndex actuatorIndex,
                                             HelioAnalogPin outputPin,
                                             int classType)
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

bool HelioVariableActuator::isEnabled(float tolerance) const
{
    return _enabled && _intensity >= tolerance - FLT_EPSILON;
}

bool HelioVariableActuator::_enableActuator(float intensity)
{
    bool wasEnabled = _enabled;
    intensity = constrain(intensity, 0.0f, 1.0f);

    if (_outputPin.isValid() && (!_enabled || !isFPEqual(_intensity, intensity))) {
        _intensity = intensity;
        if (!isFPEqual(_intensity, 0.0f)) {
            _enabled = true;
            _outputPin.analogWrite(_intensity);
            if (!wasEnabled) { handleActivation(); }
        } else {
            _outputPin.analogWrite_raw(0);
        }
    }

    return _enabled;
}

void HelioVariableActuator::_disableActuator()
{
    if (_outputPin.isValid() && _enabled) {
        _enabled = false;
        _outputPin.analogWrite_raw(0);
        handleActivation();
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
    if (enableMode != Helio_EnableMode_Undefined) { objectOut[SFP(HStr_Key_EnableMode)] = enableMode; }
    if (contPowerUsage.value > FLT_EPSILON) {
        JsonObject contPowerUsageObj = objectOut.createNestedObject(SFP(HStr_Key_ContinousPowerUsage));
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
    enableMode = objectIn[SFP(HStr_Key_EnableMode)] | enableMode;
    JsonVariantConst contPowerUsageVar = objectIn[SFP(HStr_Key_ContinousPowerUsage)];
    if (!contPowerUsageVar.isNull()) { contPowerUsage.fromJSONVariant(contPowerUsageVar); }
    const char *railNameStr = objectIn[SFP(HStr_Key_RailName)];
    if (railNameStr && railNameStr[0]) { strncpy(railName, railNameStr, HELIO_NAME_MAXSIZE); }
    const char *panelNameStr = objectIn[SFP(HStr_Key_PanelName)];
    if (panelNameStr && panelNameStr[0]) { strncpy(panelName, panelNameStr, HELIO_NAME_MAXSIZE); }
}

HelioMotorActuatorData::HelioMotorActuatorData()
    : HelioActuatorData(), outputPin2(), distanceUnits(Helio_UnitsType_Undefined), speedUnits(Helio_UnitsType_Undefined), contSpeed(), positionSensor{0}, speedSensor{0}
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
    if (speedUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_SpeedUnits)] = unitsTypeToSymbol(distanceUnits); }
    if (contSpeed.value > FLT_EPSILON) {
        JsonObject contSpeedObj = objectOut.createNestedObject(SFP(HStr_Key_ContinousSpeed));
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
    speedUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_SpeedUnits)]);
    JsonVariantConst contSpeedVar = objectIn[SFP(HStr_Key_ContinousSpeed)];
    if (!contSpeedVar.isNull()) { contSpeed.fromJSONVariant(contSpeedVar); }
    const char *positionSensorStr = objectIn[SFP(HStr_Key_PositionSensor)];
    if (positionSensorStr && positionSensorStr[0]) { strncpy(positionSensor, positionSensorStr, HELIO_NAME_MAXSIZE); }
    const char *speedSensorStr = objectIn[SFP(HStr_Key_SpeedSensor)];
    if (speedSensorStr && speedSensorStr[0]) { strncpy(speedSensor, speedSensorStr, HELIO_NAME_MAXSIZE); }
}

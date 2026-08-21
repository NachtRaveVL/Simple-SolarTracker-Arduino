/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Variable Motor Actuator
*/

#include "Helioduino.h"

static constexpr float HELIO_CONT_SERVO_MIN = 0.025f;
static constexpr float HELIO_CONT_SERVO_MAX = 0.125f;
static constexpr float HELIO_CONT_SERVO_NEUTRAL = (HELIO_CONT_SERVO_MIN + HELIO_CONT_SERVO_MAX) * 0.5f;
static constexpr float HELIO_CONT_SERVO_SPAN = (HELIO_CONT_SERVO_MAX - HELIO_CONT_SERVO_MIN) * 0.5f;

HelioVariableMotorActuator::HelioVariableMotorActuator(Helio_ActuatorType actuatorType,
                                                       hposi_t actuatorIndex,
                                                       HelioAnalogPin outputPin,
                                                       HelioAnalogPin outputPin2,
                                                       Pair<float,float> travelRange)
    : HelioVariableActuator(actuatorType, actuatorIndex, outputPin, VariableMotor),
      HelioDistanceUnitsInterfaceStorage(defaultDistanceUnits()),
      _outputPin2(outputPin2), _contSpeed(), _position(this), _speed(this),
      _minimum(this), _maximum(this), _travelRange(travelRange), _signedIntensity(0.0f),
      _travelPosStart(0.0f), _travelDistAccum(0.0f), _travelTimeStart(0), _travelTimeAccum(0),
      _coastTimeMillis(0)
{
    HELIO_HARD_ASSERT(isCenteredServoOutput() || _outputPin2.isValid(), SFP(HStr_Err_InvalidPinOrType));

    if (_outputPin2.isValid()) {
        _outputPin2.init();
        _outputPin2.analogWrite_raw(0);
    } else if (isCenteredServoOutput()) {
        _outputPin.analogWrite(HELIO_CONT_SERVO_NEUTRAL);
    }

    _position.setMeasurementUnits(getDistanceUnits());
    _speed.setMeasurementUnits(getSpeedUnits());
    _minimum.setHandleMethod(&HelioVariableMotorActuator::handleMinimumTrigger, this);
    _maximum.setHandleMethod(&HelioVariableMotorActuator::handleMaximumTrigger, this);
}

HelioVariableMotorActuator::HelioVariableMotorActuator(const HelioMotorActuatorData *dataIn)
    : HelioVariableActuator(dataIn),
      HelioDistanceUnitsInterfaceStorage(definedUnitsElse(dataIn->distanceUnits, defaultDistanceUnits())),
      _outputPin2(&dataIn->outputPin2), _contSpeed(dataIn->contSpeed), _position(this), _speed(this),
      _minimum(this), _maximum(this),
      _travelRange(make_pair(dataIn->travelRange[0], dataIn->travelRange[1])), _signedIntensity(0.0f),
      _travelPosStart(0.0f), _travelDistAccum(0.0f), _travelTimeStart(0), _travelTimeAccum(0),
      _coastTimeMillis(dataIn->coastTimeMillis)
{
    if (_outputPin2.isValid()) {
        _outputPin2.init();
        _outputPin2.analogWrite_raw(0);
    } else if (isCenteredServoOutput()) {
        _outputPin.analogWrite(HELIO_CONT_SERVO_NEUTRAL);
    }

    _contSpeed.setMinFrame(1);
    convertUnits(&_contSpeed, getSpeedUnits());
    _position.setMeasurementUnits(getDistanceUnits());
    _speed.setMeasurementUnits(getSpeedUnits());
    _position.initObject(dataIn->positionSensor);
    _speed.initObject(dataIn->speedSensor);

    _minimum.setHandleMethod(&HelioVariableMotorActuator::handleMinimumTrigger, this);
    _minimum = newTriggerObjectFromSubData(&(dataIn->minTrigger));
    HELIO_SOFT_ASSERT(_minimum, SFP(HStr_Err_AllocationFailure));

    _maximum.setHandleMethod(&HelioVariableMotorActuator::handleMaximumTrigger, this);
    _maximum = newTriggerObjectFromSubData(&(dataIn->maxTrigger));
    HELIO_SOFT_ASSERT(_maximum, SFP(HStr_Err_AllocationFailure));
}

HelioVariableMotorActuator::~HelioVariableMotorActuator()
{
    _disableActuator();
}

void HelioVariableMotorActuator::update()
{
    HelioActuator::update();
    _position.updateIfNeeded(true);
    _speed.updateIfNeeded(true);
    _minimum.updateIfNeeded(true);
    _maximum.updateIfNeeded(true);

    if (_enabled && _travelTimeStart) {
        millis_t time = nzMillis();
        if (time - _travelTimeAccum >= HELIO_ACT_TRAVELCALC_UPDATEMS) {
            handleTravelTime(time);
        }
    }
}

SharedPtr<HelioObjInterface> HelioVariableMotorActuator::getSharedPtrFor(const HelioObjInterface *obj) const
{
    return obj->getKey() == _minimum.getKey() ? _minimum.getSharedPtrFor(obj) :
           obj->getKey() == _maximum.getKey() ? _maximum.getSharedPtrFor(obj) :
           HelioObject::getSharedPtrFor(obj);
}

bool HelioVariableMotorActuator::getCanEnable()
{
    return _outputPin.isValid() && (isCenteredServoOutput() || _outputPin2.isValid()) && HelioActuator::getCanEnable();
}

float HelioVariableMotorActuator::getDriveIntensity() const
{
    return _signedIntensity;
}

bool HelioVariableMotorActuator::isEnabled(float tolerance) const
{
    return _enabled && fabsf(_signedIntensity) >= tolerance - FLT_EPSILON;
}

void HelioVariableMotorActuator::writeMotorOutput(float intensity)
{
    intensity = constrain(intensity, -1.0f, 1.0f);

    if (isCenteredServoOutput()) {
        _outputPin.analogWrite(HELIO_CONT_SERVO_NEUTRAL + (intensity * HELIO_CONT_SERVO_SPAN));
    } else if (intensity > FLT_EPSILON) {
        _outputPin2.analogWrite_raw(0);
        _outputPin.analogWrite(intensity);
    } else if (intensity < -FLT_EPSILON) {
        _outputPin.analogWrite_raw(0);
        _outputPin2.analogWrite(-intensity);
    } else {
        _outputPin.analogWrite_raw(0);
        _outputPin2.analogWrite_raw(0);
    }
}

void HelioVariableMotorActuator::_enableActuator(float intensity)
{
    if (!getCanEnable()) { return; }

    intensity = constrain(intensity, -1.0f, 1.0f);
    if ((intensity < -FLT_EPSILON && isMinTravel(true)) ||
        (intensity > FLT_EPSILON && isMaxTravel(true))) {
        intensity = 0.0f;
    }

    if (fabsf(intensity) <= FLT_EPSILON) {
        _disableActuator();
        return;
    }

    bool wasEnabled = _enabled;
    _signedIntensity = intensity;
    _intensity = fabsf(intensity);
    _enabled = true;
    writeMotorOutput(intensity);

    if (!wasEnabled) { handleActivation(); }
}

void HelioVariableMotorActuator::_disableActuator()
{
    bool wasEnabled = _enabled;
    _enabled = false;
    writeMotorOutput(0.0f);

    if (wasEnabled) { handleActivation(); }
    _signedIntensity = 0.0f;
    _intensity = 0.0f;
}

void HelioVariableMotorActuator::handleActivation()
{
    millis_t time = nzMillis();

    if (_enabled) {
        _travelTimeStart = _travelTimeAccum = time;
        _travelDistAccum = 0.0f;
        _travelPosStart = _position.getMeasurementValue(true);
    } else if (_travelTimeStart) {
        handleTravelTime(time);

        if (_coastTimeMillis && _contSpeed.value > FLT_EPSILON) {
            float coastDistance = getCoastDistance(getDistanceUnits());
            _travelDistAccum += _signedIntensity < 0.0f ? -coastDistance : coastDistance;
            float estimatedPosition = _travelPosStart + _travelDistAccum;
            estimatedPosition = constrain(estimatedPosition, _travelRange.first, _travelRange.second);
            _travelDistAccum = estimatedPosition - _travelPosStart;
            _position.setMeasurement(estimatedPosition);
        }

        _travelTimeStart = 0;
    }

    HelioActuator::handleActivation();
}

bool HelioVariableMotorActuator::canTravel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits)
{
    if (_contSpeed.value > FLT_EPSILON) {
        convertUnits(&distance, &distanceUnits, getDistanceUnits());
        HelioSingleMeasurement position = getPositionSensorAttachment().getMeasurement();
        position.value += direction != Helio_DirectionMode_Reverse ? distance : -distance;
        return position.value >= _travelRange.first - FLT_EPSILON && position.value <= _travelRange.second + FLT_EPSILON;
    }
    return false;
}

HelioActivationHandle HelioVariableMotorActuator::travel(Helio_DirectionMode direction, float distance, Helio_UnitsType distanceUnits)
{
    if (_contSpeed.value > FLT_EPSILON) {
        convertUnits(&distance, &distanceUnits, getDistanceUnits());
        const uint32_t totalTravelTime = (uint32_t)((fabsf(distance) / _contSpeed.value) * secondsToMillis(SECS_PER_MIN));
        return travel(direction, (millis_t)helioPoweredTravelTime(totalTravelTime, _coastTimeMillis));
    }
    return HelioActivationHandle();
}

bool HelioVariableMotorActuator::canTravel(Helio_DirectionMode direction, millis_t time)
{
    if (_contSpeed.value > FLT_EPSILON) {
        const uint32_t poweredTime = time > 0 ? (uint32_t)time : 0;
        const uint32_t predictedTime = poweredTime + (poweredTime ? _coastTimeMillis : 0);
        return canTravel(direction, _contSpeed.value * (predictedTime / (float)secondsToMillis(SECS_PER_MIN)), getDistanceUnits());
    }
    return false;
}

HelioActivationHandle HelioVariableMotorActuator::travel(Helio_DirectionMode direction, millis_t time)
{
    if (!canTravel(direction, time)) { return HelioActivationHandle(); }
    return enableActuator(direction, 1.0f, time);
}

void HelioVariableMotorActuator::setDistanceUnits(Helio_UnitsType distanceUnits)
{
    if (_distUnits != distanceUnits) {
        _distUnits = distanceUnits;
        convertUnits(&_contSpeed, getSpeedUnits());
        _position.setMeasurementUnits(getDistanceUnits());
        _speed.setMeasurementUnits(getSpeedUnits());
        bumpRevisionIfNeeded();
    }
}

void HelioVariableMotorActuator::setContinuousSpeed(HelioSingleMeasurement contSpeed)
{
    _contSpeed = contSpeed;
    _contSpeed.setMinFrame(1);
    convertUnits(&_contSpeed, getSpeedUnits());
    bumpRevisionIfNeeded();
}

const HelioSingleMeasurement &HelioVariableMotorActuator::getContinuousSpeed()
{
    return _contSpeed;
}

void HelioVariableMotorActuator::setCoastTimeMillis(uint32_t coastTimeMillis)
{
    if (_coastTimeMillis != coastTimeMillis) {
        _coastTimeMillis = coastTimeMillis;
        bumpRevisionIfNeeded();
    }
}

float HelioVariableMotorActuator::getCoastDistance(Helio_UnitsType distanceUnits) const
{
    float coastDistance = fabsf(_contSpeed.value) * (_coastTimeMillis / (float)secondsToMillis(SECS_PER_MIN));
    Helio_UnitsType coastUnits = getDistanceUnits();
    if (distanceUnits != Helio_UnitsType_Undefined) {
        convertUnits(&coastDistance, &coastUnits, distanceUnits);
    }
    return coastDistance;
}

Pair<float,float> HelioVariableMotorActuator::getTravelRange() const
{
    return _travelRange;
}

bool HelioVariableMotorActuator::isMinTravel(bool poll)
{
    if (_minimum.isTriggered(poll)) { return true; }
    return _position.getMeasurementValue(poll) <= _travelRange.first + FLT_EPSILON;
}

bool HelioVariableMotorActuator::isMaxTravel(bool poll)
{
    if (_maximum.isTriggered(poll)) { return true; }
    return _position.getMeasurementValue(poll) >= _travelRange.second - FLT_EPSILON;
}

HelioSensorAttachment &HelioVariableMotorActuator::getPositionSensorAttachment()
{
    return _position;
}

HelioSensorAttachment &HelioVariableMotorActuator::getSpeedSensorAttachment()
{
    return _speed;
}

HelioTriggerAttachment &HelioVariableMotorActuator::getMinimumTriggerAttachment()
{
    return _minimum;
}

HelioTriggerAttachment &HelioVariableMotorActuator::getMaximumTriggerAttachment()
{
    return _maximum;
}

void HelioVariableMotorActuator::saveToData(HelioData *dataOut)
{
    HelioVariableActuator::saveToData(dataOut);

    _outputPin2.saveToData(&((HelioMotorActuatorData *)dataOut)->outputPin2);
    ((HelioMotorActuatorData *)dataOut)->travelRange[0] = _travelRange.first;
    ((HelioMotorActuatorData *)dataOut)->travelRange[1] = _travelRange.second;
    ((HelioMotorActuatorData *)dataOut)->distanceUnits = _distUnits;
    ((HelioMotorActuatorData *)dataOut)->coastTimeMillis = _coastTimeMillis;
    if (_contSpeed.isSet()) {
        _contSpeed.saveToData(&(((HelioMotorActuatorData *)dataOut)->contSpeed));
    }
    if (_position.isSet()) {
        strncpy(((HelioMotorActuatorData *)dataOut)->positionSensor, _position.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_speed.isSet()) {
        strncpy(((HelioMotorActuatorData *)dataOut)->speedSensor, _speed.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_minimum.isSet()) {
        _minimum->saveToData(&(((HelioMotorActuatorData *)dataOut)->minTrigger));
    }
    if (_maximum.isSet()) {
        _maximum->saveToData(&(((HelioMotorActuatorData *)dataOut)->maxTrigger));
    }
}

void HelioVariableMotorActuator::handleTravelTime(millis_t time)
{
    if (getPositionSensor(true)) {
        auto position = _position.getMeasurement();
        auto travelDistTotal = position.value - _travelPosStart;

        if (!getSpeedSensor()) {
            auto timeDelta = (time - _travelTimeAccum) / (float)secondsToMillis(SECS_PER_MIN);
            auto distDelta = travelDistTotal - _travelDistAccum;
            if (timeDelta > FLT_EPSILON) { _speed.setMeasurement(distDelta / timeDelta); }
        }

        _travelDistAccum = travelDistTotal;
    } else {
        auto speed = getSpeedSensor(true) ? _speed.getMeasurement() : _contSpeed;
        float speedMagnitude = fabsf(speed.value);
        if (!getSpeedSensor()) { speedMagnitude *= fabsf(_signedIntensity); }
        const float minSpeed = fabsf(_contSpeed.value) * HELIO_ACT_TRAVELCALC_MINSPEED;

        if (speedMagnitude >= minSpeed - FLT_EPSILON) {
            auto timeDelta = (time - _travelTimeAccum) / (float)secondsToMillis(SECS_PER_MIN);
            auto distDelta = speedMagnitude * timeDelta * (_signedIntensity < 0.0f ? -1.0f : 1.0f);
            _travelDistAccum += distDelta;
            _position.setMeasurement(constrain(_travelPosStart + _travelDistAccum, _travelRange.first, _travelRange.second));
        }
    }

    _travelTimeAccum = time;
}

void HelioVariableMotorActuator::handleMinimumTrigger(Helio_TriggerState minimumTrigger)
{
    if (triggerStateToBool(minimumTrigger) && !getPositionSensor()) {
        getPositionSensorAttachment().setMeasurement(_travelRange.first, getDistanceUnits());
    }
}

void HelioVariableMotorActuator::handleMaximumTrigger(Helio_TriggerState maximumTrigger)
{
    if (triggerStateToBool(maximumTrigger) && !getPositionSensor()) {
        getPositionSensorAttachment().setMeasurement(_travelRange.second, getDistanceUnits());
    }
}

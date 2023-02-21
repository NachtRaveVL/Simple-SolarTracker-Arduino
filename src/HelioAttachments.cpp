/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Attachment Points
*/

#include "Helioduino.h"

HelioDLinkObject::HelioDLinkObject()
    : _key(hkey_none), _obj(nullptr), _keyStr(nullptr)
{ ; }

HelioDLinkObject::HelioDLinkObject(const HelioDLinkObject &obj)
    : _key(obj._key), _obj(obj._obj), _keyStr(nullptr)
{
    if (obj._keyStr) {
        auto len = strnlen(obj._keyStr, HELIO_NAME_MAXSIZE);
        if (len) {
            _keyStr = (const char *)malloc(len + 1);
            strncpy((char *)_keyStr, obj._keyStr, len + 1);
        }
    }
}

HelioDLinkObject::~HelioDLinkObject()
{
    if (_keyStr) { free((void *)_keyStr); }
}

void HelioDLinkObject::unresolve()
{
    if (_obj && !_keyStr) {
        auto id = _obj->getId();
        auto len = id.keyString.length();
        if (len) {
            _keyStr = (const char *)malloc(len + 1);
            strncpy((char *)_keyStr, id.keyString.c_str(), len + 1);
        }
    }
    HELIO_HARD_ASSERT(!_obj || _key == _obj->getKey(), SFP(HStr_Err_OperationFailure));
    _obj = nullptr;
}

SharedPtr<HelioObjInterface> HelioDLinkObject::_getObject()
{
    if (_obj) { return _obj; }
    if (_key == hkey_none) { return nullptr; }
    if (Helioduino::_activeInstance) {
        _obj = static_pointer_cast<HelioObjInterface>(Helioduino::_activeInstance->_objects[_key]);
    }
    if (_obj && _keyStr) {
        free((void *)_keyStr); _keyStr = nullptr;
    }
    return _obj;
}


HelioAttachment::HelioAttachment(HelioObjInterface *parent, hposi_t subIndex)
    : HelioSubObject(parent), _obj(), _subIndex(subIndex)
{ ; }

HelioAttachment::HelioAttachment(const HelioAttachment &attachment)
    : HelioSubObject(attachment._parent), _obj(), _subIndex(attachment._subIndex)
{
    setObject(attachment._obj);
}

HelioAttachment::~HelioAttachment()
{
    if (isResolved() && _obj->isObject() && _parent && _parent->isObject()) {
        _obj.get<HelioObject>()->removeLinkage((HelioObject *)_parent);
    }
}

void HelioAttachment::attachObject()
{
    if (resolve() && _obj->isObject() && _parent && _parent->isObject()) { // purposeful resolve in front
        _obj.get<HelioObject>()->addLinkage((HelioObject *)_parent);
    }
}

void HelioAttachment::detachObject()
{
    if (isResolved() && _obj->isObject() && _parent && _parent->isObject()) {
        _obj.get<HelioObject>()->removeLinkage((HelioObject *)_parent);
    }
    // note: used to set _obj to nullptr here, but found that it's best not to -> avoids additional operator= calls during typical detach scenarios
}

void HelioAttachment::updateIfNeeded(bool poll)
{
    // intended to be overridden by derived classes, but not an error if left not implemented
}

void HelioAttachment::setParent(HelioObjInterface *parent)
{
    if (_parent != parent) {
        if (isResolved() && _obj->isObject() && _parent && _parent->isObject()) { _obj.get<HelioObject>()->removeLinkage((HelioObject *)_parent); }

        _parent = parent;

        if (isResolved() && _obj->isObject() && _parent && _parent->isObject()) { _obj.get<HelioObject>()->addLinkage((HelioObject *)_parent); }
    }
}


HelioActuatorAttachment::HelioActuatorAttachment(HelioObjInterface *parent, hposi_t subIndex)
    : HelioSignalAttachment<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS>(parent, subIndex, &HelioActuator::getActivationSignal),
       _actHandle(), _actSetup(), _updateSlot(nullptr), _rateMultiplier(1.0f), _calledLastUpdate(false)
{ ; }

HelioActuatorAttachment::HelioActuatorAttachment(const HelioActuatorAttachment &attachment)
    : HelioSignalAttachment<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS>(attachment),
      _actHandle(attachment._actHandle), _actSetup(attachment._actSetup),
      _updateSlot(attachment._updateSlot ? attachment._updateSlot->clone() : nullptr),
      _rateMultiplier(attachment._rateMultiplier), _calledLastUpdate(false)
{ ; }

HelioActuatorAttachment::~HelioActuatorAttachment()
{
    if (_updateSlot) { delete _updateSlot; _updateSlot = nullptr; }
}

void HelioActuatorAttachment::updateIfNeeded(bool poll)
{
    if (_actHandle.isValid()) {
        if (isActivated()) {
            _actHandle.elapseTo();
            if (_updateSlot) { _updateSlot->operator()(this); }
            _calledLastUpdate = _actHandle.isDone();
        } else if (_actHandle.isDone() && !_calledLastUpdate) {
            if (_updateSlot) { _updateSlot->operator()(this); }
            _calledLastUpdate = true;
        }
    }
}

void HelioActuatorAttachment::setupActivation(float value, millis_t duration, bool force)
{
    if (resolve()) {
        value = get()->calibrationInvTransform(value);

        if (get()->isMotorType()) {
            setupActivation(HelioActivation(value > FLT_EPSILON ? Helio_DirectionMode_Forward : value < -FLT_EPSILON ? Helio_DirectionMode_Reverse : Helio_DirectionMode_Stop, fabsf(value), duration, (force ? Helio_ActivationFlags_Forced : Helio_ActivationFlags_None)));
            return;
        }
    }

    setupActivation(HelioActivation(Helio_DirectionMode_Forward, value, duration, (force ? Helio_ActivationFlags_Forced : Helio_ActivationFlags_None)));
}

void HelioActuatorAttachment::enableActivation()
{
    if (!_actHandle.actuator && _actSetup.isValid() && resolve()) {
        if (_actHandle.isDone()) { applySetup(); } // repeats existing setup
        _calledLastUpdate = false;
        _actHandle = getObject();
    }
}

void HelioActuatorAttachment::setUpdateSlot(const Slot<HelioActuatorAttachment *> &updateSlot)
{
    if (!_updateSlot || !_updateSlot->operator==(&updateSlot)) {
        if (_updateSlot) { delete _updateSlot; _updateSlot = nullptr; }
        _updateSlot = updateSlot.clone();
    }
}

void HelioActuatorAttachment::applySetup()
{
    if (_actSetup.isValid()) {
        if (isFPEqual(_rateMultiplier, 1.0f)) {
            _actHandle.activation = _actSetup;
        } else {
            _actHandle.activation.direction = _actSetup.direction;
            _actHandle.activation.flags = _actSetup.flags;

            if (resolve() && get()->isAnyBinaryClass()) { // Duration based change for rate multiplier
                _actHandle.activation.intensity = _actSetup.intensity;
                if (!_actHandle.isUntimed()) {
                    _actHandle.activation.duration = _actSetup.duration * _rateMultiplier;
                } else { // cannot directly use rate multiplier
                    _actHandle.activation.duration = _actSetup.duration;
                }
            } else { // Intensity based change for rate multiplier
                _actHandle.activation.intensity = _actSetup.intensity * _rateMultiplier;
                _actHandle.activation.duration = _actSetup.duration;
            }
        }

        if (isActivated() && resolve()) { get()->setNeedsUpdate(); }
    }
}


HelioSensorAttachment::HelioSensorAttachment(HelioObjInterface *parent, hposi_t subIndex, uint8_t measurementRow)
    : HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS>(parent, subIndex, &HelioSensor::getMeasurementSignal),
      _measurementRow(measurementRow), _convertParam(FLT_UNDEF), _needsMeasurement(true)
{
    setHandleMethod(&HelioSensorAttachment::handleMeasurement);
}

HelioSensorAttachment::HelioSensorAttachment(const HelioSensorAttachment &attachment)
    : HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS>(attachment),
      _measurement(attachment._measurement), _measurementRow(attachment._measurementRow),
      _convertParam(attachment._convertParam), _needsMeasurement(attachment._needsMeasurement)
{
    setHandleSlot(*attachment._handleSlot);
}

HelioSensorAttachment::~HelioSensorAttachment()
{ ; }

void HelioSensorAttachment::attachObject()
{
    HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS>::attachObject();

    if (_handleSlot) { _handleSlot->operator()(get()->getMeasurement()); }
    else { handleMeasurement(get()->getMeasurement()); }
}

void HelioSensorAttachment::detachObject()
{
    HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS>::detachObject();

    setNeedsMeasurement();
}

void HelioSensorAttachment::updateIfNeeded(bool poll)
{
    if ((poll || _needsMeasurement) && resolve()) {
        if (_handleSlot) { _handleSlot->operator()(get()->getMeasurement()); }
        else { handleMeasurement(get()->getMeasurement()); }

        get()->takeMeasurement((poll || _needsMeasurement)); // purposeful recheck
    }
}

void HelioSensorAttachment::setMeasurement(HelioSingleMeasurement measurement)
{
    auto outUnits = definedUnitsElse(getMeasurementUnits(), measurement.units);
    _measurement = measurement;
    _measurement.setMinFrame(1);

    convertUnits(&_measurement, outUnits, _convertParam);
    _needsMeasurement = false;
}

void HelioSensorAttachment::setMeasurementRow(uint8_t measurementRow)
{
    if (_measurementRow != measurementRow) {
        _measurementRow = measurementRow;

        setNeedsMeasurement();
    }
}

void HelioSensorAttachment::setMeasurementUnits(Helio_UnitsType units, float convertParam)
{
    if (_measurement.units != units || !isFPEqual(_convertParam, convertParam)) {
        _convertParam = convertParam;
        convertUnits(&_measurement, units, _convertParam);

        setNeedsMeasurement();
    }
}

void HelioSensorAttachment::handleMeasurement(const HelioMeasurement *measurement)
{
    if (measurement && measurement->frame) {
        setMeasurement(getAsSingleMeasurement(measurement, _measurementRow));
    }
}


HelioTriggerAttachment::HelioTriggerAttachment(HelioObjInterface *parent, hposi_t subIndex)
    : HelioSignalAttachment<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS>(parent, subIndex, &HelioTrigger::getTriggerSignal)
{ ; }

HelioTriggerAttachment::HelioTriggerAttachment(const HelioTriggerAttachment &attachment)
    : HelioSignalAttachment<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS>(attachment)
{ ; }

HelioTriggerAttachment::~HelioTriggerAttachment()
{ ; }

void HelioTriggerAttachment::updateIfNeeded(bool poll)
{
    if (poll && resolve()) { get()->update(); }
}


HelioDriverAttachment::HelioDriverAttachment(HelioObjInterface *parent, hposi_t subIndex)
    : HelioSignalAttachment<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS>(parent, subIndex, &HelioDriver::getDrivingSignal)
{ ; }

HelioDriverAttachment::HelioDriverAttachment(const HelioDriverAttachment &attachment)
    : HelioSignalAttachment<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS>(attachment)
{ ; }

HelioDriverAttachment::~HelioDriverAttachment()
{ ; }

void HelioDriverAttachment::updateIfNeeded(bool poll)
{
    if (poll && resolve()) { get()->update(); }
}

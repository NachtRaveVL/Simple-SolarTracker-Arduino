/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Attachment Points
*/

#include "Helioduino.h"

HelioDLinkObject::HelioDLinkObject()
    : _key((Helio_KeyType)-1), _obj(nullptr), _keyStr(nullptr)
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
    if (_key == (Helio_KeyType)-1) { return nullptr; }
    if (Helioduino::_activeInstance) {
        _obj = static_pointer_cast<HelioObjInterface>(Helioduino::_activeInstance->_objects[_key]);
    }
    if (_obj && _keyStr) {
        free((void *)_keyStr); _keyStr = nullptr;
    }
    return _obj;
}


HelioAttachment::HelioAttachment(HelioObjInterface *parent)
    : _parent(parent), _obj()
{ ; }

HelioAttachment::HelioAttachment(const HelioAttachment &attachment)
    : _parent(attachment._parent), _obj()
{
    setObject(attachment._obj);
}

HelioAttachment::~HelioAttachment()
{
    if (_parent && isResolved()) {
        _obj->removeLinkage((HelioObject *)_parent);
    }
}

void HelioAttachment::attachObject()
{
    if (resolve() && _parent) { // purposeful resolve in front
        _obj->addLinkage((HelioObject *)_parent);
    }
}

void HelioAttachment::detachObject()
{
    if (_parent && isResolved()) {
        _obj->removeLinkage((HelioObject *)_parent);
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
        if (isResolved() && _parent) { _obj->removeLinkage((HelioObject *)_parent); }
        _parent = parent;
        if (isResolved() && _parent) { _obj->addLinkage((HelioObject *)_parent); }
    }
}


HelioSensorAttachment::HelioSensorAttachment(HelioObjInterface *parent, uint8_t measurementRow)
    : HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS>(
          parent, &HelioSensor::getMeasurementSignal),
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

    if (_handleSlot) { _handleSlot->operator()(get()->getLatestMeasurement()); }
    else { handleMeasurement(get()->getLatestMeasurement()); }
}

void HelioSensorAttachment::detachObject()
{
    HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS>::detachObject();

    setNeedsMeasurement();
}

void HelioSensorAttachment::updateIfNeeded(bool poll)
{
    if (resolve() && (_needsMeasurement || poll)) {
        if (_handleSlot) { _handleSlot->operator()(get()->getLatestMeasurement()); }
        else { handleMeasurement(get()->getLatestMeasurement()); }

        get()->takeMeasurement((_needsMeasurement || poll)); // purposeful recheck
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


HelioTriggerAttachment::HelioTriggerAttachment(HelioObjInterface *parent)
    : HelioSignalAttachment<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS>(
        parent, &HelioTrigger::getTriggerSignal)
{ ; }

HelioTriggerAttachment::HelioTriggerAttachment(const HelioTriggerAttachment &attachment)
    : HelioSignalAttachment<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS>(attachment)
{ ; }

HelioTriggerAttachment::~HelioTriggerAttachment()
{ ; }

void HelioTriggerAttachment::updateIfNeeded(bool poll)
{
    if (resolve()) { get()->update(); }

}


HelioDriverAttachment::HelioDriverAttachment(HelioObjInterface *parent)
    : HelioSignalAttachment<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS>(
        parent, &HelioDriver::getDrivingSignal)
{ ; }

HelioDriverAttachment::HelioDriverAttachment(const HelioDriverAttachment &attachment)
    : HelioSignalAttachment<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS>(attachment)
{ ; }

HelioDriverAttachment::~HelioDriverAttachment()
{ ; }

void HelioDriverAttachment::updateIfNeeded(bool poll)
{
    if (resolve()) { get()->update(); }
}


HelioActuatorAttachment::HelioActuatorAttachment(HelioObjInterface *parent)
    :  HelioSignalAttachment<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS>(
        parent, &HelioActuator::getActivationSignal),
       _actuatorHandle(), _updateSlot(nullptr), _rateMultiplier(1.0f)
{ ; }

HelioActuatorAttachment::HelioActuatorAttachment(const HelioActuatorAttachment &attachment)
    : HelioSignalAttachment<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS>(attachment),
      _updateSlot(attachment._updateSlot ? attachment._updateSlot->clone() : nullptr),
      _actuatorHandle(attachment._actuatorHandle), _rateMultiplier(attachment._rateMultiplier)
{ ; }

HelioActuatorAttachment::~HelioActuatorAttachment()
{
    if (_updateSlot) { delete _updateSlot; _updateSlot = nullptr; }
}

void HelioActuatorAttachment::updateIfNeeded(bool poll = false)
{
    if (isEnabled()) {
        _actuatorHandle.elapseBy(millis() - _actuatorHandle.checkTime);
        if (_updateSlot) {
            _updateSlot->operator()(&_actuatorHandle);
        }
    }
}

void HelioActuatorAttachment::setUpdateSlot(const Slot<HelioActivationHandle *> &updateSlot)
{
    if (!_updateSlot || !_updateSlot->operator==(&updateSlot)) {
        if (_updateSlot) { delete _updateSlot; _updateSlot = nullptr; }
        _updateSlot = updateSlot.clone();
    }
}

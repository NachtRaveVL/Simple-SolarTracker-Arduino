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
        _keyStr = (const char *)malloc(strlen(obj._keyStr) + 1);
        strcpy((char *)_keyStr, obj._keyStr);
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
            strcpy((char *)_keyStr, id.keyString.c_str());
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
{
    HELIO_HARD_ASSERT(_parent, SFP(HStr_Err_InvalidParameter));
}

HelioAttachment::~HelioAttachment()
{
    if (isResolved()) {
        _obj->removeLinkage((HelioObject *)_parent);
    }
}

void HelioAttachment::attachObject()
{
    _obj->addLinkage((HelioObject *)_parent);
}

void HelioAttachment::detachObject()
{
    if (isResolved()) {
        _obj->removeLinkage((HelioObject *)_parent);
    }
    // note: used to set _obj to nullptr here, but found that it's best not to -> avoids additional operator= calls during typical detach scenarios
}

HelioSensorAttachment::HelioSensorAttachment(HelioObjInterface *parent, uint8_t measurementRow)
    : HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_MEASUREMENT_SLOTS>(
          parent, &HelioSensor::getMeasurementSignal),
      _measurementRow(measurementRow), _convertParam(FLT_UNDEF), _needsMeasurement(true)
{
    setHandleMethod(&HelioSensorAttachment::handleMeasurement);
}

void HelioSensorAttachment::attachObject()
{
    HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_MEASUREMENT_SLOTS>::attachObject();

    if (_handleMethod) { _handleMethod->operator()(get()->getLatestMeasurement()); }
    else { handleMeasurement(get()->getLatestMeasurement()); }
}

void HelioSensorAttachment::detachObject()
{
    HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_MEASUREMENT_SLOTS>::detachObject();

    setNeedsMeasurement();
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
    : HelioSignalAttachment<Helio_TriggerState, HELIO_TRIGGER_STATE_SLOTS>(
        parent, &HelioTrigger::getTriggerSignal)
{ ; }


HelioBalancerAttachment::HelioBalancerAttachment(HelioObjInterface *parent)
    : HelioSignalAttachment<Helio_BalancerState, HELIO_BALANCER_STATE_SLOTS>(
        parent, &HelioBalancer::getBalancerSignal)
{ ; }

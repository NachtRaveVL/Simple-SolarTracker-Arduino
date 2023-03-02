/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Triggers
*/

#include "Helioduino.h"

// Creates trigger object from passed trigger data
HelioTrigger *newTriggerObjectFromSubData(const HelioTriggerSubData *dataIn)
{
    if (!dataIn || !isValidType(dataIn->type)) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && isValidType(dataIn->type), SFP(HStr_Err_InvalidParameter));

    if (dataIn) {
        switch (dataIn->type) {
            case (hid_t)HelioTrigger::MeasureValue:
                return new HelioMeasurementValueTrigger(dataIn);
            case (hid_t)HelioTrigger::MeasureRange:
                return new HelioMeasurementRangeTrigger(dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioTrigger::HelioTrigger(HelioIdentity sensorId, uint8_t measurementRow, float detriggerTol, millis_t detriggerDelay, int typeIn)
    : type((typeof(type))typeIn), _sensor(this), _detriggerTol(detriggerTol), _detriggerDelay(detriggerDelay),
      _lastTrigger(0), _triggerState(Helio_TriggerState_Disabled)
{
    _sensor.setMeasurementRow(measurementRow);
    _sensor.initObject(sensorId);
}

HelioTrigger::HelioTrigger(SharedPtr<HelioSensor> sensor, uint8_t measurementRow, float detriggerTol, millis_t detriggerDelay, int typeIn)
    : type((typeof(type))typeIn), _sensor(this), _detriggerTol(detriggerTol), _detriggerDelay(detriggerDelay),
      _lastTrigger(0), _triggerState(Helio_TriggerState_Disabled)
{
    _sensor.setMeasurementRow(measurementRow);
    _sensor.initObject(sensor);
}

HelioTrigger::HelioTrigger(const HelioTriggerSubData *dataIn)
    : type((typeof(type))(dataIn->type)), _sensor(this), _lastTrigger(0), _triggerState(Helio_TriggerState_Disabled),
      _detriggerTol(dataIn->detriggerTol), _detriggerDelay(dataIn->detriggerDelay)
{
    _sensor.setMeasurementRow(dataIn->measurementRow);
    _sensor.setMeasurementUnits(dataIn->measurementUnits);
    _sensor.initObject(dataIn->sensorName);
}

void HelioTrigger::saveToData(HelioTriggerSubData *dataOut) const
{
    ((HelioTriggerSubData *)dataOut)->type = (int8_t)type;
    if (_sensor.isSet()) {
        strncpy(((HelioTriggerSubData *)dataOut)->sensorName, _sensor.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    ((HelioTriggerSubData *)dataOut)->measurementRow = getMeasurementRow();
    ((HelioTriggerSubData *)dataOut)->measurementUnits = getMeasurementUnits();
    ((HelioTriggerSubData *)dataOut)->detriggerTol = _detriggerTol;
    ((HelioTriggerSubData *)dataOut)->detriggerDelay = _detriggerDelay;
}

void HelioTrigger::update()
{
    _sensor.updateIfNeeded(true);
}

Helio_TriggerState HelioTrigger::getTriggerState(bool poll)
{
    _sensor.updateIfNeeded(poll);
    return _triggerState;
}

void HelioTrigger::setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t)
{
    if (_measurementUnits[0] != measurementUnits) {
        _measurementUnits[0] = measurementUnits;
        bumpRevisionIfNeeded();
    }
}

Helio_UnitsType HelioTrigger::getMeasurementUnits(uint8_t) const
{
    return definedUnitsElse(_measurementUnits[0], _sensor.getMeasurementUnits());
}

HelioSensorAttachment &HelioTrigger::getSensorAttachment()
{
    return _sensor;
}

Signal<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS> &HelioTrigger::getTriggerSignal()
{
    return _triggerSignal;
}


HelioMeasurementValueTrigger::HelioMeasurementValueTrigger(HelioIdentity sensorId, float tolerance, bool triggerBelow, uint8_t measurementRow, float detriggerTol, millis_t detriggerDelay)
    : HelioTrigger(sensorId, measurementRow, detriggerTol, detriggerDelay, MeasureValue),
      _triggerTol(tolerance), _triggerBelow(triggerBelow)
{
    _sensor.setHandleMethod(&HelioMeasurementValueTrigger::handleMeasurement, this);
}

HelioMeasurementValueTrigger::HelioMeasurementValueTrigger(SharedPtr<HelioSensor> sensor, float tolerance, bool triggerBelow, uint8_t measurementRow, float detriggerTol, millis_t detriggerDelay)
    : HelioTrigger(sensor, measurementRow, detriggerTol, detriggerDelay, MeasureValue),
      _triggerTol(tolerance), _triggerBelow(triggerBelow)
{
    _sensor.setHandleMethod(&HelioMeasurementValueTrigger::handleMeasurement, this);
}

HelioMeasurementValueTrigger::HelioMeasurementValueTrigger(const HelioTriggerSubData *dataIn)
    : HelioTrigger(dataIn),
      _triggerTol(dataIn->dataAs.measureValue.tolerance), _triggerBelow(dataIn->dataAs.measureValue.triggerBelow)
{
    _sensor.setHandleMethod(&HelioMeasurementValueTrigger::handleMeasurement, this);
}

void HelioMeasurementValueTrigger::saveToData(HelioTriggerSubData *dataOut) const
{
    HelioTrigger::saveToData(dataOut);

    ((HelioTriggerSubData *)dataOut)->dataAs.measureValue.tolerance = _triggerTol;
    ((HelioTriggerSubData *)dataOut)->dataAs.measureValue.triggerBelow = _triggerBelow;
}

void HelioMeasurementValueTrigger::setTriggerTolerance(float tolerance)
{
    if (!isFPEqual(_triggerTol, tolerance)) {
        _triggerTol = tolerance;

        _sensor.setNeedsMeasurement();
        bumpRevisionIfNeeded();
    }
}

void HelioMeasurementValueTrigger::handleMeasurement(const HelioMeasurement *measurement)
{
    if (measurement && measurement->frame) {
        bool wasState = triggerStateToBool(_triggerState);
        bool nextState = wasState;

        if (measurement->isBinaryType()) {
            nextState = ((HelioBinaryMeasurement *)measurement)->state != _triggerBelow;
            _sensor.setMeasurement(getAsSingleMeasurement(measurement, getMeasurementRow()));
        } else {
            auto measure = getAsSingleMeasurement(measurement, getMeasurementRow());
            convertUnits(&measure, getMeasurementUnits(), getMeasurementConvertParam());
            _sensor.setMeasurement(measure);

            float tolAdditive = (nextState ? _detriggerTol : 0);
            nextState = (_triggerBelow ? measure.value <= _triggerTol + tolAdditive + FLT_EPSILON
                                       : measure.value >= _triggerTol - tolAdditive - FLT_EPSILON);
        }

        if (isDetriggerDelayActive() && nzMillis() - _lastTrigger >= _detriggerDelay) {
            _lastTrigger = 0;
        }

        if (_triggerState == Helio_TriggerState_Disabled ||
            (nextState != wasState && (nextState || !isDetriggerDelayActive()))) {
            _triggerState = triggerStateFromBool(nextState);
            _lastTrigger = nextState && _detriggerDelay ? nzMillis() : 0;

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<Helio_TriggerState>(_triggerSignal, _triggerState);
            #else
                _triggerSignal.fire(_triggerState);
            #endif
        }
    }
}


HelioMeasurementRangeTrigger::HelioMeasurementRangeTrigger(HelioIdentity sensorId, float toleranceLow, float toleranceHigh, bool triggerOutside, uint8_t measurementRow, float detriggerTol, millis_t detriggerDelay)
    : HelioTrigger(sensorId, measurementRow, detriggerTol, detriggerDelay, MeasureRange),
      _triggerTolLow(toleranceLow), _triggerTolHigh(toleranceHigh), _triggerOutside(triggerOutside)
{
    _sensor.setHandleMethod(&HelioMeasurementRangeTrigger::handleMeasurement, this);
}

HelioMeasurementRangeTrigger::HelioMeasurementRangeTrigger(SharedPtr<HelioSensor> sensor, float toleranceLow, float toleranceHigh, bool triggerOutside, uint8_t measurementRow, float detriggerTol, millis_t detriggerDelay)
    : HelioTrigger(sensor, measurementRow, detriggerTol, detriggerDelay, MeasureRange),
      _triggerTolLow(toleranceLow), _triggerTolHigh(toleranceHigh), _triggerOutside(triggerOutside)
{
    _sensor.setHandleMethod(&HelioMeasurementRangeTrigger::handleMeasurement, this);
}

HelioMeasurementRangeTrigger::HelioMeasurementRangeTrigger(const HelioTriggerSubData *dataIn)
    : HelioTrigger(dataIn),
      _triggerTolLow(dataIn->dataAs.measureRange.toleranceLow),
      _triggerTolHigh(dataIn->dataAs.measureRange.toleranceHigh),
      _triggerOutside(dataIn->dataAs.measureRange.triggerOutside)
{
    _sensor.setHandleMethod(&HelioMeasurementRangeTrigger::handleMeasurement, this);
}

void HelioMeasurementRangeTrigger::saveToData(HelioTriggerSubData *dataOut) const
{
    HelioTrigger::saveToData(dataOut);

    ((HelioTriggerSubData *)dataOut)->dataAs.measureRange.toleranceLow = _triggerTolLow;
    ((HelioTriggerSubData *)dataOut)->dataAs.measureRange.toleranceHigh = _triggerTolHigh;
    ((HelioTriggerSubData *)dataOut)->dataAs.measureRange.triggerOutside = _triggerOutside;
}

void HelioMeasurementRangeTrigger::setTriggerMidpoint(float toleranceMid)
{
    float toleranceRangeHalf = (_triggerTolHigh - _triggerTolLow) * 0.5f;

    if (!isFPEqual(_triggerTolLow, toleranceMid - toleranceRangeHalf)) {
        _triggerTolLow = toleranceMid - toleranceRangeHalf;
        _triggerTolHigh = toleranceMid + toleranceRangeHalf;

        _sensor.setNeedsMeasurement();
        bumpRevisionIfNeeded();
    }
}

void HelioMeasurementRangeTrigger::handleMeasurement(const HelioMeasurement *measurement)
{
    if (measurement && measurement->frame) {
        bool wasState = triggerStateToBool(_triggerState);
        bool nextState = wasState;

        auto measure = getAsSingleMeasurement(measurement, getMeasurementRow());
        convertUnits(&measure, getMeasurementUnits(), getMeasurementConvertParam());
        _sensor.setMeasurement(measure);

        float tolAdditive = (nextState ? _detriggerTol : 0);

        if (!_triggerOutside) {
            nextState = (measure.value >= _triggerTolLow - tolAdditive - FLT_EPSILON &&
                         measure.value <= _triggerTolHigh + tolAdditive + FLT_EPSILON);
        } else {
            nextState = (measure.value <= _triggerTolLow + tolAdditive + FLT_EPSILON &&
                         measure.value >= _triggerTolHigh - tolAdditive - FLT_EPSILON);
        }

        if (isDetriggerDelayActive() && nzMillis() - _lastTrigger >= _detriggerDelay) {
            _lastTrigger = 0;
        }

        if (_triggerState == Helio_TriggerState_Disabled ||
            (nextState != wasState && (nextState || !isDetriggerDelayActive()))) {
            _triggerState = triggerStateFromBool(nextState);
            _lastTrigger = nextState && _detriggerDelay ? nzMillis() : 0;

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<Helio_TriggerState>(_triggerSignal, _triggerState);
            #else
                _triggerSignal.fire(_triggerState);
            #endif
        }
    }
}


HelioTriggerSubData::HelioTriggerSubData()
    : HelioSubData(), sensorName{0}, measurementRow(0), dataAs{.measureRange={0.0f,0.0f,false}},
      detriggerTol(0), detriggerDelay(0), measurementUnits(Helio_UnitsType_Undefined)
{ ; }

void HelioTriggerSubData::toJSONObject(JsonObject &objectOut) const
{
    HelioSubData::toJSONObject(objectOut);

    if (sensorName[0]) { objectOut[SFP(HStr_Key_SensorName)] = charsToString(sensorName, HELIO_NAME_MAXSIZE); }
    if (measurementRow > 0) { objectOut[SFP(HStr_Key_MeasurementRow)] = measurementRow; }
    switch (type) {
        case (hid_t)HelioTrigger::MeasureValue:
            objectOut[SFP(HStr_Key_Tolerance)] = dataAs.measureValue.tolerance;
            objectOut[SFP(HStr_Key_TriggerBelow)] = dataAs.measureValue.triggerBelow;
            break;
        case (hid_t)HelioTrigger::MeasureRange:
            objectOut[SFP(HStr_Key_ToleranceLow)] = dataAs.measureRange.toleranceLow;
            objectOut[SFP(HStr_Key_ToleranceHigh)] = dataAs.measureRange.toleranceHigh;
            objectOut[SFP(HStr_Key_TriggerOutside)] = dataAs.measureRange.triggerOutside;
            break;
        default: break;
    }
    if (detriggerTol > FLT_EPSILON) { objectOut[SFP(HStr_Key_DetriggerTol)] = detriggerTol; }
    if (detriggerDelay > 0) { objectOut[SFP(HStr_Key_DetriggerDelay)] = detriggerDelay; }
    if (measurementUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_MeasurementUnits)] = unitsTypeToSymbol(measurementUnits); }
}

void HelioTriggerSubData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioSubData::fromJSONObject(objectIn);

    const char *sensorNameStr = objectIn[SFP(HStr_Key_SensorName)];
    if (sensorNameStr && sensorNameStr[0]) { strncpy(sensorName, sensorNameStr, HELIO_NAME_MAXSIZE); }
    measurementRow = objectIn[SFP(HStr_Key_MeasurementRow)] | measurementRow;
    switch (type) {
        case (hid_t)HelioTrigger::MeasureValue:
            dataAs.measureValue.tolerance = objectIn[SFP(HStr_Key_Tolerance)] | dataAs.measureValue.tolerance;
            dataAs.measureValue.triggerBelow = objectIn[SFP(HStr_Key_TriggerBelow)] | dataAs.measureValue.triggerBelow;
            break;
        case (hid_t)HelioTrigger::MeasureRange:
            dataAs.measureRange.toleranceLow = objectIn[SFP(HStr_Key_ToleranceLow)] | dataAs.measureRange.toleranceLow;
            dataAs.measureRange.toleranceHigh = objectIn[SFP(HStr_Key_ToleranceHigh)] | dataAs.measureRange.toleranceHigh;
            dataAs.measureRange.triggerOutside = objectIn[SFP(HStr_Key_TriggerOutside)] | dataAs.measureRange.triggerOutside;
            break;
        default: break;
    }
    detriggerTol = objectIn[SFP(HStr_Key_DetriggerTol)] | detriggerTol;
    detriggerDelay = objectIn[SFP(HStr_Key_DetriggerDelay)] | detriggerDelay;
    measurementUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_MeasurementUnits)]);
}

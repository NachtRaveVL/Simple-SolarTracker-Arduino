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
            case (int8_t)HelioTrigger::MeasureValue:
                return new HelioMeasurementValueTrigger(dataIn);
            case (int8_t)HelioTrigger::MeasureRange:
                return new HelioMeasurementRangeTrigger(dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioTrigger::HelioTrigger(HelioIdentity sensorId, uint8_t measurementRow, int typeIn)
    : type((typeof(type))typeIn), _sensor(this), _triggerState(Helio_TriggerState_Disabled)
{
    _sensor.setMeasurementRow(measurementRow);
    _sensor.setObject(sensorId);
}

HelioTrigger::HelioTrigger(SharedPtr<HelioSensor> sensor, uint8_t measurementRow, int typeIn)
    : type((typeof(type))typeIn), _sensor(this), _triggerState(Helio_TriggerState_Disabled)
{
    _sensor.setMeasurementRow(measurementRow);
    _sensor.setObject(sensor);
}

HelioTrigger::HelioTrigger(const HelioTriggerSubData *dataIn)
    : type((typeof(type))(dataIn->type)), _sensor(this), _triggerState(Helio_TriggerState_Disabled)
{
    setToleranceUnits(dataIn->toleranceUnits);
    _sensor.setObject(dataIn->sensorName);
}

void HelioTrigger::saveToData(HelioTriggerSubData *dataOut) const
{
    ((HelioTriggerSubData *)dataOut)->type = (int8_t)type;
    if (_sensor.getId()) {
        strncpy(((HelioTriggerSubData *)dataOut)->sensorName, _sensor.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    ((HelioTriggerSubData *)dataOut)->measurementRow = _sensor.getMeasurementRow();
    ((HelioTriggerSubData *)dataOut)->toleranceUnits = getToleranceUnits();
}

void HelioTrigger::update()
{
    _sensor.updateIfNeeded(true);
}

void HelioTrigger::handleLowMemory()
{ ; }

Helio_TriggerState HelioTrigger::getTriggerState() const
{
    return _triggerState;
}

Signal<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS> &HelioTrigger::getTriggerSignal()
{
    return _triggerSignal;
}


HelioMeasurementValueTrigger::HelioMeasurementValueTrigger(HelioIdentity sensorId, float tolerance, bool triggerBelow, float detriggerTol, uint8_t measurementRow)
    : HelioTrigger(sensorId, measurementRow, MeasureValue),
      _triggerTol(tolerance), _detriggerTol(detriggerTol), _triggerBelow(triggerBelow)
{
    _sensor.setHandleMethod(&HelioMeasurementValueTrigger::handleMeasurement);
}

HelioMeasurementValueTrigger::HelioMeasurementValueTrigger(SharedPtr<HelioSensor> sensor, float tolerance, bool triggerBelow, float detriggerTol, uint8_t measurementRow)
    : HelioTrigger(sensor, measurementRow, MeasureValue),
      _triggerTol(tolerance), _detriggerTol(detriggerTol), _triggerBelow(triggerBelow)
{
    _sensor.setHandleMethod(&HelioMeasurementValueTrigger::handleMeasurement);
}

HelioMeasurementValueTrigger::HelioMeasurementValueTrigger(const HelioTriggerSubData *dataIn)
    : HelioTrigger(dataIn),
      _triggerTol(dataIn->dataAs.measureValue.tolerance), _detriggerTol(dataIn->detriggerTol),
      _triggerBelow(dataIn->dataAs.measureValue.triggerBelow)
{
    _sensor.setHandleMethod(&HelioMeasurementValueTrigger::handleMeasurement);
}

void HelioMeasurementValueTrigger::saveToData(HelioTriggerSubData *dataOut) const
{
    HelioTrigger::saveToData(dataOut);

    ((HelioTriggerSubData *)dataOut)->dataAs.measureValue.tolerance = _triggerTol;
    ((HelioTriggerSubData *)dataOut)->detriggerTol = _detriggerTol;
    ((HelioTriggerSubData *)dataOut)->dataAs.measureValue.triggerBelow = _triggerBelow;
}

void HelioMeasurementValueTrigger::setTriggerTolerance(float tolerance)
{
    if (!isFPEqual(_triggerTol, tolerance)) {
        _triggerTol = tolerance;

        _sensor.setNeedsMeasurement();
    }
}

void HelioMeasurementValueTrigger::handleMeasurement(const HelioMeasurement *measurement)
{
    if (measurement && measurement->frame) {
        bool nextState = triggerStateToBool(_triggerState);

        if (measurement->isBinaryType()) {
            nextState = ((HelioBinaryMeasurement *)measurement)->state != _triggerBelow;
            _sensor.setMeasurement(getAsSingleMeasurement(measurement, _sensor.getMeasurementRow()));
        } else {
            auto measure = getAsSingleMeasurement(measurement, _sensor.getMeasurementRow());
            convertUnits(&measure, getToleranceUnits(), _sensor.getMeasurementConvertParam());
            _sensor.setMeasurement(measure);

            float tolAdditive = (nextState ? _detriggerTol : 0);
            nextState = (_triggerBelow ? measure.value <= _triggerTol + tolAdditive + FLT_EPSILON
                                       : measure.value >= _triggerTol - tolAdditive - FLT_EPSILON);
        }

        if (_triggerState == Helio_TriggerState_Disabled ||
            nextState != triggerStateToBool(_triggerState)) {
            _triggerState = triggerStateFromBool(nextState);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<Helio_TriggerState>(_triggerSignal, _triggerState);
            #else
                _triggerSignal.fire(_triggerState);
            #endif
        }
    }
}


HelioMeasurementRangeTrigger::HelioMeasurementRangeTrigger(HelioIdentity sensorId, float toleranceLow, float toleranceHigh, bool triggerOutside, float detriggerTol, uint8_t measurementRow)
    : HelioTrigger(sensorId, measurementRow, MeasureRange),
      _triggerTolLow(toleranceLow), _triggerTolHigh(toleranceHigh), _detriggerTol(detriggerTol),
      _triggerOutside(triggerOutside)
{
    _sensor.setHandleMethod(&HelioMeasurementRangeTrigger::handleMeasurement);
}

HelioMeasurementRangeTrigger::HelioMeasurementRangeTrigger(SharedPtr<HelioSensor> sensor, float toleranceLow, float toleranceHigh, bool triggerOutside, float detriggerTol, uint8_t measurementRow)
    : HelioTrigger(sensor, measurementRow, MeasureRange),
      _triggerTolLow(toleranceLow), _triggerTolHigh(toleranceHigh), _detriggerTol(detriggerTol),
      _triggerOutside(triggerOutside)
{
    _sensor.setHandleMethod(&HelioMeasurementRangeTrigger::handleMeasurement);
}

HelioMeasurementRangeTrigger::HelioMeasurementRangeTrigger(const HelioTriggerSubData *dataIn)
    : HelioTrigger(dataIn),
      _triggerTolLow(dataIn->dataAs.measureRange.toleranceLow),
      _triggerTolHigh(dataIn->dataAs.measureRange.toleranceHigh),
      _detriggerTol(dataIn->detriggerTol),
      _triggerOutside(dataIn->dataAs.measureRange.triggerOutside)
{
    _sensor.setHandleMethod(&HelioMeasurementRangeTrigger::handleMeasurement);
}

void HelioMeasurementRangeTrigger::saveToData(HelioTriggerSubData *dataOut) const
{
    HelioTrigger::saveToData(dataOut);

    ((HelioTriggerSubData *)dataOut)->dataAs.measureRange.toleranceLow = _triggerTolLow;
    ((HelioTriggerSubData *)dataOut)->dataAs.measureRange.toleranceHigh = _triggerTolHigh;
    ((HelioTriggerSubData *)dataOut)->detriggerTol = _detriggerTol;
    ((HelioTriggerSubData *)dataOut)->dataAs.measureRange.triggerOutside = _triggerOutside;
}

void HelioMeasurementRangeTrigger::updateTriggerMidpoint(float toleranceMid)
{
    float toleranceRangeHalf = (_triggerTolHigh - _triggerTolLow) * 0.5f;

    if (!isFPEqual(_triggerTolLow, toleranceMid - toleranceRangeHalf)) {
        _triggerTolLow = toleranceMid - toleranceRangeHalf;
        _triggerTolHigh = toleranceMid + toleranceRangeHalf;

        _sensor.setNeedsMeasurement();
    }
}

void HelioMeasurementRangeTrigger::handleMeasurement(const HelioMeasurement *measurement)
{
    if (measurement && measurement->frame) {
        bool nextState = triggerStateToBool(_triggerState);

        auto measure = getAsSingleMeasurement(measurement, _sensor.getMeasurementRow());
        convertUnits(&measure, getToleranceUnits(), _sensor.getMeasurementConvertParam());
        _sensor.setMeasurement(measure);

        float tolAdditive = (nextState ? _detriggerTol : 0);

        if (!_triggerOutside) {
            nextState = (measure.value >= _triggerTolLow - tolAdditive - FLT_EPSILON &&
                         measure.value <= _triggerTolHigh + tolAdditive + FLT_EPSILON);
        } else {
            nextState = (measure.value <= _triggerTolLow + tolAdditive + FLT_EPSILON &&
                         measure.value >= _triggerTolHigh - tolAdditive - FLT_EPSILON);
        }

        if (_triggerState == Helio_TriggerState_Disabled ||
            nextState != triggerStateToBool(_triggerState)) {
            _triggerState = triggerStateFromBool(nextState);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<Helio_TriggerState>(_triggerSignal, _triggerState);
            #else
                _triggerSignal.fire(_triggerState);
            #endif
        }
    }
}


HelioTriggerSubData::HelioTriggerSubData()
    : HelioSubData(), sensorName{0}, measurementRow(0), dataAs{.measureRange={0.0f,0.0f,false}}, detriggerTol(0), toleranceUnits(Helio_UnitsType_Undefined)
{ ; }

void HelioTriggerSubData::toJSONObject(JsonObject &objectOut) const
{
    HelioSubData::toJSONObject(objectOut);

    if (sensorName[0]) { objectOut[SFP(HStr_Key_SensorName)] = charsToString(sensorName, HELIO_NAME_MAXSIZE); }
    if (measurementRow > 0) { objectOut[SFP(HStr_Key_MeasurementRow)] = measurementRow; }
    switch (type) {
        case 0: // MeasureValue
            objectOut[SFP(HStr_Key_Tolerance)] = dataAs.measureValue.tolerance;
            objectOut[SFP(HStr_Key_TriggerBelow)] = dataAs.measureValue.triggerBelow;
            break;
        case 1: // MeasureRange
            objectOut[SFP(HStr_Key_ToleranceLow)] = dataAs.measureRange.toleranceLow;
            objectOut[SFP(HStr_Key_ToleranceHigh)] = dataAs.measureRange.toleranceHigh;
            objectOut[SFP(HStr_Key_TriggerOutside)] = dataAs.measureRange.triggerOutside;
            break;
        default: break;
    }
    if (detriggerTol > 0) { objectOut[SFP(HStr_Key_DetriggerTol)] = detriggerTol; }
    if (toleranceUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_ToleranceUnits)] = unitsTypeToSymbol(toleranceUnits); }
}

void HelioTriggerSubData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioSubData::fromJSONObject(objectIn);

    const char *sensorNameStr = objectIn[SFP(HStr_Key_SensorName)];
    if (sensorNameStr && sensorNameStr[0]) { strncpy(sensorName, sensorNameStr, HELIO_NAME_MAXSIZE); }
    measurementRow = objectIn[SFP(HStr_Key_MeasurementRow)] | measurementRow;
    switch (type) {
        case 0: // MeasureValue
            dataAs.measureValue.tolerance = objectIn[SFP(HStr_Key_Tolerance)] | dataAs.measureValue.tolerance;
            dataAs.measureValue.triggerBelow = objectIn[SFP(HStr_Key_TriggerBelow)] | dataAs.measureValue.triggerBelow;
            break;
        case 1: // MeasureRange
            dataAs.measureRange.toleranceLow = objectIn[SFP(HStr_Key_ToleranceLow)] | dataAs.measureRange.toleranceLow;
            dataAs.measureRange.toleranceHigh = objectIn[SFP(HStr_Key_ToleranceHigh)] | dataAs.measureRange.toleranceHigh;
            dataAs.measureRange.triggerOutside = objectIn[SFP(HStr_Key_TriggerOutside)] | dataAs.measureRange.triggerOutside;
            break;
        default: break;
    }
    detriggerTol = objectIn[SFP(HStr_Key_DetriggerTol)] | detriggerTol;
    toleranceUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_ToleranceUnits)]);
}

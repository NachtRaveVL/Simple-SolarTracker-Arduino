/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Balancers
*/

#include "Helioduino.h"

HelioBalancer::HelioBalancer(SharedPtr<HelioSensor> sensor, float targetSetpoint, float targetRange, uint8_t measurementRow, int typeIn)
    : type((typeof(type))typeIn), _targetSetpoint(targetSetpoint), _targetRange(targetRange), _enabled(false),
      _sensor(this), _balancerState(Helio_BalancerState_Undefined)
{
    _sensor.setMeasurementRow(measurementRow);
    _sensor.setHandleMethod(&HelioBalancer::handleMeasurement);
    _sensor.setObject(sensor);
}

HelioBalancer::~HelioBalancer()
{
    _enabled = false;
    disableAllActuators();
}

void HelioBalancer::update()
{
    _sensor.updateIfNeeded(true);
}

void HelioBalancer::setTargetSetpoint(float targetSetpoint)
{
    if (!isFPEqual(_targetSetpoint, targetSetpoint)) {
        _targetSetpoint = targetSetpoint;

        _sensor.setNeedsMeasurement();
    }
}

Helio_BalancerState HelioBalancer::getBalancerState() const
{
    return _balancerState;
}

void HelioBalancer::setIncrementActuators(const Vector<Pair<SharedPtr<HelioActuator>, float>, HELIO_BAL_INCACTUATORS_MAXSIZE> &incActuators)
{
    for (auto actuatorIter = _incActuators.begin(); actuatorIter != _incActuators.end(); ++actuatorIter) {
        bool found = false;
        auto key = actuatorIter->first->getKey();

        for (auto actuatorInIter = incActuators.begin(); actuatorInIter != incActuators.end(); ++actuatorInIter) {
            if (key == actuatorInIter->first->getKey()) {
                found = true;
                break;
            }
        }

        if (!found && actuatorIter->first->isEnabled()) { // disables actuators not found in new list, prevents same used actuators from prev cycle from turning off/on on cycle switch
            actuatorIter->first->disableActuator();
        }
    }

    {   _incActuators.clear();
        for (auto actuatorInIter = incActuators.begin(); actuatorInIter != incActuators.end(); ++actuatorInIter) {
            auto actuator = (*actuatorInIter);
            _incActuators.push_back(actuator);
        }
    }
}

void HelioBalancer::setDecrementActuators(const Vector<Pair<SharedPtr<HelioActuator>, float>, HELIO_BAL_DECACTUATORS_MAXSIZE> &decActuators)
{
    for (auto actuatorIter = _decActuators.begin(); actuatorIter != _decActuators.end(); ++actuatorIter) {
        bool found = false;
        auto key = actuatorIter->first->getKey();

        for (auto actuatorInIter = decActuators.begin(); actuatorInIter != decActuators.end(); ++actuatorInIter) {
            if (key == actuatorInIter->first->getKey()) {
                found = true;
                break;
            }
        }

        if (!found && actuatorIter->first->isEnabled()) { // disables actuators not found in new list
            actuatorIter->first->disableActuator();
        }
    }

    {   _decActuators.clear();
        for (auto actuatorInIter = decActuators.begin(); actuatorInIter != decActuators.end(); ++actuatorInIter) {
            auto actuator = (*actuatorInIter);
            _decActuators.push_back(actuator);
        }
    }
}

Signal<Helio_BalancerState, HELIO_BALANCER_STATE_SLOTS> &HelioBalancer::getBalancerSignal()
{
    return _balancerSignal;
}

void HelioBalancer::disableAllActuators()
{
    for (auto actuatorIter = _incActuators.begin(); actuatorIter != _incActuators.end(); ++actuatorIter) {
        actuatorIter->first->disableActuator();
    }
    for (auto actuatorIter = _decActuators.begin(); actuatorIter != _decActuators.end(); ++actuatorIter) {
        actuatorIter->first->disableActuator();
    }
}

void HelioBalancer::handleMeasurement(const HelioMeasurement *measurement)
{
    if (measurement && measurement->frame) {
        auto balancerStateBefore = _balancerState;

        auto measure = getAsSingleMeasurement(measurement, _sensor.getMeasurementRow());
        convertUnits(&measure, getTargetUnits(), _sensor.getMeasurementConvertParam());
        _sensor.setMeasurement(measure);

        if (_enabled) {
            float halfTargetRange = _targetRange * 0.5f;
            if (measure.value > _targetSetpoint - halfTargetRange + FLT_EPSILON &&
                measure.value < _targetSetpoint + halfTargetRange - FLT_EPSILON) {
                _balancerState = Helio_BalancerState_Balanced;
            } else {
                _balancerState = measure.value > _targetSetpoint ? Helio_BalancerState_TooHigh : Helio_BalancerState_TooLow;
            }

            if (_balancerState != balancerStateBefore) {
                #ifdef HELIO_USE_MULTITASKING
                    scheduleSignalFireOnce<Helio_BalancerState>(_balancerSignal, _balancerState);
                #else
                    _balancerSignal.fire(_balancerState);
                #endif
            }
        }
    }
}


HelioLinearEdgeBalancer::HelioLinearEdgeBalancer(SharedPtr<HelioSensor> sensor, float targetSetpoint, float targetRange, float edgeOffset, float edgeLength, uint8_t measurementRow)
    : HelioBalancer(sensor, targetSetpoint, targetRange, measurementRow, LinearEdge), _edgeOffset(edgeOffset), _edgeLength(edgeLength)
{ ; }

void HelioLinearEdgeBalancer::update()
{
    HelioBalancer::update();
    if (!_enabled || !_sensor) { return; }

    if (_balancerState != Helio_BalancerState_Balanced && _balancerState != Helio_BalancerState_Undefined) {
        auto measure = _sensor.getMeasurement(true);

        float x = fabsf(measure.value - _targetSetpoint);
        float val = _edgeLength > FLT_EPSILON ? mapValue<float>(x, _edgeOffset, _edgeOffset + _edgeLength, 0.0f, 1.0f)
                                                : (x >= _edgeOffset - FLT_EPSILON ? 1.0 : 0.0f);
        val = constrain(val, 0.0f, 1.0f);

        if (_balancerState == Helio_BalancerState_TooLow) {
            for (auto actuatorIter = _incActuators.begin(); actuatorIter != _incActuators.end(); ++actuatorIter) {
                actuatorIter->first->enableActuator(val * actuatorIter->second);
            }
        } else {
            for (auto actuatorIter = _decActuators.begin(); actuatorIter != _decActuators.end(); ++actuatorIter) {
                actuatorIter->first->enableActuator(val * actuatorIter->second);
            }
        }
    }
}

// HelioActivationHandle handle(actuator->enableActuator(time));
// while (handle.actuator && handle.duration) { handle.actuator->update(); delay(1); }
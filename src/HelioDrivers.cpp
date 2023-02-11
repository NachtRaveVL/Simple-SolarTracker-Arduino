/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Drivers
*/

#include "Helioduino.h"

HelioDriver::HelioDriver(float travelRate, int typeIn)
    : type((typeof(type))typeIn), _targetUnits(Helio_UnitsType_Raw_0_1),
      _targetSetpoint(FLT_UNDEF), _travelRate(travelRate),
      _trackMin(__FLT_MAX__), _trackMax(-__FLT_MAX__),
      _drivingState(Helio_DrivingState_Undefined), _enabled(false)
{ ; }

HelioDriver::~HelioDriver()
{
    _enabled = false;
    disableAllActivations();
}

void HelioDriver::update()
{
    for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
        attachIter->updateIfNeeded(true);
    }

    handleOffset(getMaximumOffset());
}

void HelioDriver::setTargetSetpoint(float targetSetpoint)
{
    if (!isFPEqual(_targetSetpoint, targetSetpoint)) {
        _targetSetpoint = targetSetpoint;
    }
}

void HelioDriver::setTravelRate(float travelRate)
{
    if (!isFPEqual(_travelRate, travelRate)) {
        _travelRate = travelRate;
    }
}

void HelioDriver::setActuators(const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &actuators)
{
    for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
        bool found = false;
        auto key = attachIter->getKey();

        for (auto attachInIter = actuators.begin(); attachInIter != actuators.end(); ++attachInIter) {
            if (key == attachInIter->getKey()) {
                found = true;
                break;
            }
        }

        if (!found) { // disables activations not found in new list
            attachIter->disableActivation();
        }
    }

    {   _actuators.clear();
        _trackMin = __FLT_MAX__; _trackMax = -__FLT_MAX__;
        for (auto attachInIter = actuators.begin(); attachInIter != actuators.end(); ++attachInIter) {
            _actuators.push_back(*attachInIter);
            _actuators.back().setParent(this);
            auto actuator = _actuators.back().get();
            if (actuator) {
                auto trackExtents = actuator->getTrackExtents();
                if (trackExtents.first < _trackMin) { _trackMin = trackExtents.first; }
                if (trackExtents.second > _trackMax) { _trackMax = trackExtents.second; }
            }
        }
    }
}

Helio_DrivingState HelioDriver::getDrivingState() const
{
    return _drivingState;
}

float HelioDriver::getMaximumOffset(bool poll)
{
    float maxDelta = 0;

    for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
        if ((*attachIter)->isAnyMotorClass()) {
            HelioPositionSensorAttachmentInterface *positionInt = attachIter->HelioAttachment::get<HelioPositionSensorAttachmentInterface>();
            float delta = _targetSetpoint - positionInt->getPosition().getMeasurementValue(poll);
            if (fabsf(delta) > maxDelta) { maxDelta = delta; }
        } else {
            float delta = _targetSetpoint - (*attachIter)->getCalibratedValue();
            if (fabsf(delta) > maxDelta) { maxDelta = delta; }
        }
    }

    return maxDelta;
}

Signal<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> &HelioDriver::getDrivingSignal()
{
    return _drivingSignal;
}

void HelioDriver::disableAllActivations()
{
    for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
        attachIter->disableActivation();
    }
}


// HelioAbsoluteDriver::HelioAbsoluteDriver(SharedPtr<HelioSensor> sensor, float targetSetpoint, float targetRange, float edgeOffset, float edgeLength, uint8_t measurementRow)
//     : HelioDriver(sensor, targetSetpoint, targetRange, measurementRow, Servo), _edgeOffset(edgeOffset), _edgeLength(edgeLength)
// { ; }

void HelioAbsoluteDriver::handleOffset(float maximumOffset)
{
    if (!_enabled) { return; }

    auto hadDrivingState = _drivingState;
    _drivingState = maximumOffset < -FLT_EPSILON ? Helio_DrivingState_GoHigher :
                    maximumOffset > FLT_EPSILON ? Helio_DrivingState_GoLower :
                    Helio_DrivingState_OnTarget;

    if (_drivingState != Helio_DrivingState_OnTarget && _drivingState != Helio_DrivingState_Undefined && _targetSetpoint != FLT_UNDEF) {
        millis_t time = nzMillis();
        if (!_lastUpdate) { _lastUpdate = time; }
        millis_t delta = time - _lastUpdate;
        _lastUpdate = time;

        for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
            if (isInstantaneous()) {
                attachIter->setupActivation(_targetSetpoint);
                attachIter->enableActivation();
            } else {
                float position = (*attachIter)->getCalibratedValue();
                if (position < _targetSetpoint) {
                    position += _travelRate * delta / secondsToMillis(SECS_PER_MIN);
                    if (position > _targetSetpoint) { position = _targetSetpoint; }
                } else if (position > _targetSetpoint) {
                    position -= _travelRate * delta / secondsToMillis(SECS_PER_MIN);
                    if (position < _targetSetpoint) { position = _targetSetpoint; }
                }
                attachIter->setupActivation(position);
                attachIter->enableActivation();
            }
        }
    } else {
        disableAllActivations();
    }

    if (hadDrivingState != _drivingState && _drivingState != Helio_DrivingState_Undefined) {
        #ifdef HELIO_USE_MULTITASKING
            scheduleSignalFireOnce<Helio_DrivingState>(_drivingSignal, _drivingState);
        #else
            _drivingSignal.fire(_drivingState);
        #endif
    }
}

void HelioIncrementalDriver::handleOffset(float maximumOffset) {
    if (!_enabled) { return; }

    auto hadDrivingState = _drivingState;
    _drivingState = maximumOffset < -(_targetRange * 0.5f) ? Helio_DrivingState_GoHigher :
                    maximumOffset > (_targetRange * 0.5f) ? Helio_DrivingState_GoLower :
                    Helio_DrivingState_OnTarget;

    if (_drivingState != Helio_DrivingState_OnTarget && _drivingState != Helio_DrivingState_Undefined && _targetSetpoint != FLT_UNDEF) {
        for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
            HelioPositionSensorAttachmentInterface *positionInt = attachIter->HelioAttachment::get<HelioPositionSensorAttachmentInterface>();
            float position = positionInt->getPosition().getMeasurementValue();

            if (fabsf(position - _targetSetpoint) <= _targetRange) {
                attachIter->disableActivation();
            } else if (position < _targetSetpoint) {
                attachIter->setupActivation(_travelRate);
                attachIter->enableActivation();
            } else if (position > _targetSetpoint) {
                attachIter->setupActivation(-_travelRate);
                attachIter->enableActivation();
            }
        }
    } else {
        disableAllActivations();
    }

    if (hadDrivingState != _drivingState && _drivingState != Helio_DrivingState_Undefined) {
        #ifdef HELIO_USE_MULTITASKING
            scheduleSignalFireOnce<Helio_DrivingState>(_drivingSignal, _drivingState);
        #else
            _drivingSignal.fire(_drivingState);
        #endif
    }
}
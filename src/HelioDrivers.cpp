/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Drivers
*/

#include "Helioduino.h"

HelioDriver::HelioDriver(float targetSetpoint, float travelRate, int typeIn)
    : type((typeof(type))typeIn), _trackRange(make_pair(__FLT_MAX__,-__FLT_MAX__)),
      _targetSetpoint(targetSetpoint), _travelRate(travelRate),
      _drivingState(Helio_DrivingState_Undefined), _enabled(false)
{ ; }

HelioDriver::~HelioDriver()
{
    _enabled = false;
    disableAllActivations();
}

void HelioDriver::update()
{
    handleMaxOffset(getMaxTargetOffset(true));
}

void HelioDriver::setActuators(const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &actuators)
{
    for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
        bool found = false;
        auto key = attachIter->getKey();

        for (auto attachInIter = actuators.begin(); attachInIter != actuators.end(); ++attachInIter) {
            if (key == attachInIter->getKey()) {
                auto activation = *attachInIter;
                activation.setupActivation(attachIter->getActivationSetup());
                if (attachIter->getUpdateSlot()) { activation.setUpdateSlot(*attachIter->getUpdateSlot()); }
                found = true;
                break;
            }
        }

        if (!found) { // disables activations not found in new list
            attachIter->disableActivation();
        }
    }

    {   _actuators.clear();
        float trackMin = __FLT_MAX__, trackMax = -__FLT_MAX__;
        for (auto attachInIter = actuators.begin(); attachInIter != actuators.end(); ++attachInIter) {
            _actuators.push_back(*attachInIter);
            _actuators.back().setParent(this);
            auto actuator = _actuators.back().get();
            if (actuator) {
                auto trackExtents = actuator->getTravelRange();
                if (trackExtents.first < trackMin) { trackMin = trackExtents.first; }
                if (trackExtents.second > trackMax) { trackMax = trackExtents.second; }
            }
        }
        _trackRange = make_pair(trackMin, trackMax);
    }
}

float HelioDriver::getMaxTargetOffset(bool poll)
{
    float maxDelta = 0;

    for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
        if ((*attachIter)->isAnyMotorClass()) {
            auto position = attachIter->HelioAttachment::get<HelioPositionSensorAttachmentInterface>()->getPositionSensorAttachment().getMeasurement(poll).asUnits(getMeasurementUnits());

            float delta = _targetSetpoint - position.value;
            if (fabsf(delta) > maxDelta) { maxDelta = delta; }
        } else {
            float delta = _targetSetpoint - (*attachIter)->getCalibratedValue();
            if (fabsf(delta) > maxDelta) { maxDelta = delta; }
        }
    }

    return maxDelta;
}

Helio_DrivingState HelioDriver::getDrivingState(bool poll)
{
    if (poll) { return getMaxTargetOffset(true) > FLT_EPSILON ? Helio_DrivingState_OffTarget : Helio_DrivingState_AlignedTarget; }
    return _drivingState;
}

void HelioDriver::setTargetSetpoint(float targetSetpoint)
{
    if (!isFPEqual(_targetSetpoint, targetSetpoint)) {
        _targetSetpoint = targetSetpoint;
        bumpRevisionIfNeeded();
    }
}

void HelioDriver::setEnabled(bool enabled)
{
    _enabled = enabled;
}

void HelioDriver::setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t measurementRow)
{
    if (_measurementUnits[measurementRow] != measurementUnits) {
        _measurementUnits[measurementRow] = measurementUnits;
    }
}

Helio_UnitsType HelioDriver::getMeasurementUnits(uint8_t measurementRow) const
{
    return _measurementUnits[measurementRow];
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


HelioAbsoluteDriver::HelioAbsoluteDriver(float travelRate, int typeIn)
    : HelioDriver(FLT_UNDEF, travelRate, typeIn), _lastUpdate(0)
{ ; }

HelioAbsoluteDriver::~HelioAbsoluteDriver()
{ ; }

void HelioAbsoluteDriver::setEnabled(bool enabled)
{
    if (_enabled != enabled) {
        HelioDriver::setEnabled(enabled);
        _lastUpdate = 0;
    }
}

void HelioAbsoluteDriver::handleMaxOffset(float maxOffset)
{
    auto hadDrivingState = _drivingState;
    _drivingState = maxOffset > FLT_EPSILON ? Helio_DrivingState_OffTarget : Helio_DrivingState_AlignedTarget;

    if (_enabled && _drivingState != Helio_DrivingState_AlignedTarget && _targetSetpoint != FLT_UNDEF) {
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


HelioIncrementalDriver::HelioIncrementalDriver(float nearbyRange, float alignedRange, float travelRate, float maxDifference, int typeIn)
    : HelioDriver(FLT_UNDEF, travelRate, typeIn), _nearbyRange(nearbyRange), _alignedRange(alignedRange), _maxDifference(maxDifference)
{ ; }

HelioIncrementalDriver::~HelioIncrementalDriver()
{ ; }

Helio_DrivingState HelioIncrementalDriver::getDrivingState(bool poll)
{
    if (poll) {
        float maxOffset = getMaxTargetOffset(true);
        return maxOffset > _nearbyRange + FLT_EPSILON  ? Helio_DrivingState_OffTarget :
               maxOffset > _alignedRange + FLT_EPSILON ? Helio_DrivingState_NearbyTarget
                                                       : Helio_DrivingState_AlignedTarget;
    }
    return _drivingState;
}

void HelioIncrementalDriver::handleMaxOffset(float maxOffset) {
    auto hadDrivingState = _drivingState;
    _drivingState = maxOffset > _nearbyRange + FLT_EPSILON  ? Helio_DrivingState_OffTarget :
                    maxOffset > _alignedRange + FLT_EPSILON ? Helio_DrivingState_NearbyTarget
                                                            : Helio_DrivingState_AlignedTarget;

    if (_enabled && _drivingState != Helio_DrivingState_AlignedTarget && _targetSetpoint != FLT_UNDEF) {
        float offsetLimit = maxOffset - _maxDifference;

        for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
            auto position = attachIter->HelioAttachment::get<HelioPositionSensorAttachmentInterface>()->getPositionSensorAttachment().getMeasurement(true).asUnits(getMeasurementUnits());
            float offset = fabsf(_targetSetpoint - position.value);

            if (offset <= _alignedRange + FLT_EPSILON || offset < offsetLimit - FLT_EPSILON) { // aligned or too fast
                attachIter->disableActivation();
            } else {
                attachIter->setupActivation(_targetSetpoint > position.value ? _travelRate : -_travelRate);
                attachIter->setRateMultiplier((offset <= _nearbyRange + FLT_EPSILON ? HELIO_DRV_FINETRAVEL_RATEMULT : 1.0f));
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

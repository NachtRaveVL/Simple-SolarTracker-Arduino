/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Drivers
*/

#include "Helioduino.h"
#include "HelioCoreLogic.h"

HelioDriver::HelioDriver(float targetSetpoint, float travelRate, int typeIn)
    : type(static_cast<decltype(Absolute)>(typeIn)), _trackRange(make_pair(__FLT_MAX__,-__FLT_MAX__)),
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
    float maxDelta = 0.0f;

    for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
        float delta = 0.0f;

        if ((*attachIter)->isAnyMotorClass()) {
            auto position = attachIter->HelioAttachment::get<HelioPositionSensorAttachmentInterface>()->getPositionSensorAttachment().getMeasurement(poll).asUnits(getMeasurementUnits());
            delta = _targetSetpoint - position.value;
        } else {
            delta = _targetSetpoint - (*attachIter)->getCalibratedValue();
        }

        maxDelta = helioLargerMagnitude(maxDelta, delta);
    }

    return maxDelta;
}

Helio_DrivingState HelioDriver::getDrivingState(bool poll)
{
    if (poll) { return fabsf(getMaxTargetOffset(true)) > FLT_EPSILON ? Helio_DrivingState_OffTarget : Helio_DrivingState_AlignedTarget; }
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

void HelioDriver::setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t)
{
    if (_measurementUnits[0] != measurementUnits) {
        _measurementUnits[0] = measurementUnits;
        //bumpRevisionIfNeeded();
    }
}

Helio_UnitsType HelioDriver::getMeasurementUnits(uint8_t) const
{
    return _measurementUnits[0];
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
    _drivingState = fabsf(maxOffset) > FLT_EPSILON ? Helio_DrivingState_OffTarget : Helio_DrivingState_AlignedTarget;

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


HelioIncrementalDriver::HelioIncrementalDriver(float nearbyRange, float alignedRange, float maxDifference, float travelRate, int typeIn)
    : HelioDriver(FLT_UNDEF, travelRate, typeIn),
      _nearbyRange(fabsf(nearbyRange)), _alignedRange(fabsf(alignedRange)), _maxDifference(fabsf(maxDifference)),
      _lastTravelDirection(0)
{ ; }

HelioIncrementalDriver::~HelioIncrementalDriver()
{ ; }

float HelioIncrementalDriver::getCoastDistance(HelioActuatorAttachment &attachment) const
{
    if (!attachment.resolve() || !attachment->isRelayMotorClass()) { return 0.0f; }
    auto motor = attachment.HelioAttachment::get<HelioRelayMotorActuator>();
    return motor ? motor->getCoastDistance(getMeasurementUnits()) : 0.0f;
}

bool HelioIncrementalDriver::shouldHold(HelioActuatorAttachment &attachment, float signedOffset) const
{
    return helioShouldHoldIncrementalMotor(signedOffset, _lastTravelDirection,
                                           _alignedRange, _nearbyRange,
                                           getCoastDistance(attachment));
}

Helio_DrivingState HelioIncrementalDriver::getDrivingState(bool poll)
{
    if (poll) {
        float maxMagnitude = 0.0f;
        bool needsTravel = false;

        for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
            auto position = attachIter->HelioAttachment::get<HelioPositionSensorAttachmentInterface>()->getPositionSensorAttachment().getMeasurement(true).asUnits(getMeasurementUnits());
            const float signedOffset = _targetSetpoint - position.value;
            const float offset = fabsf(signedOffset);
            if (offset > maxMagnitude) { maxMagnitude = offset; }
            if (!shouldHold(*attachIter, signedOffset)) { needsTravel = true; }
        }

        if (!needsTravel) { return Helio_DrivingState_AlignedTarget; }
        return maxMagnitude > _nearbyRange + FLT_EPSILON ? Helio_DrivingState_OffTarget
                                                          : Helio_DrivingState_NearbyTarget;
    }
    return _drivingState;
}

void HelioIncrementalDriver::handleMaxOffset(float maxOffset)
{
    auto hadDrivingState = _drivingState;
    const float maxMagnitude = fabsf(maxOffset);
    bool needsTravel = false;

    for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
        auto position = attachIter->HelioAttachment::get<HelioPositionSensorAttachmentInterface>()->getPositionSensorAttachment().getMeasurement(true).asUnits(getMeasurementUnits());
        if (!shouldHold(*attachIter, _targetSetpoint - position.value)) {
            needsTravel = true;
            break;
        }
    }

    _drivingState = !needsTravel ? Helio_DrivingState_AlignedTarget :
                    maxMagnitude > _nearbyRange + FLT_EPSILON ? Helio_DrivingState_OffTarget
                                                              : Helio_DrivingState_NearbyTarget;

    if (_enabled && needsTravel && _targetSetpoint != FLT_UNDEF) {
        const float offsetLimit = maxMagnitude - _maxDifference;

        for (auto attachIter = _actuators.begin(); attachIter != _actuators.end(); ++attachIter) {
            auto position = attachIter->HelioAttachment::get<HelioPositionSensorAttachmentInterface>()->getPositionSensorAttachment().getMeasurement(true).asUnits(getMeasurementUnits());
            const float signedOffset = _targetSetpoint - position.value;
            const float offset = fabsf(signedOffset);

            if (offset < offsetLimit - FLT_EPSILON || shouldHold(*attachIter, signedOffset)) { // aligned/coasting or too far ahead of other actuators
                attachIter->disableActivation();
            } else {
                const int direction = helioDirectionForOffset(signedOffset, FLT_EPSILON);
                if (!direction) {
                    attachIter->disableActivation();
                    continue;
                }

                _lastTravelDirection = direction;
                attachIter->setupActivation(direction > 0 ? _travelRate : -_travelRate);
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

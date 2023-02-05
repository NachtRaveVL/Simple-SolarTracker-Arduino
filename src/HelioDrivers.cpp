/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Drivers
*/

#include "Helioduino.h"

HelioDriver::HelioDriver(float trackMin, float trackMax, float targetSetpoint, float travelRate, int typeIn)
    : type((typeof(type))typeIn), _drivingState(Helio_DrivingState_Undefined), _enabled(false),
      _trackMin(trackMin), _trackMax(trackMax), _targetSetpoint(targetSetpoint), _travelRate(travelRate)
{ ; }

HelioDriver::~HelioDriver()
{
    _enabled = false;
    disableAllActivations();
}

void HelioDriver::update()
{
    for (auto activationIter = _actuators.begin(); activationIter != _actuators.end(); ++activationIter) {
        activationIter->updateIfNeeded();
    }

    // todo actual updates
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
        for (auto activationIter = _actuators.begin(); activationIter != _actuators.end(); ++activationIter) {
            activationIter->setRateMultiplier(travelRate);
            // todo recalculate actuator intensity / dirty flag
        }
    }
}

Helio_DrivingState HelioDriver::getDrivingState() const
{
    return _drivingState;
}

void HelioDriver::setActuators(const Vector<HelioActuatorAttachment, HELIO_DRV_ACTUATORS_MAXSIZE> &actuators)
{
    for (auto activationIter = _actuators.begin(); activationIter != _actuators.end(); ++activationIter) {
        bool found = false;
        auto key = activationIter->getKey();

        for (auto activationInIter = actuators.begin(); activationInIter != actuators.end(); ++activationInIter) {
            if (key == activationInIter->getKey()) {
                found = true;
                break;
            }
        }

        if (!found) { // disables activations not found in new list
            activationIter->disableActivation();
        }
    }

    {   _actuators.clear();
        for (auto activationInIter = actuators.begin(); activationInIter != actuators.end(); ++activationInIter) {
            _actuators.push_back(*activationInIter);
            _actuators.back().setParent(this);
        }
    }
}

Signal<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> &HelioDriver::getDrivingSignal()
{
    return _drivingSignal;
}

void HelioDriver::disableAllActivations()
{
    for (auto activationIter = _actuators.begin(); activationIter != _actuators.end(); ++activationIter) {
        activationIter->disableActivation();
    }
}


// HelioServoDriver::HelioServoDriver(SharedPtr<HelioSensor> sensor, float targetSetpoint, float targetRange, float edgeOffset, float edgeLength, uint8_t measurementRow)
//     : HelioDriver(sensor, targetSetpoint, targetRange, measurementRow, Servo), _edgeOffset(edgeOffset), _edgeLength(edgeLength)
// { ; }

// void HelioServoDriver::update()
// {
//     HelioDriver::update();
//     if (!_enabled || !_sensor) { return; }

//     if (_drivingState != Helio_DrivingState_OnTarget && _drivingState != Helio_DrivingState_Undefined) {
//         auto measure = _sensor.getMeasurement(true);

//         float x = fabsf(measure.value - _targetSetpoint);
//         float val = _edgeLength > FLT_EPSILON ? mapValue<float>(x, _edgeOffset, _edgeOffset + _edgeLength, 0.0f, 1.0f)
//                                               : (x >= _edgeOffset - FLT_EPSILON ? 1.0 : 0.0f);
//         val = constrain(val, 0.0f, 1.0f);

//         if (_drivingState == Helio_DrivingState_GoHigher) {
//             for (auto activationIter = _actuators.begin(); activationIter != _actuators.end(); ++activationIter) {
//                 activationIter->setupActivation(val * activationIter->getRateMultiplier());
//                 activationIter->enableActivation();
//             }
//         } else {
//             for (auto activationIter = _actuators.begin(); activationIter != _actuators.end(); ++activationIter) {
//                 activationIter->setupActivation(-val * activationIter->getRateMultiplier());
//                 activationIter->enableActivation();
//             }
//         }
//     }
// }

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Panels
*/

#include "Helioduino.h"

HelioPanel *newPanelObjectFromData(const HelioPanelData *dataIn)
{
    if (dataIn && isValidType(dataIn->id.object.idType)) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && dataIn->isObjectData(), SFP(HStr_Err_InvalidParameter));

    if (dataIn && dataIn->isObjectData()) {
        switch (dataIn->id.object.classType) {
            case (hid_t)HelioPanel::Balancing:
                return new HelioBalancingPanel((const HelioBalancingPanelData *)dataIn);
            case (hid_t)HelioPanel::Tracking:
                return new HelioTrackingPanel((const HelioTrackingPanelData *)dataIn);
            case (hid_t)HelioPanel::Reflecting:
                return new HelioReflectingPanel((const HelioReflectingPanelData *)dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioPanel::HelioPanel(Helio_PanelType panelType, hposi_t panelIndex, float alignedTolerance, int classTypeIn)
    : HelioObject(HelioIdentity(panelType, panelIndex)), classType((typeof(classType))classTypeIn),
      HelioPowerUnitsInterfaceStorage(defaultPowerUnits()), _coverDriver(this),
      _panelState(Helio_PanelState_Undefined), _alignedTolerance(alignedTolerance),
      _homePosition{0}, _axisOffset{0}, _inDaytimeMode(false),
      _isHorzCoords(!getIsEquatorialCoordsFromType(panelType)),
      _drivesHorz(getDrivesHorizontalAxis(panelType)), _drivesVert(getDrivesVerticalAxis(panelType)),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this,0),HelioDriverAttachment(this,1)}
{
    allocateLinkages(HELIO_PANEL_LINKS_BASESIZE);
    _powerProd.setMeasurementUnits(getPowerUnits());
}

HelioPanel::HelioPanel(const HelioPanelData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))(dataIn->id.object.classType)),
      HelioPowerUnitsInterfaceStorage(definedUnitsElse(dataIn->powerUnits, defaultPowerUnits())),
      _panelState(Helio_PanelState_Undefined),
      _homePosition{dataIn->homePosition[0],dataIn->homePosition[1]},
      _axisOffset{dataIn->axisOffset[0],dataIn->axisOffset[1]},
      _inDaytimeMode(false), _coverDriver(this),
      _isHorzCoords(!getIsEquatorialCoordsFromType((Helio_PanelType)dataIn->id.object.objType)),
      _drivesHorz(getDrivesHorizontalAxis((Helio_PanelType)dataIn->id.object.objType)),
      _drivesVert(getDrivesVerticalAxis((Helio_PanelType)dataIn->id.object.objType)),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this,0),HelioDriverAttachment(this,1)}
{
    allocateLinkages(HELIO_PANEL_LINKS_BASESIZE);
    _powerProd.setMeasurementUnits(getPowerUnits());
    _powerProd.setObject(dataIn->powerProdSensor);
}

HelioPanel::~HelioPanel()
{ ; }

void HelioPanel::update()
{
    HelioObject::update();

    _powerProd.updateIfNeeded(true);

    _axisDriver[0].updateIfNeeded(true);
    _axisDriver[1].updateIfNeeded(true);

    if (isDaylight(true)) {
        handleState(isAligned(true) ? Helio_PanelState_AlignedToSun : Helio_PanelState_TravelingToSun);
    } else {
        handleState(isAligned(true) ? Helio_PanelState_AlignedToHome : Helio_PanelState_TravelingToHome);
    }
}

bool HelioPanel::canActivate(HelioActuator *actuator)
{
    switch (actuator->getActuatorType()) {
        case Helio_ActuatorType_PanelBrake:
            return _panelState == Helio_PanelState_AlignedToSun || _panelState == Helio_PanelState_AlignedToHome;
        case Helio_ActuatorType_PanelCover:
        case Helio_ActuatorType_PanelSprayer:
            return _panelState == Helio_PanelState_AlignedToHome;
        case Helio_ActuatorType_PanelHeater:
            return true;
        default:
            if (actuator->isTravelType()) {
                auto panelBrakes = linksFilterActuatorsByPanelAndType(getLinkages(), this, Helio_ActuatorType_PanelBrake);
                for (auto brakeIter = panelBrakes.begin(); brakeIter != panelBrakes.end(); ++brakeIter) {
                    HelioActuator *brake = (HelioActuator *)*brakeIter;
                    if (brake->isEnabled()) { return false; }
                }
            }
            return true;
    }
}

bool HelioPanel::isDaylight(bool poll)
{
    return _inDaytimeMode;
}

void HelioPanel::setPowerUnits(Helio_UnitsType powerUnits)
{
    if (_powerUnits != powerUnits) {
        _powerUnits = powerUnits;

        _powerProd.setMeasurementUnits(powerUnits);
    }
}

HelioSensorAttachment &HelioPanel::getPowerProductionSensorAttachment()
{
    return _powerProd;
}

Signal<Helio_PanelState, HELIO_PANEL_SIGNAL_SLOTS> &HelioPanel::getStateSignal()
{
    return _stateSignal;
}

HelioData *HelioPanel::allocateData() const
{
    return _allocateDataForObjType((int8_t)_id.type, (int8_t)classType);
}

void HelioPanel::saveToData(HelioData *dataOut)
{
    HelioObject::saveToData(dataOut);

    dataOut->id.object.classType = (int8_t)classType;
    ((HelioPanelData *)dataOut)->powerUnits = getPowerUnits();
    ((HelioPanelData *)dataOut)->alignedTolerance = _alignedTolerance;
    ((HelioPanelData *)dataOut)->homePosition[0] = _homePosition[0];
    ((HelioPanelData *)dataOut)->homePosition[1] = _homePosition[1];
    ((HelioPanelData *)dataOut)->axisOffset[0] = _axisOffset[0];
    ((HelioPanelData *)dataOut)->axisOffset[1] = _axisOffset[1];
    if (_powerProd.isSet()) {
        strncpy(((HelioPanelData *)dataOut)->powerProdSensor, _powerProd.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
}


HelioBalancingPanel::HelioBalancingPanel(Helio_PanelType panelType, hposi_t panelIndex, float intTolerance, float minIntensity, int classType)
    : HelioPanel(panelType, panelIndex, intTolerance, classType),
      _minIntensity(minIntensity),
      _ldrSensors{HelioSensorAttachment(this,0),HelioSensorAttachment(this,1),HelioSensorAttachment(this,2),HelioSensorAttachment(this,3)}
{ ; }

HelioBalancingPanel::HelioBalancingPanel(const HelioBalancingPanelData *dataIn)
    : HelioPanel(dataIn),
      _minIntensity(dataIn->minIntensity),
      _ldrSensors{HelioSensorAttachment(this,0),HelioSensorAttachment(this,1),HelioSensorAttachment(this,2),HelioSensorAttachment(this,3)}
{
    _ldrSensors[0].setObject(dataIn->ldrSensorHorzMin);
    _ldrSensors[1].setObject(dataIn->ldrSensorHorzMax);
    _ldrSensors[2].setObject(dataIn->ldrSensorVertMin);
    _ldrSensors[3].setObject(dataIn->ldrSensorVertMax);
}

HelioBalancingPanel::~HelioBalancingPanel()
{ ; }

void HelioBalancingPanel::update()
{
    _ldrSensors[0].updateIfNeeded(true);
    _ldrSensors[1].updateIfNeeded(true);
    _ldrSensors[2].updateIfNeeded(true);
    _ldrSensors[3].updateIfNeeded(true);

    HelioPanel::update();
}

bool HelioBalancingPanel::isAligned(bool poll)
{
    if (poll) {
        bool aligned = true;
        if (drivesHorizontalAxis()) { // purposefully leaving out pre-checks to force driving of some getMeasurement() calls
            auto intHorzMin = _ldrSensors[0].getMeasurement(true).asUnits(Helio_UnitsType_Raw_1);
            auto intHorzMax = _ldrSensors[1].getMeasurement(true).asUnits(Helio_UnitsType_Raw_1);

            if (isDaylight(true) && _ldrSensors[0] && _ldrSensors[1]) {
                aligned = aligned && intHorzMin.value >= _minIntensity - FLT_EPSILON &&
                                     intHorzMax.value >= _minIntensity - FLT_EPSILON &&
                          fabsf(intHorzMax.value - intHorzMin.value) <= _alignedTolerance + FLT_EPSILON;
            } else {
                aligned = aligned && _axisDriver[0].resolve() && isFPEqual(_homePosition[0], _axisDriver[0]->getTargetSetpoint()) && _axisDriver[0]->isAligned(true);
            }
        }
        if (drivesVerticalAxis()) { // purposefully leaving out pre-checks to force driving of some getMeasurement() calls
            auto intVertMin = _ldrSensors[2].getMeasurement(true).asUnits(Helio_UnitsType_Raw_1);
            auto intVertMax = _ldrSensors[3].getMeasurement(true).asUnits(Helio_UnitsType_Raw_1);

            if (isDaylight(true) && _ldrSensors[2] && _ldrSensors[3]) {
                aligned = aligned && intVertMin.value >= _minIntensity - FLT_EPSILON &&
                                     intVertMax.value >= _minIntensity - FLT_EPSILON &&
                          fabsf(intVertMax.value - intVertMin.value) <= _alignedTolerance + FLT_EPSILON;
            } else {
                aligned = aligned && _axisDriver[1].resolve() && isFPEqual(_homePosition[1], _axisDriver[1]->getTargetSetpoint()) && _axisDriver[1]->isAligned(true);
            }
        }
        return aligned;
    }
    return _panelState == (isDaylight() ? Helio_PanelState_AlignedToSun : Helio_PanelState_AlignedToHome);
}

void HelioBalancingPanel::saveToData(HelioData *dataOut)
{
    HelioPanel::saveToData(dataOut);

    ((HelioBalancingPanelData *)dataOut)->minIntensity = _minIntensity;
    if (_ldrSensors[0].isSet()) {
        strncpy(((HelioBalancingPanelData *)dataOut)->ldrSensorHorzMin, _ldrSensors[0].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_ldrSensors[1].isSet()) {
        strncpy(((HelioBalancingPanelData *)dataOut)->ldrSensorHorzMax, _ldrSensors[1].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_ldrSensors[2].isSet()) {
        strncpy(((HelioBalancingPanelData *)dataOut)->ldrSensorVertMin, _ldrSensors[2].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_ldrSensors[3].isSet()) {
        strncpy(((HelioBalancingPanelData *)dataOut)->ldrSensorVertMax, _ldrSensors[3].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
}

void HelioBalancingPanel::handleState(Helio_PanelState panelState)
{
    if (panelState == Helio_PanelState_Undefined) { return; }
    Helio_PanelState hadPanelState = _panelState;
    _panelState = panelState;

    if (!(_panelState == Helio_PanelState_AlignedToSun || _panelState == Helio_PanelState_AlignedToHome)) {
        if (drivesHorizontalAxis() && _axisDriver[0].resolve()) {
            auto intHorzMin = _ldrSensors[0].getMeasurement().asUnits(Helio_UnitsType_Raw_1);
            auto intHorzMax = _ldrSensors[1].getMeasurement().asUnits(Helio_UnitsType_Raw_1);

            if (intHorzMin.value > FLT_EPSILON && intHorzMax.value > FLT_EPSILON) {
                if (fabsf(intHorzMax.value - intHorzMin.value) <= _alignedTolerance + FLT_EPSILON) {
                    _axisDriver[0]->setEnabled(false);
                } else {
                    _axisDriver[0]->setEnabled(true);
                    auto setPoint = _axisDriver[0]->getTargetSetpoint();
                    auto rateStep = (HELIO_CONTROL_LOOP_INTERVAL / 1000.0f) * _axisDriver[0]->getTravelRate();
                    if (intHorzMin.value > intHorzMax.value) {
                        setPoint -= rateStep;
                    } else {
                        setPoint += rateStep;
                    }
                    _axisDriver[0]->setTargetSetpoint(setPoint);
                }
            }
        }
        if (drivesVerticalAxis() && _axisDriver[1].resolve()) {
            auto intVertMin = _ldrSensors[2].getMeasurement().asUnits(Helio_UnitsType_Raw_1);
            auto intVertMax = _ldrSensors[3].getMeasurement().asUnits(Helio_UnitsType_Raw_1);

            if (intVertMin.value > FLT_EPSILON && intVertMax.value > FLT_EPSILON) {
                if (fabsf(intVertMax.value - intVertMin.value) <= _alignedTolerance + FLT_EPSILON) {
                    _axisDriver[1]->setEnabled(false);
                } else {
                    _axisDriver[1]->setEnabled(true);
                    auto setPoint = _axisDriver[1]->getTargetSetpoint();
                    auto rateStep = (HELIO_CONTROL_LOOP_INTERVAL / 1000.0f) * _axisDriver[1]->getTravelRate();
                    if (intVertMin.value > intVertMax.value) {
                        setPoint -= rateStep;
                    } else {
                        setPoint += rateStep;
                    }
                    _axisDriver[1]->setTargetSetpoint(setPoint);
                }
            }
        }
    } else {
        if (_axisDriver[0].resolve()) { _axisDriver[0]->setEnabled(false); }
        if (_axisDriver[1].resolve()) { _axisDriver[1]->setEnabled(false); }
    }

    if (hadPanelState != _panelState && _panelState != Helio_PanelState_Undefined) {
        #ifdef HELIO_USE_MULTITASKING
            scheduleSignalFireOnce<Helio_PanelState>(_stateSignal, _panelState);
        #else
            _stateSignal.fire(_panelState);
        #endif
    }
}


HelioTrackingPanel::HelioTrackingPanel(Helio_PanelType panelType, hposi_t panelIndex, float angleTolerance, int classType)
    : HelioPanel(panelType, panelIndex, angleTolerance, classType),
      HelioTemperatureUnitsInterfaceStorage(defaultTemperatureUnits()),
      HelioDistanceUnitsInterfaceStorage(defaultDistanceUnits()),
      _lastAlignedTime(0), _locationOffset{0}, _sunPosition{0}, _facingPosition{0},
      _powerUsage(this), _axisAngle{HelioSensorAttachment(this,0),HelioSensorAttachment(this,1)},
      _temperature(this), _windSpeed(this), _heatingTrigger(this), _stormingTrigger(this)
{
    _axisAngle[0].setMeasurementUnits(Helio_UnitsType_Angle_Degrees_360);
    _axisAngle[1].setMeasurementUnits(Helio_UnitsType_Angle_Degrees_360);
    _powerUsage.setMeasurementUnits(getPowerUnits());
    _temperature.setMeasurementUnits(getTemperatureUnits());
    _windSpeed.setMeasurementUnits(getSpeedUnits());
}

HelioTrackingPanel::HelioTrackingPanel(const HelioTrackingPanelData *dataIn)
    : HelioPanel(dataIn),
      HelioTemperatureUnitsInterfaceStorage(definedUnitsElse(dataIn->temperatureUnits, defaultTemperatureUnits())),
      HelioDistanceUnitsInterfaceStorage(definedUnitsElse(dataIn->distanceUnits, defaultDistanceUnits())),
      _lastAlignedTime(dataIn->lastAlignedTime), _sunPosition{0},
      _locationOffset{dataIn->locationOffset[0], dataIn->locationOffset[1]},
      _facingPosition{dataIn->axisPosition[0], dataIn->axisPosition[1]},
      _powerUsage(this), _axisAngle{HelioSensorAttachment(this,0),HelioSensorAttachment(this,1)},
      _temperature(this), _windSpeed(this), _heatingTrigger(this), _stormingTrigger(this)
{
    _powerUsage.setMeasurementUnits(getPowerUnits());
    _powerUsage.setObject(dataIn->powerUsageSensor);

    _axisAngle[0].setMeasurementUnits(Helio_UnitsType_Angle_Degrees_360);
    _axisAngle[0].setObject(dataIn->axisSensorHorz);

    _axisAngle[1].setMeasurementUnits(Helio_UnitsType_Angle_Degrees_360);
    _axisAngle[1].setObject(dataIn->axisSensorVert);

    _temperature.setMeasurementUnits(getTemperatureUnits());
    _temperature.setObject(dataIn->temperatureSensor);

    _windSpeed.setMeasurementUnits(getSpeedUnits());
    _windSpeed.setObject(dataIn->windSpeedSensor);

    _heatingTrigger.setObject(newTriggerObjectFromSubData(&dataIn->heatingTrigger));
    _stormingTrigger.setObject(newTriggerObjectFromSubData(&dataIn->stormingTrigger));
}

HelioTrackingPanel::~HelioTrackingPanel()
{ ; }

void HelioTrackingPanel::update()
{
    _powerUsage.updateIfNeeded(true);
    _axisAngle[0].updateIfNeeded(true);
    _axisAngle[1].updateIfNeeded(true);
    _temperature.updateIfNeeded(true);
    _windSpeed.updateIfNeeded(true);
    _heatingTrigger.updateIfNeeded(true);
    _stormingTrigger.updateIfNeeded(true);

    HelioPanel::update();
}

bool HelioTrackingPanel::canActivate(HelioActuator *actuator)
{
    switch (actuator->getActuatorType()) {
        case Helio_ActuatorType_PanelHeater:
            return !_heatingTrigger.resolve() || triggerStateToBool(_heatingTrigger.getTriggerState());
        default:
            return HelioPanel::canActivate(actuator);
    }
}

bool HelioTrackingPanel::isDaylight(bool poll)
{
    return _inDaytimeMode && (!_stormingTrigger.resolve() || triggerStateToBool(_stormingTrigger.getTriggerState(poll)));
}

bool HelioTrackingPanel::isAligned(bool poll)
{
    if (poll) {
        recalcSunPosition();
        recalcFacingPosition();

        bool aligned = true;
        if (aligned && drivesHorizontalAxis()) {
            if (_axisAngle[0].resolve()) {
                auto axisPosHorz = _axisAngle[0].getMeasurement().asUnits(Helio_UnitsType_Angle_Degrees_360).wrappedBy360();
                aligned = aligned && fabsf(_facingPosition[0] - axisPosHorz.value) <= _alignedTolerance + FLT_EPSILON;
            }
            aligned = aligned && _axisDriver[0].resolve() && isFPEqual(_facingPosition[0], _axisDriver[0]->getTargetSetpoint()) && _axisDriver[0]->isAligned(true);
        }
        if (aligned && drivesVerticalAxis()) {
            if (_axisAngle[1].resolve()) {
                auto axisPosVert = _axisAngle[1].getMeasurement().asUnits(Helio_UnitsType_Angle_Degrees_360).wrappedBy360();
                aligned = aligned && fabsf(_facingPosition[1] - axisPosVert.value) <= _alignedTolerance + FLT_EPSILON;
            }
            aligned = aligned && _axisDriver[1].resolve() && isFPEqual(_facingPosition[1], _axisDriver[1]->getTargetSetpoint()) && _axisDriver[1]->isAligned(true);
        }
        return aligned;
    }
    return _panelState == (isDaylight() ? Helio_PanelState_AlignedToSun : Helio_PanelState_AlignedToHome);
}

void HelioTrackingPanel::recalcSunPosition()
{
    time_t time = unixNow();
    JulianDay julianTime(time);

    if (isHorizontalCoords()) {
        Location location = getController() ? getController()->getSystemLocation() : Location();
        if (location.hasPosition()) {
            calcHorizontalCoordinates(julianTime, location.latitude + _locationOffset[0], location.longitude + _locationOffset[1], _sunPosition[0], _sunPosition[1]);
        }
    } else if (isEquatorialCoords()) {
        double radius;
        calcEquatorialCoordinates(julianTime, _sunPosition[0], _sunPosition[1], radius);
    } else {
        HELIO_SOFT_ASSERT(false, SFP(HStr_Err_UnsupportedOperation));
    }
}

void HelioTrackingPanel::recalcFacingPosition()
{
    if (drivesHorizontalAxis()) {
        if (isDaylight()) {
            _facingPosition[0] = wrapBy360(_sunPosition[0] + _axisOffset[0]);
        } else {
            _facingPosition[0] = wrapBy360(_homePosition[0] + _axisOffset[0]);
        }
        if (_axisDriver[0].resolve()) { _axisDriver[0]->setTargetSetpoint(_facingPosition[0]); }
    }
    if (drivesVerticalAxis()) {
        if (isDaylight()) {
            _facingPosition[1] = wrapBy180Neg180(_sunPosition[1] + _axisOffset[1]);
        } else {
            _facingPosition[1] = wrapBy180Neg180(_homePosition[1] + _axisOffset[1]);
        }
        if (_axisDriver[1].resolve()) { _axisDriver[1]->setTargetSetpoint(_facingPosition[1]); }
    }
}

void HelioTrackingPanel::setPowerUnits(Helio_UnitsType powerUnits)
{
    if (_powerUnits != powerUnits) {
        _powerUnits = powerUnits;

        _powerProd.setMeasurementUnits(getPowerUnits());
        _powerUsage.setMeasurementUnits(getPowerUnits());
    }
}

void HelioTrackingPanel::setTemperatureUnits(Helio_UnitsType temperatureUnits)
{
    if (_tempUnits != temperatureUnits) {
        _tempUnits = temperatureUnits;

        _temperature.setMeasurementUnits(getTemperatureUnits());
    }
}

void HelioTrackingPanel::setDistanceUnits(Helio_UnitsType distanceUnits)
{
    if (_distUnits != distanceUnits) {
        _distUnits = distanceUnits;

        _windSpeed.setMeasurementUnits(getSpeedUnits());
    }
}

HelioSensorAttachment &HelioTrackingPanel::getPowerUsageSensorAttachment()
{
    return _powerUsage;
}

HelioSensorAttachment &HelioTrackingPanel::getTemperatureSensorAttachment()
{
    return _temperature;
}

HelioSensorAttachment &HelioTrackingPanel::getWindSpeedSensorAttachment()
{
    return _windSpeed;
}

void HelioTrackingPanel::saveToData(HelioData *dataOut)
{
    HelioPanel::saveToData(dataOut);

    ((HelioTrackingPanelData *)dataOut)->lastAlignedTime = _lastAlignedTime;
    ((HelioTrackingPanelData *)dataOut)->locationOffset[0] = _locationOffset[0];
    ((HelioTrackingPanelData *)dataOut)->locationOffset[1] = _locationOffset[1];
    if (_axisAngle[0].isSet()) {
        ((HelioTrackingPanelData *)dataOut)->axisPosition[0] = _axisAngle[0].getMeasurement(true).asUnits(Helio_UnitsType_Angle_Degrees_360).wrapBy360().value;
        strncpy(((HelioTrackingPanelData *)dataOut)->axisSensorHorz, _axisAngle[0].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_axisAngle[1].isSet()) {
        ((HelioTrackingPanelData *)dataOut)->axisPosition[1] = _axisAngle[1].getMeasurement(true).asUnits(Helio_UnitsType_Angle_Degrees_360).wrapBy180Neg180().value;
        strncpy(((HelioTrackingPanelData *)dataOut)->axisSensorVert, _axisAngle[1].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_powerUsage.isSet()) {
        strncpy(((HelioTrackingPanelData *)dataOut)->powerUsageSensor, _powerUsage.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_temperature.isSet()) {
        strncpy(((HelioTrackingPanelData *)dataOut)->temperatureSensor, _temperature.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_windSpeed.isSet()) {
        strncpy(((HelioTrackingPanelData *)dataOut)->windSpeedSensor, _windSpeed.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_heatingTrigger.isSet()) {
        _heatingTrigger->saveToData(&(((HelioTrackingPanelData *)dataOut)->heatingTrigger));
    }
    if (_stormingTrigger.isSet()) {
        _stormingTrigger->saveToData(&(((HelioTrackingPanelData *)dataOut)->stormingTrigger));
    }
}

void HelioTrackingPanel::handleState(Helio_PanelState panelState)
{
    if (panelState == Helio_PanelState_Undefined) { return; }
    Helio_PanelState hadPanelState = _panelState;
    _panelState = panelState;

    if (_panelState != (isDaylight() ? Helio_PanelState_AlignedToSun : Helio_PanelState_AlignedToHome)) {
        if (drivesHorizontalAxis() && _axisDriver[0].resolve()) { _axisDriver[0]->setEnabled(true); }
        if (drivesVerticalAxis() && _axisDriver[1].resolve()) { _axisDriver[1]->setEnabled(true); }
    } else {
        if (drivesHorizontalAxis() && _axisDriver[0].resolve()) { _axisDriver[0]->setEnabled(false); }
        if (drivesVerticalAxis() && _axisDriver[1].resolve()) { _axisDriver[1]->setEnabled(false); }
    }

    if (hadPanelState != _panelState && _panelState != Helio_PanelState_Undefined) {
        #ifdef HELIO_USE_MULTITASKING
            scheduleSignalFireOnce<Helio_PanelState>(_stateSignal, _panelState);
        #else
            _stateSignal.fire(_panelState);
        #endif
    }
}


HelioReflectingPanel::HelioReflectingPanel(Helio_PanelType panelType, hposi_t panelIndex, float angleTolerance, int classType)
    : HelioTrackingPanel(panelType, panelIndex, angleTolerance, classType),
      _reflectPosition{0}
{ ; }

HelioReflectingPanel::HelioReflectingPanel(const HelioReflectingPanelData *dataIn)
    : HelioTrackingPanel(dataIn),
      _reflectPosition{dataIn->reflectPosition[0], dataIn->reflectPosition[1]}
{ ; }

HelioReflectingPanel::~HelioReflectingPanel()
{ ; }

void HelioReflectingPanel::saveToData(HelioData *dataOut)
{
    HelioTrackingPanel::saveToData(dataOut);

    ((HelioReflectingPanelData *)dataOut)->reflectPosition[0] = _reflectPosition[0];
    ((HelioReflectingPanelData *)dataOut)->reflectPosition[1] = _reflectPosition[1];
}

void HelioReflectingPanel::recalcFacingPosition()
{
    if (drivesHorizontalAxis()) {
        if (isDaylight()) {
            _facingPosition[0] = wrapBy360((_sunPosition[0] + ((wrapBy180Neg180(_reflectPosition[0]) - wrapBy180Neg180(_sunPosition[0])) * 0.5f)) + _axisOffset[0]);
        } else {
            _facingPosition[0] = wrapBy360(_homePosition[0] + _axisOffset[0]);
        }
        if (_axisDriver[0].resolve()) { _axisDriver[0]->setTargetSetpoint(_facingPosition[0]); }
    }
    if (drivesVerticalAxis()) {
        if (isDaylight()) {
            _facingPosition[1] = wrapBy180Neg180((_sunPosition[1] + ((wrapBy180Neg180(_reflectPosition[1]) - wrapBy180Neg180(_sunPosition[1])) * 0.5f)) + _axisOffset[1]);
        } else {
            _facingPosition[1] = wrapBy180Neg180(_homePosition[1] + _axisOffset[1]);
        }
        if (_axisDriver[1].resolve()) { _axisDriver[1]->setTargetSetpoint(_facingPosition[1]); }
    }
}


HelioPanelData::HelioPanelData()
    : HelioObjectData(), powerUnits(Helio_UnitsType_Undefined), alignedTolerance(FLT_UNDEF), homePosition{0}, axisOffset{0}, powerProdSensor{0}
{
    _size = sizeof(*this);
}

void HelioPanelData::toJSONObject(JsonObject &objectOut) const
{
    HelioObjectData::toJSONObject(objectOut);

    if (powerUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_PowerUnits)] = unitsTypeToSymbol(powerUnits); }
    if (!(isFPEqual(homePosition[0],0) && isFPEqual(homePosition[1],0))) { objectOut[SFP(HStr_Key_HomePosition)] = commaStringFromArray(homePosition, 2); }
    if (!(isFPEqual(axisOffset[0],0) && isFPEqual(axisOffset[1],0))) { objectOut[SFP(HStr_Key_AxisOffset)] = commaStringFromArray(axisOffset, 2); }
    if (powerProdSensor[0]) { objectOut[SFP(HStr_Key_PowerProductionSensor)] = charsToString(powerProdSensor, HELIO_NAME_MAXSIZE); }
}

void HelioPanelData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioObjectData::fromJSONObject(objectIn);

    powerUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_PowerUnits)]);
    alignedTolerance = objectIn[SFP(HStr_Key_AlignedTolerance)] | alignedTolerance;
    JsonVariantConst homePositionVar = objectIn[SFP(HStr_Key_HomePosition)];
    commaStringToArray(homePositionVar, homePosition, 2);
    JsonVariantConst axisOffsetVar = objectIn[SFP(HStr_Key_AxisOffset)];
    commaStringToArray(axisOffsetVar, axisOffset, 2);
    const char *powerProdSensorStr = objectIn[SFP(HStr_Key_PowerProductionSensor)];
    if (powerProdSensorStr && powerProdSensorStr[0]) { strncpy(powerProdSensor, powerProdSensorStr, HELIO_NAME_MAXSIZE); }
}

HelioBalancingPanelData::HelioBalancingPanelData()
    : HelioPanelData(), minIntensity(HELIO_PANEL_ALIGN_LDRMIN), ldrSensorHorzMin{0}, ldrSensorHorzMax{0}, ldrSensorVertMin{0}, ldrSensorVertMax{0}
{
    _size = sizeof(*this);
}

void HelioBalancingPanelData::toJSONObject(JsonObject &objectOut) const
{
    HelioPanelData::toJSONObject(objectOut);

    if (alignedTolerance != FLT_UNDEF && !isFPEqual(alignedTolerance, HELIO_PANEL_ALIGN_LDRTOL)) { objectOut[SFP(HStr_Key_AlignedTolerance)] = alignedTolerance; }
    if (!isFPEqual(minIntensity, HELIO_PANEL_ALIGN_LDRMIN)) { objectOut[SFP(HStr_Key_MinIntensity)] = minIntensity; }
    if (ldrSensorHorzMin[0]) { objectOut[SFP(HStr_Key_LDRSensorHorzMin)] = charsToString(ldrSensorHorzMin, HELIO_NAME_MAXSIZE); }
    if (ldrSensorHorzMax[0]) { objectOut[SFP(HStr_Key_LDRSensorHorzMax)] = charsToString(ldrSensorHorzMax, HELIO_NAME_MAXSIZE); }
    if (ldrSensorVertMin[0]) { objectOut[SFP(HStr_Key_LDRSensorVertMin)] = charsToString(ldrSensorVertMin, HELIO_NAME_MAXSIZE); }
    if (ldrSensorVertMax[0]) { objectOut[SFP(HStr_Key_LDRSensorVertMax)] = charsToString(ldrSensorVertMax, HELIO_NAME_MAXSIZE); }
}

void HelioBalancingPanelData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioPanelData::fromJSONObject(objectIn);

    minIntensity = objectIn[SFP(HStr_Key_MinIntensity)] | minIntensity;
    const char *ldrSensorHorzMinStr = objectIn[SFP(HStr_Key_LDRSensorHorzMin)];
    if (ldrSensorHorzMinStr && ldrSensorHorzMinStr[0]) { strncpy(ldrSensorHorzMin, ldrSensorHorzMinStr, HELIO_NAME_MAXSIZE); }
    const char *ldrSensorHorzMaxStr = objectIn[SFP(HStr_Key_LDRSensorHorzMax)];
    if (ldrSensorHorzMaxStr && ldrSensorHorzMaxStr[0]) { strncpy(ldrSensorHorzMax, ldrSensorHorzMaxStr, HELIO_NAME_MAXSIZE); }
    const char *ldrSensorVertMinStr = objectIn[SFP(HStr_Key_LDRSensorVertMin)];
    if (ldrSensorVertMinStr && ldrSensorVertMinStr[0]) { strncpy(ldrSensorVertMin, ldrSensorVertMinStr, HELIO_NAME_MAXSIZE); }
    const char *ldrSensorVertMaxStr = objectIn[SFP(HStr_Key_LDRSensorVertMax)];
    if (ldrSensorVertMaxStr && ldrSensorVertMaxStr[0]) { strncpy(ldrSensorVertMax, ldrSensorVertMaxStr, HELIO_NAME_MAXSIZE); }
}

HelioTrackingPanelData::HelioTrackingPanelData()
    : HelioPanelData(), temperatureUnits(Helio_UnitsType_Undefined), distanceUnits(Helio_UnitsType_Undefined), lastAlignedTime(0),
      lastCleanedTime(0), locationOffset{0}, axisPosition{0}, axisSensorHorz{0}, axisSensorVert{0}, powerUsageSensor{0},
      temperatureSensor{0}, windSpeedSensor{0}, heatingTrigger(), stormingTrigger()
{
    _size = sizeof(*this);
}

void HelioTrackingPanelData::toJSONObject(JsonObject &objectOut) const
{
    HelioPanelData::toJSONObject(objectOut);

    if (alignedTolerance != FLT_UNDEF && !isFPEqual(alignedTolerance, HELIO_PANEL_ALIGN_DEGTOL)) { objectOut[SFP(HStr_Key_AlignedTolerance)] = alignedTolerance; }
    if (temperatureUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_TemperatureUnits)] = unitsTypeToSymbol(temperatureUnits); }
    if (distanceUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_DistanceUnits)] = unitsTypeToSymbol(distanceUnits); }
    if (lastAlignedTime) { objectOut[SFP(HStr_Key_LastAlignedTime)] = lastAlignedTime; }
    if (lastCleanedTime) { objectOut[SFP(HStr_Key_LastCleanedTime)] = lastCleanedTime; }
    if (!(isFPEqual(locationOffset[0],0) && isFPEqual(locationOffset[1],0))) { objectOut[SFP(HStr_Key_LocationOffset)] = commaStringFromArray(locationOffset, 2); }
    if (!(isFPEqual(axisPosition[0],0) && isFPEqual(axisPosition[1],0))) { objectOut[SFP(HStr_Key_AxisPosition)] = commaStringFromArray(axisPosition, 2); }
    if (axisSensorHorz[0]) { objectOut[SFP(HStr_Key_AxisSensorHorz)] = charsToString(axisSensorHorz, HELIO_NAME_MAXSIZE); }
    if (axisSensorVert[0]) { objectOut[SFP(HStr_Key_AxisSensorVert)] = charsToString(axisSensorVert, HELIO_NAME_MAXSIZE); }
    if (powerUsageSensor[0]) { objectOut[SFP(HStr_Key_PowerUsageSensor)] = charsToString(powerUsageSensor, HELIO_NAME_MAXSIZE); }
    if (temperatureSensor[0]) { objectOut[SFP(HStr_Key_TemperatureSensor)] = charsToString(temperatureSensor, HELIO_NAME_MAXSIZE); }
    if (windSpeedSensor[0]) { objectOut[SFP(HStr_Key_WindSpeedSensor)] = charsToString(windSpeedSensor, HELIO_NAME_MAXSIZE); }
    if (heatingTrigger.isSet()) {
        JsonObject heatingTriggerObj = objectOut.createNestedObject(SFP(HStr_Key_HeatingTrigger));
        heatingTrigger.toJSONObject(heatingTriggerObj);
    }
    if (stormingTrigger.isSet()) {
        JsonObject stormingTriggerObj = objectOut.createNestedObject(SFP(HStr_Key_StormingTrigger));
        stormingTrigger.toJSONObject(stormingTriggerObj);
    }
}

void HelioTrackingPanelData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioPanelData::fromJSONObject(objectIn);

    temperatureUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_TemperatureUnits)]);
    distanceUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_DistanceUnits)]);
    lastAlignedTime = objectIn[SFP(HStr_Key_LastAlignedTime)] | lastAlignedTime;
    lastCleanedTime = objectIn[SFP(HStr_Key_LastCleanedTime)] | lastCleanedTime;
    JsonVariantConst locationOffsetVar = objectIn[SFP(HStr_Key_LocationOffset)];
    commaStringToArray(locationOffsetVar, locationOffset, 2);
    JsonVariantConst axisPositionVar = objectIn[SFP(HStr_Key_AxisPosition)];
    commaStringToArray(axisPositionVar, axisPosition, 2);
    const char *axisSensorHorzStr = objectIn[SFP(HStr_Key_AxisSensorHorz)];
    if (axisSensorHorzStr && axisSensorHorzStr[0]) { strncpy(axisSensorHorz, axisSensorHorzStr, HELIO_NAME_MAXSIZE); }
    const char *axisSensorVertStr = objectIn[SFP(HStr_Key_AxisSensorVert)];
    if (axisSensorVertStr && axisSensorVertStr[0]) { strncpy(axisSensorVert, axisSensorVertStr, HELIO_NAME_MAXSIZE); }
    const char *powerUsageSensorStr = objectIn[SFP(HStr_Key_PowerUsageSensor)];
    if (powerUsageSensorStr && powerUsageSensorStr[0]) { strncpy(powerUsageSensor, powerUsageSensorStr, HELIO_NAME_MAXSIZE); }
    const char *temperatureSensorStr = objectIn[SFP(HStr_Key_TemperatureSensor)];
    if (temperatureSensorStr && temperatureSensorStr[0]) { strncpy(temperatureSensor, temperatureSensorStr, HELIO_NAME_MAXSIZE); }
    const char *windSpeedSensorStr = objectIn[SFP(HStr_Key_WindSpeedSensor)];
    if (windSpeedSensorStr && windSpeedSensorStr[0]) { strncpy(windSpeedSensor, windSpeedSensorStr, HELIO_NAME_MAXSIZE); }
    JsonObjectConst heatingTriggerObj = objectIn[SFP(HStr_Key_HeatingTrigger)];
    if (!heatingTriggerObj.isNull()) { heatingTrigger.fromJSONObject(heatingTriggerObj); }
    JsonObjectConst stormingTriggerObj = objectIn[SFP(HStr_Key_StormingTrigger)];
    if (!stormingTriggerObj.isNull()) { stormingTrigger.fromJSONObject(stormingTriggerObj); }
}

HelioReflectingPanelData::HelioReflectingPanelData()
    : HelioTrackingPanelData(), reflectPosition{0}
{
    _size = sizeof(*this);
}

void HelioReflectingPanelData::toJSONObject(JsonObject &objectOut) const
{
    HelioTrackingPanelData::toJSONObject(objectOut);

    if (!isFPEqual(reflectPosition[0],0) && !isFPEqual(reflectPosition[1],0)) { objectOut[SFP(HStr_Key_ReflectPosition)] = commaStringFromArray(reflectPosition, 2); }
}

void HelioReflectingPanelData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioTrackingPanelData::fromJSONObject(objectIn);

    JsonVariantConst reflectPositionVar = objectIn[SFP(HStr_Key_ReflectPosition)];
    commaStringToArray(reflectPositionVar, reflectPosition, 2);    
}

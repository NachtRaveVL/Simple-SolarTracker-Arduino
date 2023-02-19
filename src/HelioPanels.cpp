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
            case (int8_t)HelioPanel::Balancing:
                return new HelioBalancingPanel((const HelioBalancingPanelData *)dataIn);
            case (int8_t)HelioPanel::Tracking:
                return new HelioTrackingPanel((const HelioTrackingPanelData *)dataIn);
            case (int8_t)HelioPanel::Reflecting:
                return new HelioReflectingPanel((const HelioReflectingPanelData *)dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioPanel::HelioPanel(Helio_PanelType panelType, hposi_t panelIndex, float alignedTolerance, int classTypeIn)
    : HelioObject(HelioIdentity(panelType, panelIndex)), classType((typeof(classType))classTypeIn),
      HelioPowerUnitsInterfaceStorage(defaultPowerUnits()),
      _panelState(Helio_PanelState_Undefined), _alignedTolerance(alignedTolerance),
      _homePosition{0}, _axisOffset{0}, _inDaytimeMode(false),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)}
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
      _inDaytimeMode(false),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)}
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

    if (getScheduler()) {
        setInDaytimeMode(getScheduler()->inDaytimeMode());
    }

    if (_inDaytimeMode) {
        handleState(isAligned(true) ? Helio_PanelState_AlignedToSun : Helio_PanelState_TravelingToSun);
    } else {
        handleState(isAligned(true) ? Helio_PanelState_AlignedToHome : Helio_PanelState_TravelingToHome);
    }
}

bool HelioPanel::canActivate(HelioActuator *actuator)
{
    if (actuator->isMovementType()) {
        auto brakes = linksFilterActuatorsByPanelAndType(getLinkages(), this, Helio_ActuatorType_PanelBrake);
        for (auto brakeIter = brakes.begin(); brakeIter != brakes.end(); ++brakeIter) {
            HelioActuator *brake = (HelioActuator *)*brakeIter;
            if (brake->isEnabled()) { return false; }
        }
    } else if (actuator->getActuatorType() == Helio_ActuatorType_PanelBrake) {
        return (!_axisDriver[0] || !_axisDriver[0]->isEnabled() || _axisDriver[0]->isAligned(true)) &&
               (!_axisDriver[1] || !_axisDriver[1]->isEnabled() || _axisDriver[1]->isAligned(true));
    }
    return true;
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
    if (_powerProd.getId()) {
        strncpy(((HelioPanelData *)dataOut)->powerProdSensor, _powerProd.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
}


HelioBalancingPanel::HelioBalancingPanel(Helio_PanelType panelType, hposi_t panelIndex, float ldrTolerance, int classType)
    : HelioPanel(panelType, panelIndex, ldrTolerance, classType),
      _ldrSensors{HelioSensorAttachment(this),HelioSensorAttachment(this),HelioSensorAttachment(this),HelioSensorAttachment(this)}
{ ; }

HelioBalancingPanel::HelioBalancingPanel(const HelioBalancingPanelData *dataIn)
    : HelioPanel(dataIn),
      _ldrSensors{HelioSensorAttachment(this),HelioSensorAttachment(this),HelioSensorAttachment(this),HelioSensorAttachment(this)}
{
    _ldrSensors[0].setObject(dataIn->ldrSensor1);
    _ldrSensors[1].setObject(dataIn->ldrSensor2);
    _ldrSensors[2].setObject(dataIn->ldrSensor3);
    _ldrSensors[3].setObject(dataIn->ldrSensor4);
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
        if (drivesHorizontalAxis()) {
            if (_inDaytimeMode && _ldrSensors[0].resolve() && _ldrSensors[1].resolve()) {
                auto intensityHorzMin = _ldrSensors[0].getMeasurement(true).asUnits(Helio_UnitsType_Raw_1);
                auto intensityHorzMax = _ldrSensors[1].getMeasurement(true).asUnits(Helio_UnitsType_Raw_1);
                aligned = aligned && intensityHorzMin.value > FLT_EPSILON && intensityHorzMax.value > FLT_EPSILON &&
                          fabsf(intensityHorzMax.value - intensityHorzMin.value) <= _alignedTolerance + FLT_EPSILON;
            } else {
                aligned = aligned && _axisDriver[0].resolve() && isFPEqual(_homePosition[0], _axisDriver[0]->getTargetSetpoint()) && _axisDriver[0]->isAligned(true);
            }
        }
        if (drivesVerticalAxis()) {
            if (_inDaytimeMode && _ldrSensors[2].resolve() && _ldrSensors[3].resolve()) {
                auto intensityVertMin = _ldrSensors[2].getMeasurement(true).asUnits(Helio_UnitsType_Raw_1);
                auto intensityVertMax = _ldrSensors[3].getMeasurement(true).asUnits(Helio_UnitsType_Raw_1);
                aligned = aligned && intensityVertMin.value > FLT_EPSILON && intensityVertMax.value > FLT_EPSILON &&
                          fabsf(intensityVertMax.value - intensityVertMin.value) <= _alignedTolerance + FLT_EPSILON;
            } else {
                aligned = aligned && _axisDriver[1].resolve() && isFPEqual(_homePosition[1], _axisDriver[1]->getTargetSetpoint()) && _axisDriver[1]->isAligned(true);
            }
        }
        return aligned;
    }
    return _panelState == (_inDaytimeMode ? Helio_PanelState_AlignedToSun : Helio_PanelState_AlignedToHome);
}

void HelioBalancingPanel::saveToData(HelioData *dataOut)
{
    HelioPanel::saveToData(dataOut);

    if (_ldrSensors[0].getId()) {
        strncpy(((HelioBalancingPanelData *)dataOut)->ldrSensor1, _ldrSensors[0].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_ldrSensors[1].getId()) {
        strncpy(((HelioBalancingPanelData *)dataOut)->ldrSensor2, _ldrSensors[1].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_ldrSensors[2].getId()) {
        strncpy(((HelioBalancingPanelData *)dataOut)->ldrSensor3, _ldrSensors[2].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_ldrSensors[3].getId()) {
        strncpy(((HelioBalancingPanelData *)dataOut)->ldrSensor4, _ldrSensors[3].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
}

void HelioBalancingPanel::handleState(Helio_PanelState panelState)
{
    if (panelState == Helio_PanelState_Undefined) { return; }
    Helio_PanelState hadPanelState = _panelState;
    _panelState = panelState;

    if (_panelState != (_inDaytimeMode ? Helio_PanelState_AlignedToSun : Helio_PanelState_AlignedToHome)) {
        if (drivesHorizontalAxis() && _axisDriver[0].resolve()) {
            auto intensityHorzMin = _ldrSensors[0].getMeasurement().asUnits(Helio_UnitsType_Raw_1);
            auto intensityHorzMax = _ldrSensors[1].getMeasurement().asUnits(Helio_UnitsType_Raw_1);

            if (intensityHorzMin.value > FLT_EPSILON && intensityHorzMax.value > FLT_EPSILON) {
                if (fabsf(intensityHorzMax.value - intensityHorzMin.value) <= _alignedTolerance + FLT_EPSILON) {
                    _axisDriver[0]->setEnabled(false);
                } else {
                    _axisDriver[0]->setEnabled(true);
                    auto setPoint = _axisDriver[0]->getTargetSetpoint();
                    auto rateStep = (HELIO_CONTROL_LOOP_INTERVAL / 1000.0f) * _axisDriver[0]->getTravelRate();
                    if (intensityHorzMin.value > intensityHorzMax.value) {
                        setPoint -= rateStep;
                    } else {
                        setPoint += rateStep;
                    }
                    _axisDriver[0]->setTargetSetpoint(setPoint);
                }
            }
        }
        if (drivesVerticalAxis() && _axisDriver[1].resolve()) {
            auto intensityVertMin = _ldrSensors[2].getMeasurement().asUnits(Helio_UnitsType_Raw_1);
            auto intensityVertMax = _ldrSensors[3].getMeasurement().asUnits(Helio_UnitsType_Raw_1);

            if (intensityVertMin.value > FLT_EPSILON && intensityVertMax.value > FLT_EPSILON) {
                if (fabsf(intensityVertMax.value - intensityVertMin.value) <= _alignedTolerance + FLT_EPSILON) {
                    _axisDriver[1]->setEnabled(false);
                } else {
                    _axisDriver[1]->setEnabled(true);
                    auto setPoint = _axisDriver[1]->getTargetSetpoint();
                    auto rateStep = (HELIO_CONTROL_LOOP_INTERVAL / 1000.0f) * _axisDriver[1]->getTravelRate();
                    if (intensityVertMin.value > intensityVertMax.value) {
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
      _lastChangeTime(0), _sunPosition{0}, _facingPosition{0},
      _powerUsage(this), _axisAngle{HelioSensorAttachment(this),HelioSensorAttachment(this)}
{
    _powerUsage.setMeasurementUnits(getPowerUnits());
}

HelioTrackingPanel::HelioTrackingPanel(const HelioTrackingPanelData *dataIn)
    : HelioPanel(dataIn),
      _lastChangeTime(dataIn->lastChangeTime), _sunPosition{0},
      _facingPosition{dataIn->axisPosition[0], dataIn->axisPosition[1]},
      _powerUsage(this), _axisAngle{HelioSensorAttachment(this),HelioSensorAttachment(this)}
{
    _powerUsage.setMeasurementUnits(getPowerUnits());
    _powerUsage.setObject(dataIn->powerUsageSensor);

    _axisAngle[0].setObject(dataIn->axisSensor1);
    _axisAngle[1].setObject(dataIn->axisSensor2);
}

HelioTrackingPanel::~HelioTrackingPanel()
{ ; }

void HelioTrackingPanel::update()
{
    _axisAngle[0].updateIfNeeded(true);
    _axisAngle[1].updateIfNeeded(true);

    HelioPanel::update();
}

bool HelioTrackingPanel::isAligned(bool poll)
{
    if (poll) {
        if (_inDaytimeMode) { recalcSunPosition(); }
        else { _sunPosition[0] = 0; _sunPosition[1] = 0; }
        recalcFacingPosition();

        bool aligned = true;
        if (aligned && drivesHorizontalAxis()) {
            if (_axisAngle[0].resolve()) {
                auto axisPositionHorz = _axisAngle[0].getMeasurement().asUnits(Helio_UnitsType_Angle_Degrees_360).wrappedBy360();
                aligned = aligned && fabsf(_facingPosition[0] - axisPositionHorz.value) <= _alignedTolerance + FLT_EPSILON;
            }
            aligned = aligned && _axisDriver[0].resolve() && isFPEqual(_facingPosition[0], _axisDriver[0]->getTargetSetpoint()) && _axisDriver[0]->isAligned(true);
        }
        if (aligned && drivesVerticalAxis()) {
            if (_axisAngle[1].resolve()) {
                auto axisPositionVert = _axisAngle[1].getMeasurement().asUnits(Helio_UnitsType_Angle_Degrees_360).wrappedBy360();
                aligned = aligned && fabsf(_facingPosition[1] - axisPositionVert.value) <= _alignedTolerance + FLT_EPSILON;
            }
            aligned = aligned && _axisDriver[1].resolve() && isFPEqual(_facingPosition[1], _axisDriver[1]->getTargetSetpoint()) && _axisDriver[1]->isAligned(true);
        }
        return aligned;
    }
    return _panelState == (_inDaytimeMode ? Helio_PanelState_AlignedToSun : Helio_PanelState_AlignedToHome);
}

void HelioTrackingPanel::recalcSunPosition()
{
    time_t time = unixNow();
    JulianDay julianTime(time);

    if (isHorizontalCoords()) {
        Location loc = getController() ? getController()->getSystemLocation() : Location();
        if (loc.hasPosition()) {
            calcHorizontalCoordinates(julianTime, loc.latitude, loc.longitude, _sunPosition[0], _sunPosition[1]);
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
    if (_inDaytimeMode) {
        _facingPosition[0] = wrapBy360(_sunPosition[0] + _axisOffset[0]);
        _facingPosition[1] = wrapBy180Neg180(_sunPosition[1] + _axisOffset[1]);
    } else {
        _facingPosition[0] = wrapBy360(_homePosition[0] + _axisOffset[0]);
        _facingPosition[1] = wrapBy180Neg180(_homePosition[1] + _axisOffset[1]);
    }

    if (_axisDriver[0].resolve()) { _axisDriver[0]->setTargetSetpoint(_facingPosition[0]); }
    if (_axisDriver[1].resolve()) { _axisDriver[1]->setTargetSetpoint(_facingPosition[1]); }
}

void HelioTrackingPanel::setPowerUnits(Helio_UnitsType powerUnits)
{
    if (_powerUnits != powerUnits) {
        HelioPanel::setPowerUnits(powerUnits);

        _powerUsage.setMeasurementUnits(_powerUnits);
    }
}

HelioSensorAttachment &HelioTrackingPanel::getPowerUsageSensorAttachment()
{
    return _powerUsage;
}

void HelioTrackingPanel::saveToData(HelioData *dataOut)
{
    HelioPanel::saveToData(dataOut);

    ((HelioTrackingPanelData *)dataOut)->axisPosition[0] = wrapBy360(_axisAngle[0].getMeasurement(true).asUnits(Helio_UnitsType_Angle_Degrees_360).value);
    ((HelioTrackingPanelData *)dataOut)->axisPosition[1] = wrapBy180Neg180(_axisAngle[1].getMeasurement(true).asUnits(Helio_UnitsType_Angle_Degrees_360).value);
    ((HelioTrackingPanelData *)dataOut)->lastChangeTime = _lastChangeTime;
    if (_axisAngle[0].getId()) {
        strncpy(((HelioTrackingPanelData *)dataOut)->axisSensor1, _axisAngle[0].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_axisAngle[1].getId()) {
        strncpy(((HelioTrackingPanelData *)dataOut)->axisSensor2, _axisAngle[1].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_powerUsage.getId()) {
        strncpy(((HelioTrackingPanelData *)dataOut)->powerUsageSensor, _powerUsage.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
}

void HelioTrackingPanel::handleState(Helio_PanelState panelState)
{
    if (panelState == Helio_PanelState_Undefined) { return; }
    Helio_PanelState hadPanelState = _panelState;
    _panelState = panelState;

    if (_panelState != (_inDaytimeMode ? Helio_PanelState_AlignedToSun : Helio_PanelState_AlignedToHome)) {
        if (_axisDriver[0].resolve()) { _axisDriver[0]->setEnabled(true); }
        if (_axisDriver[1].resolve()) { _axisDriver[1]->setEnabled(true); }
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
    if (_inDaytimeMode) {
        _facingPosition[0] = wrapBy360((_sunPosition[0] + ((wrapBy180Neg180(_reflectPosition[0]) - wrapBy180Neg180(_sunPosition[0])) * 0.5f)) + _axisOffset[0]);
        _facingPosition[0] = wrapBy180Neg180((_sunPosition[1] + ((wrapBy180Neg180(_reflectPosition[1]) - wrapBy180Neg180(_sunPosition[1])) * 0.5f)) + _axisOffset[1]);
    } else {
        _facingPosition[0] = wrapBy360(_homePosition[0] + _axisOffset[0]);
        _facingPosition[1] = wrapBy180Neg180(_homePosition[1] + _axisOffset[1]);
    }

    if (_axisDriver[0].resolve()) { _axisDriver[0]->setTargetSetpoint(_facingPosition[0]); }
    if (_axisDriver[1].resolve()) { _axisDriver[1]->setTargetSetpoint(_facingPosition[1]); }
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
    if (!isFPEqual(homePosition[0],0) && !isFPEqual(homePosition[1],0)) { objectOut[SFP(HStr_Key_HomePosition)] = commaStringFromArray(homePosition, 2); }
    if (!isFPEqual(axisOffset[0],0) && !isFPEqual(axisOffset[1],0)) { objectOut[SFP(HStr_Key_AxisOffset)] = commaStringFromArray(axisOffset, 2); }
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
    : HelioPanelData(), ldrSensor1{0}, ldrSensor2{0}, ldrSensor3{0}, ldrSensor4{0}
{
    _size = sizeof(*this);
}

void HelioBalancingPanelData::toJSONObject(JsonObject &objectOut) const
{
    HelioPanelData::toJSONObject(objectOut);

    if (alignedTolerance != FLT_UNDEF && !isFPEqual(alignedTolerance, HELIO_PANEL_ALIGN_LDRTOL)) { objectOut[SFP(HStr_Key_AlignedTolerance)] = alignedTolerance; }
    if (ldrSensor1[0]) { objectOut[SFP(HStr_Key_LDRSensor1)] = charsToString(ldrSensor1, HELIO_NAME_MAXSIZE); }
    if (ldrSensor2[0]) { objectOut[SFP(HStr_Key_LDRSensor2)] = charsToString(ldrSensor2, HELIO_NAME_MAXSIZE); }
    if (ldrSensor3[0]) { objectOut[SFP(HStr_Key_LDRSensor3)] = charsToString(ldrSensor3, HELIO_NAME_MAXSIZE); }
    if (ldrSensor4[0]) { objectOut[SFP(HStr_Key_LDRSensor4)] = charsToString(ldrSensor4, HELIO_NAME_MAXSIZE); }
}

void HelioBalancingPanelData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioPanelData::fromJSONObject(objectIn);

    const char *ldrSensor1Str = objectIn[SFP(HStr_Key_LDRSensor1)];
    if (ldrSensor1Str && ldrSensor1Str[0]) { strncpy(ldrSensor1, ldrSensor1Str, HELIO_NAME_MAXSIZE); }
    const char *ldrSensor2Str = objectIn[SFP(HStr_Key_LDRSensor2)];
    if (ldrSensor2Str && ldrSensor2Str[0]) { strncpy(ldrSensor2, ldrSensor2Str, HELIO_NAME_MAXSIZE); }
    const char *ldrSensor3Str = objectIn[SFP(HStr_Key_LDRSensor3)];
    if (ldrSensor3Str && ldrSensor3Str[0]) { strncpy(ldrSensor3, ldrSensor3Str, HELIO_NAME_MAXSIZE); }
    const char *ldrSensor4Str = objectIn[SFP(HStr_Key_LDRSensor4)];
    if (ldrSensor4Str && ldrSensor4Str[0]) { strncpy(ldrSensor4, ldrSensor4Str, HELIO_NAME_MAXSIZE); }
}

HelioTrackingPanelData::HelioTrackingPanelData()
    : HelioPanelData(), axisPosition{0}, lastChangeTime(0), axisSensor1{0}, axisSensor2{0}, powerUsageSensor{0}
{
    _size = sizeof(*this);
}

void HelioTrackingPanelData::toJSONObject(JsonObject &objectOut) const
{
    HelioPanelData::toJSONObject(objectOut);

    if (alignedTolerance != FLT_UNDEF && !isFPEqual(alignedTolerance, HELIO_PANEL_ALIGN_DEGTOL)) { objectOut[SFP(HStr_Key_AlignedTolerance)] = alignedTolerance; }
    if (!isFPEqual(axisPosition[0],0) && !isFPEqual(axisPosition[1],0)) { objectOut[SFP(HStr_Key_AxisPosition)] = commaStringFromArray(axisPosition, 2); }
    if (lastChangeTime) { objectOut[SFP(HStr_Key_LastChangeDate)] = lastChangeTime; }
    if (axisSensor1[0]) { objectOut[SFP(HStr_Key_AxisSensor1)] = charsToString(axisSensor1, HELIO_NAME_MAXSIZE); }
    if (axisSensor2[0]) { objectOut[SFP(HStr_Key_AxisSensor2)] = charsToString(axisSensor2, HELIO_NAME_MAXSIZE); }
    if (powerUsageSensor[0]) { objectOut[SFP(HStr_Key_PowerUsageSensor)] = charsToString(powerUsageSensor, HELIO_NAME_MAXSIZE); }
}

void HelioTrackingPanelData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioPanelData::fromJSONObject(objectIn);

    JsonVariantConst axisPositionVar = objectIn[SFP(HStr_Key_AxisPosition)];
    commaStringToArray(axisPositionVar, axisPosition, 2);
    lastChangeTime = objectIn[SFP(HStr_Key_LastChangeDate)] | lastChangeTime;
    const char *axisSensor1Str = objectIn[SFP(HStr_Key_AxisSensor1)];
    if (axisSensor1Str && axisSensor1Str[0]) { strncpy(axisSensor1, axisSensor1Str, HELIO_NAME_MAXSIZE); }
    const char *axisSensor2Str = objectIn[SFP(HStr_Key_AxisSensor2)];
    if (axisSensor2Str && axisSensor2Str[0]) { strncpy(axisSensor2, axisSensor2Str, HELIO_NAME_MAXSIZE); }
    const char *powerUsageSensorStr = objectIn[SFP(HStr_Key_PowerUsageSensor)];
    if (powerUsageSensorStr && powerUsageSensorStr[0]) { strncpy(powerUsageSensor, powerUsageSensorStr, HELIO_NAME_MAXSIZE); }
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

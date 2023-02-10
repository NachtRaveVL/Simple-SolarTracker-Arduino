/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Panels
*/

#include "Helioduino.h"

HelioPanel *newPanelObjectFromData(const HelioPanelData *dataIn)
{
    if (dataIn && dataIn->id.object.idType == -1) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && dataIn->isObjectData(), SFP(HStr_Err_InvalidParameter));

    if (dataIn && dataIn->isObjectData()) {
        switch (dataIn->id.object.classType) {
            case (int8_t)HelioPanel::Tracking:
                return new HelioTrackingPanel((const HelioTrackingPanelData *)dataIn);
            case (int8_t)HelioPanel::Reflecting:
                return new HelioReflectingPanel((const HelioReflectingPanelData *)dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioPanel::HelioPanel(Helio_PanelType panelType, hposi_t panelIndex, int classTypeIn)
    : HelioObject(HelioIdentity(panelType, panelIndex)), classType((typeof(classType))classTypeIn),
      _powerUnits(defaultPowerUnits()), _panelState(Helio_PanelState_Undefined),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)},
      _axisAngle{HelioSensorAttachment(this),HelioSensorAttachment(this)},
      _sunPosition(nullptr), _ldrSensors(nullptr)
{
    if (getHelioInstance() && getHelioInstance()->getSystemMode() == Helio_SystemMode_BrightnessBalancing) {
        allocateLDRSensors();
        HELIO_HARD_ASSERT(_ldrSensors, SFP(HStr_Err_AllocationFailure));
    } else {
        recalcSunPosition();
    }
    allocateLinkages(HELIO_PANEL_LINKS_BASESIZE);
    _powerProd.setMeasurementUnits(getPowerUnits());
}

HelioPanel::HelioPanel(const HelioPanelData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))(dataIn->id.object.classType)),
      _powerUnits(definedUnitsElse(dataIn->powerUnits, defaultPowerUnits())), _panelState(Helio_PanelState_Undefined),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)},
      _axisAngle{HelioSensorAttachment(this),HelioSensorAttachment(this)},
      _sunPosition(nullptr), _ldrSensors(nullptr)
{
    if (getHelioInstance() && getHelioInstance()->getSystemMode() == Helio_SystemMode_BrightnessBalancing) {
        allocateLDRSensors();
        HELIO_HARD_ASSERT(_ldrSensors, SFP(HStr_Err_AllocationFailure));
        _ldrSensors[0].setObject(dataIn->ldrSensor1);
        _ldrSensors[1].setObject(dataIn->ldrSensor2);
        if (isMultiAxis()) {
            _ldrSensors[2].setObject(dataIn->ldrSensor3);
            _ldrSensors[3].setObject(dataIn->ldrSensor4);
        }
    } else {
        recalcSunPosition();
    }
    allocateLinkages(HELIO_PANEL_LINKS_BASESIZE);
    _powerProd.setMeasurementUnits(getPowerUnits());
    _powerProd.setObject(dataIn->powerSensor);
    _axisAngle[0].setObject(dataIn->angleSensor1);
    if (isMultiAxis()) {
        _axisAngle[1].setObject(dataIn->angleSensor2);
    }
}

HelioPanel::~HelioPanel()
{
    if (_sunPosition) { delete[] _sunPosition; _sunPosition = nullptr; }
    if (_ldrSensors) { delete[] _ldrSensors; _ldrSensors = nullptr; }
}

void HelioPanel::update()
{
    HelioObject::update();

    _powerProd.updateIfNeeded(true);

    if (isSingleAxis()) {
        _axisAngle[0].updateIfNeeded(true);
        if (isLDRMode()) {
            _ldrSensors[0].updateIfNeeded(true);
            _ldrSensors[1].updateIfNeeded(true);
        }
        _axisDriver[0].updateIfNeeded(true);
    } else {
        _axisAngle[0].updateIfNeeded(true);
        _axisAngle[1].updateIfNeeded(true);
        if (isLDRMode()) {
            _ldrSensors[0].updateIfNeeded(true);
            _ldrSensors[1].updateIfNeeded(true);
            _ldrSensors[2].updateIfNeeded(true);
            _ldrSensors[3].updateIfNeeded(true);
        }
        _axisDriver[0].updateIfNeeded(true);
        _axisDriver[1].updateIfNeeded(true);
    }

    if (getSchedulerInstance()) {
        _inDaytimeMode = getSchedulerInstance()->inDaytimeMode();
    }

    if (_inDaytimeMode) {
        handleState(isAligned() ? Helio_PanelState_AlignedToSun : Helio_PanelState_TravelingToSun);
    } else {
        handleState(isAligned() ? Helio_PanelState_AlignedToHome : Helio_PanelState_TravelingToHome);
    }
}

bool HelioPanel::canActivate(HelioActuator *actuator)
{
    // todo
    return actuator && actuator->getParentPanel().get() == this;
}

bool HelioPanel::isAligned(bool poll)
{
    if (isSingleAxis()) {
        return fabsf(getAxisAngle(0).getMeasurementValue(poll) - (_inDaytimeMode ? (_sunPosition ? (float)_sunPosition[0] : FLT_UNDEF) : _homePosition[0])) <= HELIO_PANEL_ALIGN_DEGTOL;
    } else {
        return fabsf(getAxisAngle(0).getMeasurementValue(poll) - (_inDaytimeMode ? (_sunPosition ? (float)_sunPosition[0] : FLT_UNDEF) : _homePosition[0])) <= HELIO_PANEL_ALIGN_DEGTOL &&
               fabsf(getAxisAngle(1).getMeasurementValue(poll) - (_inDaytimeMode ? (_sunPosition ? (float)_sunPosition[1] : FLT_UNDEF) : _homePosition[1])) <= HELIO_PANEL_ALIGN_DEGTOL;
    }
    return true;
}

void HelioPanel::recalcSunPosition()
{
    time_t time = unixNow();
    JulianDay julianTime(time);

    allocateSPCStorage();
    HELIO_HARD_ASSERT(_sunPosition, SFP(HStr_Err_AllocationFailure));

    if (isHorizontalCoords()) {
        Location loc = getHelioInstance() ? getHelioInstance()->getSystemLocation() : Location();
        if (loc.hasPosition()) {
            calcHorizontalCoordinates(julianTime, loc.latitude, loc.longitude, _sunPosition[0], _sunPosition[1]);
        }
    } else {
        double radius;
        calcEquatorialCoordinates(julianTime, _sunPosition[0], _sunPosition[1], radius);
    }
}

void HelioPanel::setPowerUnits(Helio_UnitsType powerUnits)
{
    if (_powerUnits != powerUnits) {
        _powerUnits = powerUnits;

        _powerProd.setMeasurementUnits(powerUnits);
    }
}

Helio_UnitsType HelioPanel::getPowerUnits() const
{
    return definedUnitsElse(_powerUnits, defaultPowerUnits());
}

HelioSensorAttachment &HelioPanel::getPowerProduction(bool poll)
{
    _powerProd.updateIfNeeded(poll);
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
    ((HelioPanelData *)dataOut)->powerUnits = _powerUnits;
    if (_powerProd.getId()) {
        strncpy(((HelioPanelData *)dataOut)->powerSensor, _powerProd.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    ((HelioPanelData *)dataOut)->homePosition[0] = _homePosition[0];
    ((HelioPanelData *)dataOut)->homePosition[1] = _homePosition[1];
    if (_axisAngle[0].getId()) {
        strncpy(((HelioPanelData *)dataOut)->angleSensor1, _axisAngle[0].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (isMultiAxis() && _axisAngle[1].getId()) {
        strncpy(((HelioPanelData *)dataOut)->angleSensor2, _axisAngle[1].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (isLDRMode()) {
        if (_ldrSensors[0].getId()) {
            strncpy(((HelioPanelData *)dataOut)->ldrSensor1, _ldrSensors[0].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
        }
        if (_ldrSensors[1].getId()) {
            strncpy(((HelioPanelData *)dataOut)->ldrSensor2, _ldrSensors[1].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
        }
        if (isMultiAxis()) {
            if (_ldrSensors[2].getId()) {
                strncpy(((HelioPanelData *)dataOut)->ldrSensor3, _ldrSensors[2].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
            }
            if (_ldrSensors[3].getId()) {
                strncpy(((HelioPanelData *)dataOut)->ldrSensor4, _ldrSensors[3].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
            }
        }
    }
}


HelioPanelData::HelioPanelData()
    : HelioObjectData()
{
    _size = sizeof(*this);
}

void HelioPanelData::toJSONObject(JsonObject &objectOut) const
{
    HelioObjectData::toJSONObject(objectOut);
}

void HelioPanelData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioObjectData::fromJSONObject(objectIn);
}

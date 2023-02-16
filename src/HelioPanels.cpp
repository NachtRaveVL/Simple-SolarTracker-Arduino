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


HelioPanel::HelioPanel(Helio_PanelType panelType, hposi_t panelIndex, int classTypeIn)
    : HelioObject(HelioIdentity(panelType, panelIndex)), classType((typeof(classType))classTypeIn),
      HelioPowerUnitsInterface(defaultPowerUnits()),
      _panelState(Helio_PanelState_Undefined),
      _homePosition{0}, _facingPosition{0}, _inDaytimeMode(false),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)}
{
    allocateLinkages(HELIO_PANEL_LINKS_BASESIZE);
    _powerProd.setMeasureUnits(getPowerUnits());
}

HelioPanel::HelioPanel(const HelioPanelData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))(dataIn->id.object.classType)),
      HelioPowerUnitsInterface(definedUnitsElse(dataIn->powerUnits, defaultPowerUnits())),
      _panelState(Helio_PanelState_Undefined),
      _homePosition{dataIn->homePosition[0],dataIn->homePosition[1]}, _facingPosition{0}, _inDaytimeMode(false),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)}
{
    allocateLinkages(HELIO_PANEL_LINKS_BASESIZE);
    _powerProd.setMeasureUnits(getPowerUnits());
    _powerProd.setObject(dataIn->powerSensor);
}

HelioPanel::~HelioPanel()
{ ; }

void HelioPanel::update()
{
    HelioObject::update();

    _powerProd.updateIfNeeded(true);

    if (isSingleAxis()) {
        _axisDriver[0].updateIfNeeded(true);
    } else {
        _axisDriver[0].updateIfNeeded(true);
        _axisDriver[1].updateIfNeeded(true);
    }

    if (getScheduler()) {
        _inDaytimeMode = getScheduler()->inDaytimeMode();
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

void HelioPanel::setPowerUnits(Helio_UnitsType powerUnits)
{
    if (_powerUnits != powerUnits) {
        _powerUnits = powerUnits;

        _powerProd.setMeasureUnits(powerUnits);
    }
}

HelioSensorAttachment &HelioPanel::getPowerProduction()
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
    ((HelioPanelData *)dataOut)->powerUnits = _powerUnits;
    if (_powerProd.getId()) {
        strncpy(((HelioPanelData *)dataOut)->powerSensor, _powerProd.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    ((HelioPanelData *)dataOut)->homePosition[0] = _homePosition[0];
    ((HelioPanelData *)dataOut)->homePosition[1] = _homePosition[1];

    // if (_axisAngle[0].getId()) {
    //     strncpy(((HelioPanelData *)dataOut)->angleSensor1, _axisAngle[0].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    // }
    // if (isMultiAxis() && _axisAngle[1].getId()) {
    //     strncpy(((HelioPanelData *)dataOut)->angleSensor2, _axisAngle[1].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    // }
    // if (isLDRMode()) {
    //     if (_ldrSensors[0].getId()) {
    //         strncpy(((HelioPanelData *)dataOut)->ldrSensor1, _ldrSensors[0].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    //     }
    //     if (_ldrSensors[1].getId()) {
    //         strncpy(((HelioPanelData *)dataOut)->ldrSensor2, _ldrSensors[1].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    //     }
    //     if (isMultiAxis()) {
    //         if (_ldrSensors[2].getId()) {
    //             strncpy(((HelioPanelData *)dataOut)->ldrSensor3, _ldrSensors[2].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    //         }
    //         if (_ldrSensors[3].getId()) {
    //             strncpy(((HelioPanelData *)dataOut)->ldrSensor4, _ldrSensors[3].getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    //         }
    //     }
    // }
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
    } else {
        double radius;
        calcEquatorialCoordinates(julianTime, _sunPosition[0], _sunPosition[1], radius);
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

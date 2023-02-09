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
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)}
{
    _powerProd.setMeasurementUnits(HelioPanel::getPowerUnits());
}

HelioPanel::HelioPanel(const HelioPanelData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))(dataIn->id.object.classType)),
      _powerUnits(definedUnitsElse(dataIn->powerUnits, defaultPowerUnits())), _panelState(Helio_PanelState_Undefined),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)}
{
    _powerProd.setMeasurementUnits(HelioPanel::getPowerUnits());
    _powerProd.setObject(dataIn->powerSensor);
}

HelioPanel::~HelioPanel()
{ ; }

void HelioPanel::update()
{
    HelioObject::update();

    _powerProd.updateIfNeeded(true);

    _axisAngle[0].updateIfNeeded(true);
    if (getAxisCount() > 1) {
        _axisAngle[1].updateIfNeeded(true);
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

bool HelioPanel::isAligned()
{
    if (getAxisCount() == 1) {
        return fabsf(getAxisAngle(0).getMeasurementValue() - (_inDaytimeMode ? _sunPosition[0] : _homePosition[0])) <= HELIO_PANEL_ALIGN_DEGTOL;
    } else {
        return fabsf(getAxisAngle(0).getMeasurementValue() - (_inDaytimeMode ? _sunPosition[0] : _homePosition[0])) <= HELIO_PANEL_ALIGN_DEGTOL &&
               fabsf(getAxisAngle(1).getMeasurementValue() - (_inDaytimeMode ? _sunPosition[1] : _homePosition[1])) <= HELIO_PANEL_ALIGN_DEGTOL;
    }
}

void HelioPanel::recalcSunPosition()
{
    time_t time = unixNow();
    JulianDay julianTime(time);

    if (!isEquatorialCoords()) {
        Location loc = getHelioInstance() ? getHelioInstance()->getSystemLocation() : Location();
        if (loc.hasPosition()) {
            calcHorizontalCoordinates(julianTime, loc.latitude, loc.longitude, _sunPosition[0], _sunPosition[1]);
        }
    } else {
        double radius;
        calcEquatorialCoordinates(julianTime, _sunPosition[0], _sunPosition[1], radius);
    }
}


HelioData *HelioPanel::allocateData() const
{
    return _allocateDataForObjType((int8_t)_id.type, (int8_t)classType);
}

void HelioPanel::saveToData(HelioData *dataOut)
{
    HelioObject::saveToData(dataOut);

    dataOut->id.object.classType = (int8_t)classType;
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

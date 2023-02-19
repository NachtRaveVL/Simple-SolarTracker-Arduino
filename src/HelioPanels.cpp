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
      HelioPowerUnitsInterfaceStorage(defaultPowerUnits()),
      _panelState(Helio_PanelState_Undefined),
      _homePosition{0}, _facingPosition{0}, _inDaytimeMode(false),
      _powerProd(this), _axisDriver{HelioDriverAttachment(this),HelioDriverAttachment(this)}
{
    allocateLinkages(HELIO_PANEL_LINKS_BASESIZE);
    _powerProd.setMeasurementUnits(getPowerUnits());
}

HelioPanel::HelioPanel(const HelioPanelData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))(dataIn->id.object.classType)),
      HelioPowerUnitsInterfaceStorage(definedUnitsElse(dataIn->powerUnits, defaultPowerUnits())),
      _panelState(Helio_PanelState_Undefined),
      _homePosition{dataIn->homePosition[0],dataIn->homePosition[1]}, _facingPosition{0}, _inDaytimeMode(false),
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
        _inDaytimeMode = getScheduler()->inDaytimeMode();
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
    ((HelioPanelData *)dataOut)->homePosition[0] = _homePosition[0];
    ((HelioPanelData *)dataOut)->homePosition[1] = _homePosition[1];
    if (_powerProd.getId()) {
        strncpy(((HelioPanelData *)dataOut)->powerProdSensor, _powerProd.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
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

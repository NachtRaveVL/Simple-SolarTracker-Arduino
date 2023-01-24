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
            // todo
            default: break;
        }
    }

    return nullptr;
}


HelioPanel::HelioPanel(Helio_PanelType panelType, Helio_PositionIndex panelIndex, int classTypeIn)
    : HelioObject(HelioIdentity(panelType, panelIndex)), classType((typeof(classType))classTypeIn)
{ ; }

HelioPanel::HelioPanel(const HelioPanelData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))(dataIn->id.object.classType))
{ ; }

void HelioPanel::update()
{
    HelioObject::update();
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

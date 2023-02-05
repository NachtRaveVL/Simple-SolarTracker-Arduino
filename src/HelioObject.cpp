/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Object
*/

#include "Helioduino.h"

HelioObject *newObjectFromData(const HelioData *dataIn)
{
    if (dataIn && dataIn->id.object.idType == -1) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && dataIn->isObjectData(), SFP(HStr_Err_InvalidParameter));

    if (dataIn && dataIn->isObjectData()) {
        switch (dataIn->id.object.idType) {
            case (int8_t)HelioIdentity::Actuator:
                return newActuatorObjectFromData((HelioActuatorData *)dataIn);
            case (int8_t)HelioIdentity::Sensor:
                return newSensorObjectFromData((HelioSensorData *)dataIn);
            case (int8_t)HelioIdentity::Panel:
                return newPanelObjectFromData((HelioPanelData *)dataIn);
            case (int8_t)HelioIdentity::Rail:
                return newRailObjectFromData((HelioRailData *)dataIn);
            default: // Unable
                return nullptr;
        }
    }

    return nullptr;
}


HelioIdentity::HelioIdentity()
    : type(Unknown), objTypeAs{.actuatorType=(Helio_ActuatorType)-1}, posIndex(-1), keyString(), key((Helio_KeyType)-1)
{ ; }

HelioIdentity::HelioIdentity(Helio_KeyType key)
    : type(Unknown), objTypeAs{.actuatorType=(Helio_ActuatorType)-1}, posIndex(-1), keyString(), key(key)
{ ; }

HelioIdentity::HelioIdentity(const char *idKeyStr)
    : type(Unknown), objTypeAs{.actuatorType=(Helio_ActuatorType)-1}, posIndex(-1), keyString(idKeyStr), key(stringHash(idKeyStr))
{
    // TODO: Advanced string detokenization (may not be needed tho)
}

HelioIdentity::HelioIdentity(String idKey)
    : type(Unknown), objTypeAs{.actuatorType=(Helio_ActuatorType)-1}, posIndex(-1), keyString(idKey), key(stringHash(idKey.c_str()))
{ ; }

HelioIdentity::HelioIdentity(const HelioIdentity &id, Helio_PositionIndex positionIndex)
    : type(id.type), objTypeAs{.actuatorType=id.objTypeAs.actuatorType}, posIndex(positionIndex), keyString(), key((Helio_KeyType)-1)
{
    regenKey();
}

HelioIdentity::HelioIdentity(Helio_ActuatorType actuatorTypeIn, Helio_PositionIndex positionIndex)
    : type(Actuator), objTypeAs{.actuatorType=actuatorTypeIn}, posIndex(positionIndex), keyString(), key((Helio_KeyType)-1)
{
    regenKey();
}

HelioIdentity::HelioIdentity(Helio_SensorType sensorTypeIn, Helio_PositionIndex positionIndex)
    : type(Sensor), objTypeAs{.sensorType=sensorTypeIn}, posIndex(positionIndex), keyString(), key((Helio_KeyType)-1)
{
    regenKey();
}

HelioIdentity::HelioIdentity(Helio_PanelType panelTypeIn, Helio_PositionIndex positionIndex)
    : type(Panel), objTypeAs{.panelType=panelTypeIn}, posIndex(positionIndex), keyString(), key((Helio_KeyType)-1)
{
    regenKey();
}

HelioIdentity::HelioIdentity(Helio_RailType railTypeIn, Helio_PositionIndex positionIndex)
    : type(Rail), objTypeAs{.railType=railTypeIn}, posIndex(positionIndex), keyString(), key((Helio_KeyType)-1)
{
    regenKey();
}

HelioIdentity::HelioIdentity(const HelioData *dataIn)
    : type((typeof(type))(dataIn->id.object.idType)),
      objTypeAs{.actuatorType=(Helio_ActuatorType)(dataIn->id.object.objType)},
      posIndex(dataIn->id.object.posIndex),
      keyString(), key((Helio_KeyType)-1)
{
    regenKey();
}

Helio_KeyType HelioIdentity::regenKey()
{
    switch (type) {
        case Actuator:
            keyString = actuatorTypeToString(objTypeAs.actuatorType, true);
            break;
        case Sensor:
            keyString = sensorTypeToString(objTypeAs.sensorType, true);
            break;
        case Panel:
            keyString = panelTypeToString(objTypeAs.panelType, true);
            break;
        case Rail:
            keyString = railTypeToString(objTypeAs.railType, true);
            break;
        default: // Unable
            return key;
    }
    keyString.concat(' ');
    keyString.concat('#');
    keyString.concat(positionIndexToString(posIndex, true));
    key = stringHash(keyString);
    return key;
}


HelioObject::HelioObject(HelioIdentity id)
    : _id(id), _linksSize(0), _links(nullptr)
{ ; }

HelioObject::HelioObject(const HelioData *data)
    : _id(data), _linksSize(0), _links(nullptr)
{ ; }

HelioObject::~HelioObject()
{
    if (_links) { delete [] _links; _links = nullptr; }
}

void HelioObject::update()
{ ; }

void HelioObject::handleLowMemory()
{
    if (_links && !_links[_linksSize >> 1].first) { allocateLinkages(_linksSize >> 1); } // shrink /2 if too big
}

HelioData *HelioObject::newSaveData()
{
    auto data = allocateData();
    HELIO_SOFT_ASSERT(data, SFP(HStr_Err_AllocationFailure));
    if (data) { saveToData(data); }
    return data;
}

void HelioObject::allocateLinkages(size_t size)
{
    if (_linksSize != size) {
        Pair<HelioObject *, int8_t> *newLinks = size ? new Pair<HelioObject *, int8_t>[size] : nullptr;

        if (size) {
            HELIO_HARD_ASSERT(newLinks, SFP(HStr_Err_AllocationFailure));

            int linksIndex = 0;
            if (_links) {
                for (; linksIndex < _linksSize && linksIndex < size; ++linksIndex) {
                    newLinks[linksIndex] = _links[linksIndex];
                }
            }
            for (; linksIndex < size; ++linksIndex) {
                newLinks[linksIndex] = make_pair((HelioObject *)nullptr, (int8_t)0);
            }
        }

        if (_links) { delete [] _links; }
        _links = newLinks;
        _linksSize = size;
    }
}

bool HelioObject::addLinkage(HelioObject *obj)
{
    if (!_links) { allocateLinkages(); }
    if (_links) {
        if (_links[_linksSize-1].first) { allocateLinkages(_linksSize << 1); } // grow *2 if too small
        int linksIndex = 0;
        for (; linksIndex < _linksSize && _links[linksIndex].first; ++linksIndex) {
            if (_links[linksIndex].first == obj) {
                _links[linksIndex].second++;
                return true;
            }
        }
        if (linksIndex < _linksSize) {
            _links[linksIndex] = make_pair(obj, (int8_t)0);
            return true;
        }
    }
    return false;
}

bool HelioObject::removeLinkage(HelioObject *obj)
{
    if (_links) {
        for (int linksIndex = 0; linksIndex < _linksSize && _links[linksIndex].first; ++linksIndex) {
            if (_links[linksIndex].first == obj) {
                for (int linksSubIndex = linksIndex; linksSubIndex < _linksSize - 1; ++linksSubIndex) {
                    _links[linksSubIndex] = _links[linksSubIndex + 1];
                }
                _links[_linksSize - 1] = make_pair((HelioObject *)nullptr, (int8_t)0);
                return true;
            }
        }
    }
    return false;
}

bool HelioObject::hasLinkage(HelioObject *obj) const
{
    if (_links) {
        for (int linksIndex = 0; linksIndex < _linksSize && _links[linksIndex].first; ++linksIndex) {
            if (_links[linksIndex].first == obj) {
                return true;
            }
        }
    }
    return false;
}

HelioIdentity HelioObject::getId() const
{
    return _id;
}

Helio_KeyType HelioObject::getKey() const
{
    return _id.key;
}

String HelioObject::getKeyString() const
{
    return _id.keyString;
}

SharedPtr<HelioObjInterface> HelioObject::getSharedPtr() const
{
    return getHelioInstance() ? static_pointer_cast<HelioObjInterface>(getHelioInstance()->objectById(_id))
                              : SharedPtr<HelioObjInterface>((HelioObjInterface *)this);
}

HelioData *HelioObject::allocateData() const
{
    HELIO_HARD_ASSERT(false, SFP(HStr_Err_UnsupportedOperation));
    return new HelioData();
}

void HelioObject::saveToData(HelioData *dataOut)
{
    dataOut->id.object.idType = (int8_t)_id.type;
    dataOut->id.object.objType = (int8_t)_id.objTypeAs.actuatorType;
    dataOut->id.object.posIndex = (int8_t)_id.posIndex;
    if (_id.keyString.length()) {
        strncpy(((HelioObjectData *)dataOut)->name, _id.keyString.c_str(), HELIO_NAME_MAXSIZE);
    }
}


HelioIdentity HelioSubObject::getId() const
{
    return HelioIdentity(getKey());
}

Helio_KeyType HelioSubObject::getKey() const
{
    return (Helio_KeyType)(intptr_t)this;
}

String HelioSubObject::getKeyString() const
{
    return addressToString((uintptr_t)this);
}

SharedPtr<HelioObjInterface> HelioSubObject::getSharedPtr() const
{
    return SharedPtr<HelioObjInterface>((HelioObjInterface *)this);
}

bool HelioSubObject::addLinkage(HelioObject *obj)
{
    return false;
}

bool HelioSubObject::removeLinkage(HelioObject *obj)
{
    return false;
}

HelioObjectData::HelioObjectData()
    : HelioData(), name{0}
{
    _size = sizeof(*this);
}

void HelioObjectData::toJSONObject(JsonObject &objectOut) const
{
    HelioData::toJSONObject(objectOut);

    if (name[0]) { objectOut[SFP(HStr_Key_Id)] = charsToString(name, HELIO_NAME_MAXSIZE); }
}

void HelioObjectData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioData::fromJSONObject(objectIn);

    const char *nameStr = objectIn[SFP(HStr_Key_Id)];
    if (nameStr && nameStr[0]) { strncpy(name, nameStr, HELIO_NAME_MAXSIZE); }
}

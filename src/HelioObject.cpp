/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Object
*/

#include "Helioduino.h"

HelioObject *newObjectFromData(const HelioData *dataIn)
{
    if (dataIn && isValidType(dataIn->id.object.idType)) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && dataIn->isObjectData(), SFP(HStr_Err_InvalidParameter));

    if (dataIn && dataIn->isObjectData()) {
        switch (dataIn->id.object.idType) {
            case (hid_t)HelioIdentity::Actuator:
                return newActuatorObjectFromData((HelioActuatorData *)dataIn);
            case (hid_t)HelioIdentity::Sensor:
                return newSensorObjectFromData((HelioSensorData *)dataIn);
            case (hid_t)HelioIdentity::Panel:
                return newPanelObjectFromData((HelioPanelData *)dataIn);
            case (hid_t)HelioIdentity::Rail:
                return newRailObjectFromData((HelioRailData *)dataIn);
            default: // Unable
                return nullptr;
        }
    }

    return nullptr;
}


hkey_t HelioIdentity::regenKey()
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

            hposi_t linksIndex = 0;
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
        hposi_t linksIndex = 0;
        for (; linksIndex < _linksSize && _links[linksIndex].first; ++linksIndex) {
            if (_links[linksIndex].first == obj) {
                _links[linksIndex].second++;
                return true;
            }
        }
        if (linksIndex >= _linksSize) { allocateLinkages(_linksSize << 1); } // grow *2 if too small
        if (linksIndex < _linksSize) {
            _links[linksIndex] = make_pair(obj, (int8_t)1);
            return true;
        }
    }
    return false;
}

bool HelioObject::removeLinkage(HelioObject *obj)
{
    if (_links) {
        for (hposi_t linksIndex = 0; linksIndex < _linksSize && _links[linksIndex].first; ++linksIndex) {
            if (_links[linksIndex].first == obj) {
                if (--_links[linksIndex].second <= 0) {
                    for (int linksSubIndex = linksIndex; linksSubIndex < _linksSize - 1; ++linksSubIndex) {
                        _links[linksSubIndex] = _links[linksSubIndex + 1];
                    }
                    _links[_linksSize - 1] = make_pair((HelioObject *)nullptr, (int8_t)0);
                }
                return true;
            }
        }
    }
    return false;
}

bool HelioObject::hasLinkage(HelioObject *obj) const
{
    if (_links) {
        for (hposi_t linksIndex = 0; linksIndex < _linksSize && _links[linksIndex].first; ++linksIndex) {
            if (_links[linksIndex].first == obj) {
                return true;
            }
        }
    }
    return false;
}

void HelioObject::unresolveAny(HelioObject *obj)
{
    if (this == obj && _links) {
        HelioObject *lastObject = nullptr;
        for (hposi_t linksIndex = 0; linksIndex < _linksSize && _links[linksIndex].first; ++linksIndex) {
            HelioObject *object = _links[linksIndex].first;
            if (object != obj) {
                object->unresolveAny(obj); // may clobber indexing

                if (linksIndex && _links[linksIndex].first != object) {
                    while (linksIndex && _links[linksIndex].first != lastObject) { --linksIndex; }
                    object = lastObject;
                }
            }
            lastObject = object;
        }
    }
}

HelioIdentity HelioObject::getId() const
{
    return _id;
}

hkey_t HelioObject::getKey() const
{
    return _id.key;
}

String HelioObject::getKeyString() const
{
    return _id.keyString;
}

SharedPtr<HelioObjInterface> HelioObject::getSharedPtr() const
{
    return getController() ? static_pointer_cast<HelioObjInterface>(getController()->objectById(_id)) : nullptr;
}

SharedPtr<HelioObjInterface> HelioObject::getSharedPtrFor(const HelioObjInterface *obj) const
{
    return obj->isObject() ? obj->getSharedPtr() : nullptr;
}

bool HelioObject::isObject() const
{
    return true;
}

HelioData *HelioObject::allocateData() const
{
    HELIO_HARD_ASSERT(false, SFP(HStr_Err_UnsupportedOperation));
    return new HelioData();
}

void HelioObject::saveToData(HelioData *dataOut)
{
    dataOut->id.object.idType = (hid_t)_id.type;
    dataOut->id.object.objType = _id.objTypeAs.idType;
    dataOut->id.object.posIndex = _id.posIndex;
    dataOut->_revision = getRevision();
    if (_id.keyString.length()) {
        strncpy(((HelioObjectData *)dataOut)->name, _id.keyString.c_str(), HELIO_NAME_MAXSIZE);
    }
}


void HelioSubObject::unresolveAny(HelioObject *obj)
{ ; }

HelioIdentity HelioSubObject::getId() const
{
    return HelioIdentity(getKey());
}

hkey_t HelioSubObject::getKey() const
{
    return (hkey_t)(intptr_t)this;
}

String HelioSubObject::getKeyString() const
{
    return addressToString((uintptr_t)this);
}

SharedPtr<HelioObjInterface> HelioSubObject::getSharedPtr() const
{
    return _parent ? _parent->getSharedPtrFor((const HelioObjInterface *)this) : nullptr;
}

SharedPtr<HelioObjInterface> HelioSubObject::getSharedPtrFor(const HelioObjInterface *obj) const
{
    return obj->isObject() ? obj->getSharedPtr() : _parent ? _parent->getSharedPtrFor(obj) : nullptr;
}

bool HelioSubObject::isObject() const
{
    return false;
}

void HelioSubObject::setParent(HelioObjInterface *parent)
{
    _parent = parent;
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

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Data
*/

#include "Helioduino.h"

size_t serializeDataToBinaryStream(const HelioData *data, Stream *streamOut, size_t skipBytes)
{
    return streamOut->write((const uint8_t *)((intptr_t)data + skipBytes), data->_size - skipBytes);
}

size_t deserializeDataFromBinaryStream(HelioData *data, Stream *streamIn, size_t skipBytes)
{
    return streamIn->readBytes((uint8_t *)((intptr_t)data + skipBytes), data->_size - skipBytes);
}

HelioData *newDataFromBinaryStream(Stream *streamIn)
{
    HelioData baseDecode;
    size_t readBytes = deserializeDataFromBinaryStream(&baseDecode, streamIn, sizeof(void*));
    HELIO_SOFT_ASSERT(readBytes == baseDecode._size - sizeof(void*), SFP(HStr_Err_ImportFailure));

    if (readBytes) {
        HelioData *data = _allocateDataFromBaseDecode(baseDecode);
        HELIO_SOFT_ASSERT(data, SFP(HStr_Err_AllocationFailure));

        if (data) {
            readBytes += deserializeDataFromBinaryStream(data, streamIn, readBytes + sizeof(void*));
            HELIO_SOFT_ASSERT(readBytes == data->_size - sizeof(void*), SFP(HStr_Err_ImportFailure));

            return data;
        }
    }

    return nullptr;
}

HelioData *newDataFromJSONObject(JsonObjectConst &objectIn)
{
    HelioData baseDecode;
    baseDecode.fromJSONObject(objectIn);

    HelioData *data = _allocateDataFromBaseDecode(baseDecode);
    HELIO_SOFT_ASSERT(data, SFP(HStr_Err_AllocationFailure));

    if (data) {
        data->fromJSONObject(objectIn);
        return data;
    }

    return nullptr;
}


HelioData::HelioData()
    : id{.chars={'\0','\0','\0','\0'}}, _version(1), _revision(1), _modified(false)
{
    _size = sizeof(*this);
}

HelioData::HelioData(char id0, char id1, char id2, char id3, uint8_t version, uint8_t revision)
    : id{.chars={id0,id1,id2,id3}}, _version(version), _revision(revision), _modified(false)
{
    _size = sizeof(*this);
    HELIO_HARD_ASSERT(isStandardData(), SFP(HStr_Err_InvalidParameter));
}

HelioData::HelioData(hid_t idType, hid_t objType, hposi_t posIndex, hid_t classType, uint8_t version, uint8_t revision)
    : id{.object={idType,objType,posIndex,classType}}, _version(version), _revision(revision), _modified(false)
{
    _size = sizeof(*this);
}

HelioData::HelioData(const HelioIdentity &id)
    : HelioData(id.type, (int8_t)id.objTypeAs.actuatorType, id.posIndex, -1, 1, 1)
{
    _size = sizeof(*this);
}

void HelioData::toJSONObject(JsonObject &objectOut) const
{
    if (isStandardData()) {
        objectOut[SFP(HStr_Key_Type)] = charsToString(id.chars, sizeof(id.chars));
    } else {
        int8_t typeVals[4] = {id.object.idType, id.object.objType, id.object.posIndex, id.object.classType};
        objectOut[SFP(HStr_Key_Type)] = commaStringFromArray(typeVals, 4);
    }
    if (_version > 1) { objectOut[SFP(HStr_Key_Version)] = _version; }
    if (_revision > 1) { objectOut[SFP(HStr_Key_Revision)] = _revision; }
}

void HelioData::fromJSONObject(JsonObjectConst &objectIn)
{
    JsonVariantConst idVar = objectIn[SFP(HStr_Key_Type)];
    const char *idStr = idVar.as<const char *>();
    if (idStr && idStr[0] == 'H') {
        strncpy(id.chars, idStr, 4);
    } else if (idStr) {
        int8_t typeVals[4];
        commaStringToArray(idStr, typeVals, 4);
        id.object.idType = typeVals[0];
        id.object.objType = typeVals[1];
        id.object.posIndex = typeVals[2];
        id.object.classType = typeVals[3];
    }
    _version = objectIn[SFP(HStr_Key_Version)] | _version;
    _revision = objectIn[SFP(HStr_Key_Revision)] | _revision;
}


HelioSubData::HelioSubData()
    : type(hid_none)
{ ; }

HelioSubData::HelioSubData(hid_t dataType)
    : type(dataType)
{ ; }

void HelioSubData::toJSONObject(JsonObject &objectOut) const
{
    if (isSet()) { objectOut[SFP(HStr_Key_Type)] = type; }
}

void HelioSubData::fromJSONObject(JsonObjectConst &objectIn)
{
    type = objectIn[SFP(HStr_Key_Type)] | type;
}

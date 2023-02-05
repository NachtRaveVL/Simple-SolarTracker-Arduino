/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Attachment Points
*/

#include "Helioduino.h"

inline HelioDLinkObject &HelioDLinkObject::operator=(HelioIdentity rhs)
{
    _key = rhs.key;
    _obj = nullptr;
    if (_keyStr) { free((void *)_keyStr); _keyStr = nullptr; }

    auto len = rhs.keyString.length();
    if (len) {
        _keyStr = (const char *)malloc(len + 1);
        strncpy((char *)_keyStr, rhs.keyString.c_str(), len + 1);
    }
    return *this;
}

inline HelioDLinkObject &HelioDLinkObject::operator=(const char *rhs)
{
    _key = stringHash(rhs);
    _obj = nullptr;
    if (_keyStr) { free((void *)_keyStr); _keyStr = nullptr; }

    auto len = strnlen(rhs, HELIO_NAME_MAXSIZE);
    if (len) {
        _keyStr = (const char *)malloc(len + 1);
        strncpy((char *)_keyStr, rhs, len + 1);
    }
    return *this;
}

inline HelioDLinkObject &HelioDLinkObject::operator=(const HelioObjInterface *rhs)
{
    _key = rhs ? rhs->getKey() : (hkey_t)-1;
    _obj = rhs ? getSharedPtr<HelioObjInterface>(rhs) : nullptr;
    if (_keyStr) { free((void *)_keyStr); _keyStr = nullptr; }

    return *this;
}

inline HelioDLinkObject &HelioDLinkObject::operator=(const HelioAttachment *rhs)
{
    _key = rhs ? rhs->getKey() : (hkey_t)-1;
    _obj = rhs && rhs->isResolved() ? rhs->getSharedPtr() : nullptr;
    if (_keyStr) { free((void *)_keyStr); _keyStr = nullptr; }

    if (rhs && !rhs->isResolved()) {
        String keyString = rhs->getKeyString();
        auto len = keyString.length();
        if (len) {
            _keyStr = (const char *)malloc(len + 1);
            strncpy((char *)_keyStr, keyString.c_str(), len + 1);
        }
    }
    return *this;
}

template<class U>
inline HelioDLinkObject &HelioDLinkObject::operator=(SharedPtr<U> &rhs)
{
    _key = rhs ? rhs->getKey() : (hkey_t)-1;
    _obj = rhs ? reinterpret_pointer_cast<HelioObjInterface>(rhs) : nullptr;
    if (_keyStr) { free((void *)_keyStr); _keyStr = nullptr; }

    return *this;
}


template<class U>
void HelioAttachment::setObject(U obj)
{
    if (!(_obj == obj)) {
        if (_obj.isResolved()) { detachObject(); }

        _obj = obj; // will be replaced by templated operator= inline

        if (_obj.isResolved()) { attachObject(); }
    }
}

template<class U>
SharedPtr<U> HelioAttachment::getObject()
{
    if (_obj) { return _obj.getObject<U>(); }
    if (_obj.getKey() == (hkey_t)-1) { return nullptr; }

    if (_obj.needsResolved() && _obj._getObject()) {
        attachObject();
    }
    return _obj.getObject<U>();
}


template<class ParameterType, int Slots> template<class U>
HelioSignalAttachment<ParameterType,Slots>::HelioSignalAttachment(HelioObjInterface *parent, Signal<ParameterType,Slots> &(U::*signalGetter)(void))
    : HelioAttachment(parent), _signalGetter((SignalGetterPtr)signalGetter), _handleSlot(nullptr)
{
    HELIO_HARD_ASSERT(_signalGetter, SFP(HStr_Err_InvalidParameter));
}

template<class ParameterType, int Slots>
HelioSignalAttachment<ParameterType,Slots>::HelioSignalAttachment(const HelioSignalAttachment<ParameterType,Slots> &attachment)
    : HelioAttachment(attachment), _signalGetter((SignalGetterPtr)attachment._signalGetter),
      _handleSlot(attachment._handleSlot ? attachment._handleSlot->clone() : nullptr)
{
    HELIO_HARD_ASSERT(_signalGetter, SFP(HStr_Err_InvalidParameter));
}

template<class ParameterType, int Slots>
HelioSignalAttachment<ParameterType,Slots>::~HelioSignalAttachment()
{
    if (isResolved() && _handleSlot) {
        (get()->*_signalGetter)().detach(*_handleSlot);
    }
    if (_handleSlot) {
        delete _handleSlot; _handleSlot = nullptr;
    }
}

template<class ParameterType, int Slots>
void HelioSignalAttachment<ParameterType,Slots>::attachObject()
{
    HelioAttachment::attachObject();

    if (_handleSlot) {
        (get()->*_signalGetter)().attach(*_handleSlot);
    }
}

template<class ParameterType, int Slots>
void HelioSignalAttachment<ParameterType,Slots>::detachObject()
{
    if (isResolved() && _handleSlot) {
        (get()->*_signalGetter)().detach(*_handleSlot);
    }

    HelioAttachment::detachObject();
}

template<class ParameterType, int Slots>
void HelioSignalAttachment<ParameterType,Slots>::setHandleSlot(const Slot<ParameterType> &handleSlot)
{
    if (!_handleSlot || !_handleSlot->operator==(&handleSlot)) {
        if (isResolved() && _handleSlot) { (get()->*_signalGetter)().detach(*_handleSlot); }

        if (_handleSlot) { delete _handleSlot; _handleSlot = nullptr; }
        _handleSlot = handleSlot.clone();

        if (isResolved() && _handleSlot) { (get()->*_signalGetter)().attach(*_handleSlot); }
    }
}


inline Helio_TriggerState HelioTriggerAttachment::getTriggerState()
{
    return resolve() ? get()->getTriggerState() : Helio_TriggerState_Undefined;
}


inline Helio_DrivingState HelioDriverAttachment::getDrivingState()
{
    return resolve() ? get()->getDrivingState() : Helio_DrivingState_Undefined;
}

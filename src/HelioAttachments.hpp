/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Attachment Points
*/

#include "Helioduino.h"

inline HelioDLinkObject &HelioDLinkObject::operator=(HelioIdentity rhs)
{
    _key = rhs.key;
    _obj = nullptr;
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
    auto len = strnlen(rhs, HELIO_NAME_MAXSIZE);
    if (len) {
        _keyStr = (const char *)malloc(len + 1);
        strncpy((char *)_keyStr, rhs, len + 1);
    }
    return *this;
}

inline HelioDLinkObject &HelioDLinkObject::operator=(const HelioObjInterface *rhs)
{
    _key = rhs ? rhs->getKey() : (Helio_KeyType)-1;
    _obj = rhs ? getSharedPtr<HelioObjInterface>(rhs) : nullptr;
    if (_keyStr) { free((void *)_keyStr); _keyStr = nullptr; }
    return *this;
}

template<class U>
inline HelioDLinkObject &HelioDLinkObject::operator=(SharedPtr<U> &rhs)
{
    _key = rhs ? rhs->getKey() : (Helio_KeyType)-1;
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
    if (_obj.getKey() == (Helio_KeyType)-1) { return nullptr; }

    if (_obj.needsResolved() && _obj._getObject()) {
        attachObject();
    }
    return _obj.getObject<U>();
}


template<class ParameterType, int Slots> template<class U>
HelioSignalAttachment<ParameterType,Slots>::HelioSignalAttachment(HelioObjInterface *parent, Signal<ParameterType,Slots> &(U::*signalGetter)(void))
    : HelioAttachment(parent), _signalGetter((SignalGetterPtr)signalGetter), _handleMethod(nullptr)
{
    HELIO_HARD_ASSERT(_signalGetter, SFP(HStr_Err_InvalidParameter));
}

template<class ParameterType, int Slots>
HelioSignalAttachment<ParameterType,Slots>::HelioSignalAttachment(const HelioSignalAttachment<ParameterType,Slots> &attachment)
    : HelioAttachment(attachment._parent), _signalGetter((SignalGetterPtr)attachment._signalGetter), _handleMethod((HandleMethodSlotPtr)(attachment._handleMethod ? attachment._handleMethod->clone() : nullptr))
{
    HELIO_HARD_ASSERT(_signalGetter, SFP(HStr_Err_InvalidParameter));
}

template<class ParameterType, int Slots>
HelioSignalAttachment<ParameterType,Slots>::~HelioSignalAttachment()
{
    if (isResolved() && _handleMethod) {
        (get()->*_signalGetter)().detach(*_handleMethod);
    }
}

template<class ParameterType, int Slots>
void HelioSignalAttachment<ParameterType,Slots>::attachObject()
{
    HelioAttachment::attachObject();

    if (_handleMethod) {
        (get()->*_signalGetter)().attach(*_handleMethod);
    }
}

template<class ParameterType, int Slots>
void HelioSignalAttachment<ParameterType,Slots>::detachObject()
{
    if (isResolved() && _handleMethod) {
        (get()->*_signalGetter)().detach(*_handleMethod);
    }

    HelioAttachment::detachObject();
}

template<class ParameterType, int Slots> template<class U>
void HelioSignalAttachment<ParameterType,Slots>::setHandleMethod(MethodSlot<U,ParameterType> handleMethod)
{
    if (!_handleMethod || (*_handleMethod == handleMethod)) {
        if (isResolved() && _handleMethod) { (get()->*_signalGetter)().detach(*_handleMethod); }

        if (_handleMethod) { delete _handleMethod; _handleMethod = nullptr; }
        _handleMethod = (HandleMethodSlotPtr)handleMethod.clone();

        if (isResolved() && _handleMethod) { (get()->*_signalGetter)().attach(*_handleMethod); }
    }
}

inline void HelioSensorAttachment::updateIfNeeded(bool poll)
{
    if (resolve() && (_needsMeasurement || poll)) {
        if (_handleMethod) { _handleMethod->operator()(get()->getLatestMeasurement()); }
        else { handleMeasurement(get()->getLatestMeasurement()); }

        get()->takeMeasurement((_needsMeasurement || poll));
    }
}


inline Helio_TriggerState HelioTriggerAttachment::getTriggerState()
{
    return resolve() ? get()->getTriggerState() : Helio_TriggerState_Undefined;
}

inline void HelioTriggerAttachment::updateIfNeeded()
{
    if (resolve()) { get()->update(); }
}


inline Helio_BalancerState HelioBalancerAttachment::getBalancerState()
{
    return resolve() ? get()->getBalancerState() : Helio_BalancerState_Undefined;
}

inline void HelioBalancerAttachment::updateIfNeeded()
{
    if (resolve()) { get()->update(); }
}

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
    _key = rhs ? rhs->getKey() : hkey_none;
    _obj = rhs ? rhs->getSharedPtr() : nullptr;
    if (_keyStr) { free((void *)_keyStr); _keyStr = nullptr; }

    return *this;
}

inline HelioDLinkObject &HelioDLinkObject::operator=(const HelioAttachment *rhs)
{
    _key = rhs ? rhs->getKey() : hkey_none;
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
    _key = rhs ? rhs->getKey() : hkey_none;
    _obj = rhs ? static_pointer_cast<HelioObjInterface>(rhs) : nullptr;
    if (_keyStr) { free((void *)_keyStr); _keyStr = nullptr; }

    return *this;
}


template<class U>
void HelioAttachment::setObject(U obj, bool modify)
{
    if (!(_obj == obj)) {
        if (_obj.isResolved()) { detachObject(); }

        _obj = obj; // will be replaced by templated operator= inline

        if (_obj.isResolved()) { attachObject(); }

        if (modify && _parent) {
            if (_parent->isObject()) {
                ((HelioObject *)_parent)->bumpRevisionIfNeeded();
            } else {
                ((HelioSubObject *)_parent)->bumpRevisionIfNeeded();
            }
        }
    }
}

template<class U>
SharedPtr<U> HelioAttachment::getObject()
{
    if (_obj) { return _obj.getObject<U>(); }
    else if (!_obj.isSet()) { return nullptr; }
    else if (_obj.needsResolved() && _obj.resolveObject()) {
        attachObject();
    }
    return _obj.getObject<U>();
}


template<class ParameterType, int Slots> template<class U>
HelioSignalAttachment<ParameterType,Slots>::HelioSignalAttachment(HelioObjInterface *parent, hposi_t subIndex, Signal<ParameterType,Slots> &(U::*signalGetter)(void))
    : HelioAttachment(parent, subIndex), _signalGetter((SignalGetterPtr)signalGetter), _handleSlot(nullptr)
{ ; }

template<class ParameterType, int Slots>
HelioSignalAttachment<ParameterType,Slots>::HelioSignalAttachment(const HelioSignalAttachment<ParameterType,Slots> &attachment)
    : HelioAttachment(attachment), _signalGetter((SignalGetterPtr)attachment._signalGetter),
      _handleSlot(attachment._handleSlot ? attachment._handleSlot->clone() : nullptr)
{ ; }

template<class ParameterType, int Slots>
HelioSignalAttachment<ParameterType,Slots>::~HelioSignalAttachment()
{
    if (isResolved() && _handleSlot && _signalGetter) {
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

    if (isResolved() && _handleSlot && _signalGetter) {
        (get()->*_signalGetter)().attach(*_handleSlot);
    }
}

template<class ParameterType, int Slots>
void HelioSignalAttachment<ParameterType,Slots>::detachObject()
{
    if (isResolved() && _handleSlot && _signalGetter) {
        (get()->*_signalGetter)().detach(*_handleSlot);
    }

    HelioAttachment::detachObject();
}

template<class ParameterType, int Slots> template<class U>
void HelioSignalAttachment<ParameterType,Slots>::setSignalGetter(Signal<ParameterType,Slots> &(U::*signalGetter)(void))
{
    if (_signalGetter != signalGetter) {
        if (isResolved() && _handleSlot && _signalGetter) { (get()->*_signalGetter)().detach(*_handleSlot); }

        _signalGetter = signalGetter;

        if (isResolved() && _handleSlot && _signalGetter) { (get()->*_signalGetter)().attach(*_handleSlot); }
    }
}

template<class ParameterType, int Slots>
void HelioSignalAttachment<ParameterType,Slots>::setHandleSlot(const Slot<ParameterType> &handleSlot)
{
    if (!_handleSlot || !_handleSlot->operator==(&handleSlot)) {
        if (isResolved() && _handleSlot && _signalGetter) { (get()->*_signalGetter)().detach(*_handleSlot); }

        if (_handleSlot) { delete _handleSlot; _handleSlot = nullptr; }
        _handleSlot = handleSlot.clone();

        if (isResolved() && _handleSlot && _signalGetter) { (get()->*_signalGetter)().attach(*_handleSlot); }
    }
}


inline Helio_UnitsType HelioActuatorAttachment::getActivationUnits()
{
    return resolve() && get()->getUserCalibrationData() ? get()->getUserCalibrationData()->calibrationUnits : Helio_UnitsType_Raw_1;
}

inline float HelioActuatorAttachment::getActiveDriveIntensity()
{
    return resolve() ? get()->getDriveIntensity() : 0.0f;
}

inline float HelioActuatorAttachment::getActiveCalibratedValue()
{
    return resolve() ? get()->getCalibratedValue() : 0.0f;
}

inline float HelioActuatorAttachment::getSetupDriveIntensity() const
{
    return _actSetup.intensity;
}

inline float HelioActuatorAttachment::getSetupCalibratedValue()
{
    return resolve() ? get()->calibrationTransform(_actSetup.intensity) : 0.0f;
}


inline Helio_TriggerState HelioTriggerAttachment::getTriggerState(bool poll)
{
    return resolve() ? get()->getTriggerState(poll) : Helio_TriggerState_Undefined;
}


inline Helio_DrivingState HelioDriverAttachment::getDrivingState(bool poll)
{
    return resolve() ? get()->getDrivingState(poll) : Helio_DrivingState_Undefined;
}

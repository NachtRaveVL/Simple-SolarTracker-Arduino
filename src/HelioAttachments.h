/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Attachment Points
*/

#ifndef HelioAttachments_H
#define HelioAttachments_H

class HelioDLinkObject;
class HelioAttachment;
template<class ParameterType, int Slots> class HelioSignalAttachment;
class HelioSensorAttachment;
class HelioTriggerAttachment;
class HelioBalancerAttachment;

#include "Helioduino.h"
#include "HelioObject.h"
#include "HelioMeasurements.h"

// forward decls
extern Helio_KeyType stringHash(String);
extern String addressToString(uintptr_t);

// Delay/Dynamic Loaded/Linked Object Reference
// Simple class for delay loading objects that get references to others during system
// load. T should be a derived class of HelioObjInterface, with getId() method.
class HelioDLinkObject {
public:
    HelioDLinkObject();
    HelioDLinkObject(const HelioDLinkObject &obj);
    virtual ~HelioDLinkObject();

    inline bool isUnresolved() const { return !_obj; }
    inline bool isResolved() const { return (bool)_obj; }
    inline bool needsResolved() const { return isUnresolved() && _key != (Helio_KeyType)-1; }
    inline bool resolve() { return isResolved() || (bool)getObject(); }
    void unresolve();

    template<class U> inline void setObject(U obj) { (*this) = obj; }
    template<class U = HelioObjInterface> inline SharedPtr<U> getObject() { return reinterpret_pointer_cast<U>(_getObject()); }
    template<class U = HelioObjInterface> inline U* get() { return getObject<U>().get(); }

    inline HelioIdentity getId() const { return _obj ? _obj->getId() : (_keyStr ? HelioIdentity(_keyStr) : HelioIdentity(_key)); }
    inline Helio_KeyType getKey() const { return _key; }
    inline String getKeyString() const { return _keyStr ? String(_keyStr) : (_obj ? _obj->getKeyString() : addressToString((uintptr_t)_key)); }

    inline operator bool() const { return isResolved(); }
    inline HelioObjInterface *operator->() { return get(); }

    inline HelioDLinkObject &operator=(HelioIdentity rhs);
    inline HelioDLinkObject &operator=(const char *rhs);
    template<class U> inline HelioDLinkObject &operator=(SharedPtr<U> &rhs);
    inline HelioDLinkObject &operator=(const HelioObjInterface *rhs);
    inline HelioDLinkObject &operator=(nullptr_t) { return this->operator=((HelioObjInterface *)nullptr); }

    inline bool operator==(const HelioIdentity &rhs) const { return _key == rhs.key; }
    inline bool operator==(const char *rhs) const { return _key == stringHash(rhs); }
    template<class U> inline bool operator==(const SharedPtr<U> &rhs) const { return _key == (rhs ? rhs->getKey() : (Helio_KeyType)-1); }
    inline bool operator==(const HelioObjInterface *rhs) const { return _key == (rhs ? rhs->getKey() : (Helio_KeyType)-1); }
    inline bool operator==(nullptr_t) const { return _key == (Helio_KeyType)-1; }

protected:
    Helio_KeyType _key;                                     // Object key
    SharedPtr<HelioObjInterface> _obj;                      // Shared pointer to object
    const char *_keyStr;                                    // Copy of id.keyString (if not resolved, or unresolved)

private:
    SharedPtr<HelioObjInterface> _getObject();
    friend class Helioduino;
    friend class HelioAttachment;
};

// Simple Attachment Point Base
// This attachment registers the parent object with the linked object's linkages upon
// dereference, and unregisters the parent object at time of destruction or reassignment.
class HelioAttachment : public HelioSubObject {
public:
    HelioAttachment(HelioObjInterface *parent);
    virtual ~HelioAttachment();

    virtual void attachObject();
    virtual void detachObject();

    inline bool isUnresolved() const { return !_obj; }
    inline bool isResolved() const { return (bool)_obj; }
    inline bool needsResolved() const { return _obj.needsResolved(); }
    inline bool resolve() { return isResolved() || (bool)getObject(); }

    template<class U> void setObject(U obj);
    template<class U = HelioObjInterface> SharedPtr<U> getObject();
    template<class U = HelioObjInterface> inline U* get() { return getObject<U>().get(); }

    inline HelioIdentity getId() const { return _obj.getId(); }
    inline Helio_KeyType getKey() const { return _obj.getKey(); }
    inline String getKeyString() const { return _obj.getKeyString(); }

    inline operator bool() const { return isResolved(); }
    inline HelioObjInterface* operator->() { return get<HelioObjInterface>(); }

    inline HelioAttachment &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioAttachment &operator=(const char *rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioAttachment &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioAttachment &operator=(const U *rhs) { setObject(rhs); return *this; }

    inline bool operator==(const HelioIdentity &rhs) const { return _obj == rhs; }
    inline bool operator==(const char *rhs) { return *this == HelioIdentity(rhs); }
    template<class U> inline bool operator==(const SharedPtr<U> &rhs) const { return _obj == rhs; }
    template<class U> inline bool operator==(const U *rhs) const { return _obj == rhs; }

protected:
    HelioDLinkObject _obj;                                  // Dynamic link object
    HelioObjInterface *_parent;                             // Parent object pointer (strong due to reverse ownership)
};


// Signal Attachment Point
// This attachment registers the parent object with a Signal getter off the linked object
// upon dereference, and unregisters the parent object from the Signal at time of
// destruction or reassignment.
template<class ParameterType, int Slots = 8>
class HelioSignalAttachment : public HelioAttachment {
public:
    typedef Signal<ParameterType,Slots> &(HelioObjInterface::*SignalGetterPtr)(void);
    typedef void (HelioObjInterface::*HandleMethodPtr)(ParameterType);
    typedef MethodSlot<HelioObjInterface,ParameterType> *HandleMethodSlotPtr;

    template<class U> HelioSignalAttachment(HelioObjInterface *parent, Signal<ParameterType,Slots> &(U::*signalGetter)(void));
    HelioSignalAttachment(const HelioSignalAttachment<ParameterType,Slots> &attachment);
    virtual ~HelioSignalAttachment();

    virtual void attachObject() override;
    virtual void detachObject() override;

    template<class U> void setHandleMethod(MethodSlot<U,ParameterType> handleMethod);
    template<class U> inline void setHandleMethod(void (U::*handleMethodPtr)(ParameterType)) { setHandleMethod<U>(MethodSlot<U,ParameterType>(reinterpret_cast<U *>(_parent), handleMethodPtr)); }

    inline HelioSignalAttachment<ParameterType,Slots> &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioSignalAttachment<ParameterType,Slots> &operator=(const char *rhs) { setObject(HelioIdentity(rhs)); return *this; }
    template<class U> inline HelioSignalAttachment<ParameterType,Slots> &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioSignalAttachment<ParameterType,Slots> &operator=(const U *rhs) { setObject(rhs); return *this; }

protected:
    SignalGetterPtr _signalGetter;                          // Signal getter method ptr (weak)
    HandleMethodSlotPtr _handleMethod;                      // Handler method slot (owned)
};


// Sensor Measurement Attachment Point
// This attachment registers the parent object with a Sensor's new measurement Signal
// upon dereference, and unregisters the parent object from the Sensor at time of
// destruction or reassignment.
// Custom handle method will require a call into setMeasurement.
class HelioSensorAttachment : public HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_MEASUREMENT_SLOTS> {
public:
    typedef void (HelioObjInterface::*HandleMethodPtr)(const HelioMeasurement *);

    HelioSensorAttachment(HelioObjInterface *parent, uint8_t measurementRow = 0);

    virtual void attachObject() override;
    virtual void detachObject() override;

    inline void updateIfNeeded(bool poll = false);

    void setMeasurement(float value, Helio_UnitsType units = Helio_UnitsType_Undefined);
    void setMeasurement(HelioSingleMeasurement measurement);
    void setMeasurementRow(uint8_t measurementRow);
    void setMeasurementUnits(Helio_UnitsType units, float convertParam = FLT_UNDEF);

    inline void setNeedsMeasurement() { _needsMeasurement = true; }
    inline bool needsMeasurement() { return _needsMeasurement; }

    inline const HelioSingleMeasurement &getMeasurement(bool poll = false) { updateIfNeeded(poll); return _measurement; }
    inline uint16_t getMeasurementFrame(bool poll = false) { updateIfNeeded(poll); return _measurement.frame; }
    inline float getMeasurementValue(bool poll = false) { updateIfNeeded(poll); return _measurement.value; }
    inline Helio_UnitsType getMeasurementUnits() const { return _measurement.units; }

    inline uint8_t getMeasurementRow() const { return _measurementRow; }
    inline float getMeasurementConvertParam() const { return _convertParam; }

    inline SharedPtr<HelioSensor> getObject() { return HelioAttachment::getObject<HelioSensor>(); }
    inline HelioSensor *get() { return HelioAttachment::get<HelioSensor>(); }

    inline HelioSensor &operator*() { return *HelioAttachment::getObject<HelioSensor>().get(); }
    inline HelioSensor *operator->() { return HelioAttachment::getObject<HelioSensor>().get(); }

    inline HelioSensorAttachment &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioSensorAttachment &operator=(const char *rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioSensorAttachment &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioSensorAttachment &operator=(const U *rhs) { setObject(rhs); return *this; }

protected:
    HelioSingleMeasurement _measurement;                    // Local measurement (converted to measure units)
    uint8_t _measurementRow;                                // Measurement row
    float _convertParam;                                    // Convert param (default: FLT_UNDEF)
    bool _needsMeasurement;                                 // Stale measurement tracking flag

    void handleMeasurement(const HelioMeasurement *measurement);
};


// Trigger State Attachment Point
// This attachment registers the parent object with a Triggers's trigger Signal
// upon dereference, and unregisters the parent object from the Trigger at time of
// destruction or reassignment.
class HelioTriggerAttachment  : public HelioSignalAttachment<Helio_TriggerState, HELIO_TRIGGER_STATE_SLOTS> {
public:
    typedef void (HelioObjInterface::*HandleMethodPtr)(Helio_TriggerState);

    HelioTriggerAttachment(HelioObjInterface *parent);

    inline void updateIfNeeded();

    inline Helio_TriggerState getTriggerState();

    inline SharedPtr<HelioTrigger> getObject() { return HelioAttachment::getObject<HelioTrigger>(); }
    inline HelioTrigger *get() { return HelioAttachment::get<HelioTrigger>(); }

    inline HelioTrigger &operator*() { return *HelioAttachment::getObject<HelioTrigger>().get(); }
    inline HelioTrigger *operator->() { return HelioAttachment::getObject<HelioTrigger>().get(); }

    inline HelioTriggerAttachment &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioTriggerAttachment &operator=(const char *rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioTriggerAttachment &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioTriggerAttachment &operator=(const U *rhs) { setObject(rhs); return *this; }
};


// Balancer State Attachment Point
// This attachment registers the parent object with a Balancer's balance Signal
// upon dereference, and unregisters the parent object from the Balancer at time of
// destruction or reassignment.
class HelioBalancerAttachment : public HelioSignalAttachment<Helio_BalancerState, HELIO_BALANCER_STATE_SLOTS> {
public:
    typedef void (HelioObjInterface::*HandleMethodPtr)(Helio_BalancerState);

    HelioBalancerAttachment(HelioObjInterface *parent);

    inline void updateIfNeeded();

    inline Helio_BalancerState getBalancerState();

    inline SharedPtr<HelioBalancer> getObject() { return HelioAttachment::getObject<HelioBalancer>(); }
    inline HelioBalancer *get() { return HelioAttachment::get<HelioBalancer>(); }

    inline HelioBalancer &operator*() { return *HelioAttachment::getObject<HelioBalancer>().get(); }
    inline HelioBalancer *operator->() { return HelioAttachment::getObject<HelioBalancer>().get(); }

    inline HelioBalancerAttachment &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioBalancerAttachment &operator=(const char *rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioBalancerAttachment &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioBalancerAttachment &operator=(const U *rhs) { setObject(rhs); return *this; }
};

#endif // /ifndef HelioAttachments_H

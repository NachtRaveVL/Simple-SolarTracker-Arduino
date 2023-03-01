/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Attachment Points
*/

#ifndef HelioAttachments_H
#define HelioAttachments_H

class HelioDLinkObject;
class HelioAttachment;
template<class ParameterType, int Slots> class HelioSignalAttachment;
class HelioActuatorAttachment;
class HelioSensorAttachment;
class HelioTriggerAttachment;
class HelioDriverAttachment;

#include "Helioduino.h"
#include "HelioObject.h"
#include "HelioMeasurements.h"
#include "HelioActivation.h"

// Delay/Dynamic Loaded/Linked Object Reference
// Simple class for delay loading objects that get references to others during object
// load. T should be a derived class of HelioObjInterface, with getId() method.
class HelioDLinkObject {
public:
    HelioDLinkObject();
    HelioDLinkObject(const HelioDLinkObject &obj);
    virtual ~HelioDLinkObject();

    inline bool isUnresolved() const { return !_obj; }
    inline bool isResolved() const { return (bool)_obj; }
    inline bool needsResolved() const { return isUnresolved() && isSet(); }
    inline bool resolve() { return isResolved() || (bool)getObject(); }
    void unresolve();
    template<class U> inline void unresolveIf(U obj) { if (operator==(obj)) { unresolve(); } }

    template<class U> inline void setObject(U obj) { operator=(obj); }
    template<class U = HelioObjInterface> inline SharedPtr<U> getObject() { return reinterpret_pointer_cast<U>(resolveObject()); }
    template<class U = HelioObjInterface> inline U *get() { return getObject<U>().get(); }

    inline HelioIdentity getId() const { return _obj ? _obj->getId() : (_keyStr ? HelioIdentity(_keyStr) : HelioIdentity(_key)); }
    inline hkey_t getKey() const { return _key; }
    inline String getKeyString() const { return _keyStr ? String(_keyStr) : (_obj ? _obj->getKeyString() : addressToString((uintptr_t)_key)); }
    inline bool isSet() const { return _key != hkey_none; }

    inline operator bool() const { return isResolved(); }
    inline HelioObjInterface *operator->() { return get(); }

    inline HelioDLinkObject &operator=(HelioIdentity rhs);
    inline HelioDLinkObject &operator=(const char *rhs);
    template<class U> inline HelioDLinkObject &operator=(SharedPtr<U> &rhs);
    inline HelioDLinkObject &operator=(const HelioObjInterface *rhs);
    inline HelioDLinkObject &operator=(const HelioAttachment *rhs);
    inline HelioDLinkObject &operator=(nullptr_t) { return operator=((HelioObjInterface *)nullptr); }

    inline bool operator==(const HelioIdentity &rhs) const { return _key == rhs.key; }
    inline bool operator==(const char *rhs) const { return _key == stringHash(rhs); }
    template<class U> inline bool operator==(const SharedPtr<U> &rhs) const { return _key == (rhs ? rhs->getKey() : hkey_none); }
    inline bool operator==(const HelioObjInterface *rhs) const { return _key == (rhs ? rhs->getKey() : hkey_none); }
    inline bool operator==(nullptr_t) const { return _key == hkey_none; }

protected:
    hkey_t _key;                                            // Object key
    SharedPtr<HelioObjInterface> _obj;                      // Shared pointer to object
    const char *_keyStr;                                    // Copy of id.keyString (if not resolved, or unresolved)

private:
    SharedPtr<HelioObjInterface> resolveObject();
    friend class Helioduino;
    friend class HelioAttachment;
};

// Simple Attachment Point Base
// This attachment registers the parent object with the linked object's linkages upon
// dereference / unregisters the parent object at time of destruction or reassignment.
class HelioAttachment : public HelioSubObject {
public:
    HelioAttachment(HelioObjInterface *parent = nullptr, hposi_t subIndex = 0);
    HelioAttachment(const HelioAttachment &attachment);
    virtual ~HelioAttachment();

    // Attaches object and any relevant signaling mechanisms. Derived classes should call base class's method first.
    virtual void attachObject();
    // Detaches object from any relevant signaling mechanism. Derived classes should call base class's method last.
    virtual void detachObject();

    // Attachment updater. Overridden by derived classes. May only update owned sub-objects (main objects are owned/updated by run system).
    virtual void updateIfNeeded(bool poll = false);

    inline bool isUnresolved() const { return !_obj; }
    inline bool isResolved() const { return (bool)_obj; }
    inline bool needsResolved() const { return _obj.needsResolved(); }
    inline bool resolve() { return isResolved() || (bool)getObject(); }
    inline void unresolve() { _obj.unresolve(); } 
    template<class U> inline void unresolveIf(U obj) { _obj.unresolveIf(obj); }

    template<class U> void setObject(U obj, bool modify = true);
    template<class U> inline void initObject(U obj) { setObject(obj, false); }
    template<class U = HelioObjInterface> SharedPtr<U> getObject();
    template<class U = HelioObjInterface> inline U *get() { return getObject<U>().get(); }

    virtual void setParent(HelioObjInterface *parent) override;
    inline void setParent(HelioObjInterface *parent, hposi_t subIndex) { setParent(parent); setParentSubIndex(subIndex); }
    inline void setParentSubIndex(hposi_t subIndex) { _subIndex = subIndex; }
    inline HelioObjInterface *getParent() { return _parent; }
    inline hposi_t getParentSubIndex() { return _subIndex; }

    inline HelioIdentity getId() const { return _obj.getId(); }
    inline hkey_t getKey() const { return _obj.getKey(); }
    inline String getKeyString() const { return _obj.getKeyString(); }
    inline bool isSet() const { return _obj.isSet(); }
    virtual SharedPtr<HelioObjInterface> getSharedPtrFor(const HelioObjInterface *obj) const override;

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
    hposi_t _subIndex;                                      // Parent sub index, else 0
};


// Signal Attachment Point
// This attachment registers the parent object with a signal getter off the linked object
// upon resolvement / unregisters the parent object from the signal at time of destruction
// or reassignment.
template<class ParameterType, int Slots = 8>
class HelioSignalAttachment : public HelioAttachment {
public:
    typedef Signal<ParameterType,Slots> &(HelioObjInterface::*SignalGetterPtr)(void);

    template<class U> HelioSignalAttachment(HelioObjInterface *parent = nullptr, hposi_t subIndex = 0, Signal<ParameterType,Slots> &(U::*signalGetter)(void) = nullptr);
    HelioSignalAttachment(const HelioSignalAttachment<ParameterType,Slots> &attachment);
    virtual ~HelioSignalAttachment();

    virtual void attachObject() override;
    virtual void detachObject() override;

    // Sets the signal handler getter method to use
    template<class U> void setSignalGetter(Signal<ParameterType,Slots> &(U::*signalGetter)(void));

    // Sets a handle slot to run when attached signal fires
    void setHandleSlot(const Slot<ParameterType> &handleSlot);
    inline void setHandleFunction(void (*handleFunctionPtr)(ParameterType)) { setHandleSlot(FunctionSlot<ParameterType>(handleFunctionPtr)); }
    template<class U, class V = U> inline void setHandleMethod(void (U::*handleMethodPtr)(ParameterType), V *handleClassInst = nullptr) { setHandleSlot(MethodSlot<V,ParameterType>(handleClassInst ? handleClassInst : static_cast<V *>(_parent), handleMethodPtr)); }

    inline HelioSignalAttachment<ParameterType,Slots> &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioSignalAttachment<ParameterType,Slots> &operator=(const char *rhs) { setObject(HelioIdentity(rhs)); return *this; }
    template<class U> inline HelioSignalAttachment<ParameterType,Slots> &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioSignalAttachment<ParameterType,Slots> &operator=(const U *rhs) { setObject(rhs); return *this; }

protected:
    SignalGetterPtr _signalGetter;                          // Signal getter method ptr (weak)
    Slot<ParameterType> *_handleSlot;                       // Handler slot (owned)
};


// Actuator Attachment Point
// This attachment interfaces with actuator activation handles for actuator control, and
// registers the parent object with an actuator upon resolvement / unregisters the parent
// object from the actuator at time of destruction or reassignment.
class HelioActuatorAttachment : public HelioSignalAttachment<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS> {
public:
    HelioActuatorAttachment(HelioObjInterface *parent = nullptr, hposi_t subIndex = 0);
    HelioActuatorAttachment(const HelioActuatorAttachment &attachment);
    virtual ~HelioActuatorAttachment();

    // Updates with actuator activation handle. Does not call actuator's update() (handled by system).
    virtual void updateIfNeeded(bool poll = false) override;

    // A rate multiplier is used to adjust either the intensity or duration of activations,
    // which depends on whenever they operate in binary mode (on/off) or variably (ranged).
    inline void setRateMultiplier(float rateMultiplier) { if (!isFPEqual(_rateMultiplier, rateMultiplier)) { _rateMultiplier = rateMultiplier; applySetup(); } }
    inline float getRateMultiplier() const { return _rateMultiplier; }

    // Activations are set up first by calling one of these methods. This configures the
    // direction, intensity, duration, and any run flags that the actuator will operate
    // upon once enabled, pending any rate offsetting. These methods are re-entrant.
    // The most recently used setup values are used for repeat activations.
    inline void setupActivation(const HelioActivation &activation) { _actSetup = activation; applySetup(); }
    inline void setupActivation(const HelioActivationHandle &handle) { setupActivation(handle.activation); }
    inline void setupActivation(Helio_DirectionMode direction, float intensity = 1.0f, millis_t duration = -1, bool force = false) { setupActivation(HelioActivation(direction, intensity, duration, (force ? Helio_ActivationFlags_Forced : Helio_ActivationFlags_None))); }
    inline void setupActivation(millis_t duration = -1, bool force = false) { setupActivation(HelioActivation(Helio_DirectionMode_Forward, 1.0f, duration, (force ? Helio_ActivationFlags_Forced : Helio_ActivationFlags_None))); }
    // These activation methods take a variable value that gets transformed by any user
    // curvature calibration data before being used, assuming units to be the same. It is
    // otherwise assumed the value is a normalized driving intensity ([0,1] or [-1,1]).
    void setupActivation(float value, millis_t duration = -1, bool force = false);
    inline void setupActivation(const HelioSingleMeasurement &measurement, millis_t duration = -1, bool force = false) { setupActivation(measurement.value, duration, force); }

    // Gets what units are expected to be used in setupActivation() methods
    inline Helio_UnitsType getActivationUnits();

    // Enables activation handle with current setup, if not already active.
    // Repeat activations will reuse most recent setupActivation() values.
    void enableActivation();
    // Disables activation handle, if not already inactive.
    inline void disableActivation() { _actHandle.unset(); }

    // Activation status based on handle activation
    inline bool isActivated() const { return _actHandle.isActive(); }
    inline millis_t getTimeLeft() const { return _actHandle.getTimeLeft(); }
    inline millis_t getTimeActive(millis_t time = nzMillis()) const { return _actHandle.getTimeActive(time); }

    // Currently active driving intensity [-1.0,1.0] / calibrated value [calibMin,calibMax], from actuator
    inline float getActiveDriveIntensity();
    inline float getActiveCalibratedValue();

    // Currently setup driving intensity [-1.0,1.0] / calibrated value [calibMin,calibMax], from activation
    inline float getSetupDriveIntensity() const;
    inline float getSetupCalibratedValue();

    // Sets an update slot to run during execution of actuator that can further refine duration/intensity.
    // Useful for rate-based or variable activations. Slot receives actuator attachment pointer as parameter.
    // Guaranteed to be called with final finished activation.
    void setUpdateSlot(const Slot<HelioActuatorAttachment *> &updateSlot);
    inline void setUpdateFunction(void (*updateFunctionPtr)(HelioActuatorAttachment *)) { setUpdateSlot(FunctionSlot<HelioActuatorAttachment *>(updateFunctionPtr)); }
    template<class U> inline void setUpdateMethod(void (U::*updateMethodPtr)(HelioActivationHandle *), U *updateClassInst = nullptr) { setUpdateSlot(MethodSlot<U,HelioActuatorAttachment *>(updateClassInst ? updateClassInst : reinterpret_cast<U *>(_parent), updateMethodPtr)); }
    const Slot<HelioActuatorAttachment *> *getUpdateSlot() const { return _updateSlot; }

    inline const HelioActivationHandle &getActivationHandle() const { return _actHandle; }
    inline const HelioActivation &getActivationSetup() const { return _actSetup; }

    template<class U> inline void setObject(U obj, bool modify = false) { HelioAttachment::setObject(obj, modify); }
    inline SharedPtr<HelioActuator> getObject() { return HelioAttachment::getObject<HelioActuator>(); }
    inline HelioActuator *get() { return HelioAttachment::get<HelioActuator>(); }

    inline HelioActuator &operator*() { return *HelioAttachment::get<HelioActuator>(); }
    inline HelioActuator *operator->() { return HelioAttachment::get<HelioActuator>(); }

    inline HelioActuatorAttachment &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioActuatorAttachment &operator=(const char *rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioActuatorAttachment &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioActuatorAttachment &operator=(const U *rhs) { setObject(rhs); return *this; }

protected:
    HelioActivationHandle _actHandle;                       // Actuator activation handle (double ref to object when active)
    HelioActivation _actSetup;                              // Actuator activation setup
    Slot<HelioActuatorAttachment *> *_updateSlot;           // Update slot (owned)
    float _rateMultiplier;                                  // Rate multiplier
    bool _calledLastUpdate;                                 // Last update call flag

    void applySetup();
};


// Sensor Measurement Attachment Point
// This attachment registers the parent object with a sensor's new measurement signal
// upon resolvement / unregisters the parent object from the sensor at time of destruction
// or reassignment.
// Custom handle method is responsible for calling setMeasurement() to update measurement.
class HelioSensorAttachment : public HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS> {
public:
    HelioSensorAttachment(HelioObjInterface *parent = nullptr, hposi_t subIndex = 0, uint8_t measurementRow = 0);
    HelioSensorAttachment(const HelioSensorAttachment &attachment);
    virtual ~HelioSensorAttachment();

    virtual void attachObject() override;
    virtual void detachObject() override;

    // Updates measurement attachment with sensor. Does not call sensor's update() (handled by system).
    virtual void updateIfNeeded(bool poll = false) override;

    // Sets the current measurement associated with this process. Required to be called by custom handlers.
    void setMeasurement(HelioSingleMeasurement measurement);
    inline void setMeasurement(float value, Helio_UnitsType units = Helio_UnitsType_Undefined) { setMeasurement(HelioSingleMeasurement(value, units)); }
    void setMeasurementRow(uint8_t measurementRow);
    void setMeasurementUnits(Helio_UnitsType units, float convertParam = FLT_UNDEF);

    inline const HelioSingleMeasurement &getMeasurement(bool poll = false) { updateIfNeeded(poll); return _measurement; }
    inline uint16_t getMeasurementFrame(bool poll = false) { updateIfNeeded(poll); return _measurement.frame; }
    inline float getMeasurementValue(bool poll = false) { updateIfNeeded(poll); return _measurement.value; }
    inline Helio_UnitsType getMeasurementUnits() const { return _measurement.units; }

    inline void setNeedsMeasurement() { _needsMeasurement = true; }
    inline bool needsMeasurement() { return _needsMeasurement; }

    inline uint8_t getMeasurementRow() const { return _measurementRow; }
    inline float getMeasurementConvertParam() const { return _convertParam; }

    inline SharedPtr<HelioSensor> getObject(bool poll = false) { updateIfNeeded(poll); return HelioAttachment::getObject<HelioSensor>(); }
    inline HelioSensor *get() { return HelioAttachment::get<HelioSensor>(); }

    inline HelioSensor &operator*() { return *HelioAttachment::get<HelioSensor>(); }
    inline HelioSensor *operator->() { return HelioAttachment::get<HelioSensor>(); }

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
// This attachment registers the parent object with a triggers's trigger signal
// upon resolvement / unregisters the parent object from the trigger at time of
// destruction or reassignment.
class HelioTriggerAttachment  : public HelioSignalAttachment<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS> {
public:
    HelioTriggerAttachment(HelioObjInterface *parent = nullptr, hposi_t subIndex = 0);
    HelioTriggerAttachment(const HelioTriggerAttachment &attachment);
    virtual ~HelioTriggerAttachment();

    // Updates owned trigger attachment.
    virtual void updateIfNeeded(bool poll = false) override;

    inline Helio_TriggerState getTriggerState(bool poll = false);
    inline bool isTriggered(bool poll = false) { return getTriggerState(poll) == Helio_TriggerState_Triggered; }

    inline SharedPtr<HelioTrigger> getObject() { return HelioAttachment::getObject<HelioTrigger>(); }
    inline HelioTrigger *get() { return HelioAttachment::get<HelioTrigger>(); }

    inline HelioTrigger &operator*() { return *HelioAttachment::get<HelioTrigger>(); }
    inline HelioTrigger *operator->() { return HelioAttachment::get<HelioTrigger>(); }

    inline HelioTriggerAttachment &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioTriggerAttachment &operator=(const char *rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioTriggerAttachment &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioTriggerAttachment &operator=(const U *rhs) { setObject(rhs); return *this; }
};


// Driver Attachment Point
// This attachment registers the parent object with a driver's driving signal
// upon resolvement / unregisters the parent object from the driver at time of
// destruction or reassignment.
class HelioDriverAttachment : public HelioSignalAttachment<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> {
public:
    HelioDriverAttachment(HelioObjInterface *parent = nullptr, hposi_t subIndex = 0);
    HelioDriverAttachment(const HelioDriverAttachment &attachment);
    virtual ~HelioDriverAttachment();

    // Updates owned driver attachment.
    virtual void updateIfNeeded(bool poll = false) override;

    inline Helio_DrivingState getDrivingState(bool poll = false);

    template<class U> inline void setObject(U obj, bool modify = false) { HelioAttachment::setObject(obj, modify); }
    inline SharedPtr<HelioDriver> getObject() { return HelioAttachment::getObject<HelioDriver>(); }
    inline HelioDriver *get() { return HelioAttachment::get<HelioDriver>(); }

    inline HelioDriver &operator*() { return *HelioAttachment::get<HelioDriver>(); }
    inline HelioDriver *operator->() { return HelioAttachment::get<HelioDriver>(); }

    inline HelioDriverAttachment &operator=(const HelioIdentity &rhs) { setObject(rhs); return *this; }
    inline HelioDriverAttachment &operator=(const char *rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioDriverAttachment &operator=(SharedPtr<U> rhs) { setObject(rhs); return *this; }
    template<class U> inline HelioDriverAttachment &operator=(const U *rhs) { setObject(rhs); return *this; }
};

#endif // /ifndef HelioAttachments_H

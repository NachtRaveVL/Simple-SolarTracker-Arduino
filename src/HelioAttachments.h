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
    inline bool needsResolved() const { return isUnresolved() && _key != hkey_none; }
    inline bool resolve() { return isResolved() || (bool)getObject(); }
    void unresolve();
    template<class U> inline void unresolveIf(U obj) { if (operator==(obj)) { unresolve(); } }

    template<class U> inline void setObject(U obj) { operator=(obj); }
    template<class U = HelioObjInterface> inline SharedPtr<U> getObject() { return reinterpret_pointer_cast<U>(_getObject()); }
    template<class U = HelioObjInterface> inline U *get() { return getObject<U>().get(); }

    inline HelioIdentity getId() const { return _obj ? _obj->getId() : (_keyStr ? HelioIdentity(_keyStr) : HelioIdentity(_key)); }
    inline hkey_t getKey() const { return _key; }
    inline String getKeyString() const { return _keyStr ? String(_keyStr) : (_obj ? _obj->getKeyString() : addressToString((uintptr_t)_key)); }

    inline operator bool() const { return isResolved(); }
    inline HelioObjInterface *operator->() { return get(); }

    inline HelioDLinkObject &operator=(HelioIdentity rhs);
    inline HelioDLinkObject &operator=(const char *rhs);
    template<class U> inline HelioDLinkObject &operator=(SharedPtr<U> &rhs);
    inline HelioDLinkObject &operator=(const HelioObjInterface *rhs);
    inline HelioDLinkObject &operator=(const HelioAttachment *rhs);
    inline HelioDLinkObject &operator=(nullptr_t) { return this->operator=((HelioObjInterface *)nullptr); }

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
    SharedPtr<HelioObjInterface> _getObject();
    friend class Helioduino;
    friend class HelioAttachment;
};

// Simple Attachment Point Base
// This attachment registers the parent object with the linked object's linkages upon
// dereference / unregisters the parent object at time of destruction or reassignment.
class HelioAttachment : public HelioSubObject {
public:
    HelioAttachment(HelioObjInterface *parent = nullptr);
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

    template<class U> void setObject(U obj);
    template<class U = HelioObjInterface> SharedPtr<U> getObject();
    template<class U = HelioObjInterface> inline U *get() { return getObject<U>().get(); }

    virtual void setParent(HelioObjInterface *parent) override;
    inline HelioObjInterface *getParent() { return _parent; }

    inline HelioIdentity getId() const { return _obj.getId(); }
    inline hkey_t getKey() const { return _obj.getKey(); }
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
};


// Signal Attachment Point
// This attachment registers the parent object with a Signal getter off the linked object
// upon dereference / unregisters the parent object from the Signal at time of destruction
// or reassignment.
template<class ParameterType, int Slots = 8>
class HelioSignalAttachment : public HelioAttachment {
public:
    typedef Signal<ParameterType,Slots> &(HelioObjInterface::*SignalGetterPtr)(void);

    template<class U> HelioSignalAttachment(HelioObjInterface *parent = nullptr, Signal<ParameterType,Slots> &(U::*signalGetter)(void) = nullptr);
    HelioSignalAttachment(const HelioSignalAttachment<ParameterType,Slots> &attachment);
    virtual ~HelioSignalAttachment();

    virtual void attachObject() override;
    virtual void detachObject() override;

    // Sets the signal handler getter method to use
    template<class U> void setSignalGetter(Signal<ParameterType,Slots> &(U::*signalGetter)(void));

    // Sets a handle slot to run when attached signal fires
    void setHandleSlot(const Slot<ParameterType> &handleSlot);
    inline void setHandleFunction(void (*handleFunctionPtr)(ParameterType)) { setHandleSlot(FunctionSlot<ParameterType>(handleFunctionPtr)); }
    template<class U> inline void setHandleMethod(void (U::*handleMethodPtr)(ParameterType), U *handleClassInst = nullptr) { setHandleSlot(MethodSlot<U,ParameterType>(handleClassInst ? handleClassInst : reinterpret_cast<U *>(_parent), handleMethodPtr)); }

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
// registers the parent object with an Actuator upon dereference / unregisters the parent
// object from the Actuator at time of destruction or reassignment.
class HelioActuatorAttachment : public HelioSignalAttachment<HelioActuator *, HELIO_ACTUATOR_SIGNAL_SLOTS> {
public:
    HelioActuatorAttachment(HelioObjInterface *parent = nullptr);
    HelioActuatorAttachment(const HelioActuatorAttachment &attachment);
    virtual ~HelioActuatorAttachment();

    // Updates with actuator activation handle. Does not call actuator's update() (handled by system).
    virtual void updateIfNeeded(bool poll = false) override;

    // A rate multiplier is used to adjust either the intensity or duration of activations,
    // which depends on whenever they operate in binary mode (on/off) or variably (ranged).
    inline void setRateMultiplier(float rateMultiplier) { _rateMultiplier = rateMultiplier; applySetup(); }
    inline float getRateMultiplier() const { return _rateMultiplier; }

    // Activations are set up first by calling one of these methods. This configures the
    // direction, intensity, duration, and any run flags that the actuator will operate
    // upon once enabled, pending any rate offsetting. These methods are re-entrant.
    // The most recently used setup values are used for repeat activations.
    inline void setupActivation(const HelioActivation &activation);
    inline void setupActivation(const HelioActivationHandle &handle);
    inline void setupActivation(Helio_DirectionMode direction, float intensity = 1.0f, millis_t duration = -1, bool force = false);
    inline void setupActivation(millis_t duration, bool force = false);
    inline void setupActivation(bool force, millis_t duration = -1);
    // These activation methods take into account actuator settings such as user
    // calibration data and type checks in determining how to interpret passed value.
    void setupActivation(float value, millis_t duration = -1, bool force = false);
    inline void setupActivation(const HelioSingleMeasurement &measurement, millis_t duration = -1, bool force = false);

    // Enables activation handle with current setup, if not already active.
    // Repeat activations will reuse most recent setupActivation() values.
    void enableActivation();
    // Disables activation handle, if not already inactive.
    inline void disableActivation();

    // Activation status based on handle activation
    inline bool isActivated() const;
    inline millis_t getTimeLeft() const;
    inline millis_t getTimeActive(millis_t time = nzMillis()) const;

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

    inline const HelioActivationHandle &getHandle() const { return _actHandle; }
    inline const HelioActivation &getSetup() const { return _actSetup; }
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
    HelioActivation _actSetup;            // Actuator activation setup
    Slot<HelioActuatorAttachment *> *_updateSlot;           // Update slot (owned)
    float _rateMultiplier;                                  // Rate multiplier
    bool _calledLastUpdate;                                 // Last update call flag

    void applySetup();
};


// Sensor Measurement Attachment Point
// This attachment registers the parent object with a Sensor's new measurement Signal
// upon dereference / unregisters the parent object from the Sensor at time of destruction
// or reassignment.
// Custom handle method is responsible for calling setMeasurement() to update measurement.
class HelioSensorAttachment : public HelioSignalAttachment<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS> {
public:
    HelioSensorAttachment(HelioObjInterface *parent = nullptr, uint8_t measurementRow = 0);
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
// This attachment registers the parent object with a Triggers's trigger Signal
// upon dereference / unregisters the parent object from the Trigger at time of
// destruction or reassignment.
class HelioTriggerAttachment  : public HelioSignalAttachment<Helio_TriggerState, HELIO_TRIGGER_SIGNAL_SLOTS> {
public:
    HelioTriggerAttachment(HelioObjInterface *parent = nullptr);
    HelioTriggerAttachment(const HelioTriggerAttachment &attachment);
    virtual ~HelioTriggerAttachment();

    // Updates owned trigger attachment.
    virtual void updateIfNeeded(bool poll = false) override;

    inline Helio_TriggerState getTriggerState();

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
// This attachment registers the parent object with a Driver's driving Signal
// upon dereference / unregisters the parent object from the Driver at time of
// destruction or reassignment.
class HelioDriverAttachment : public HelioSignalAttachment<Helio_DrivingState, HELIO_DRIVER_SIGNAL_SLOTS> {
public:
    HelioDriverAttachment(HelioObjInterface *parent = nullptr);
    HelioDriverAttachment(const HelioDriverAttachment &attachment);
    virtual ~HelioDriverAttachment();

    // Updates owned balancer attachment.
    virtual void updateIfNeeded(bool poll = false) override;

    inline Helio_DrivingState getDrivingState();

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

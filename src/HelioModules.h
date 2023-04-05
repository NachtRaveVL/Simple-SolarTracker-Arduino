/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Controller Modules
*/

#ifndef HelioModules_H
#define HelioModules_H

class HelioCalibrations;
class HelioObjectRegistration;
class HelioPinHandlers;

#include "Helioduino.h"
#include "HelioPins.h"

// Calibrations Storage
// Stores user calibration data, which calibrates the various sensors output to
// an usable input value.
class HelioCalibrations {
public:
    // Adds/updates user calibration data to storage, returning success flag
    bool setUserCalibrationData(const HelioCalibrationData *calibrationData);

    // Drops/removes user calibration data from storage, returning success flag
    bool dropUserCalibrationData(const HelioCalibrationData *calibrationData);

    // Returns user calibration data instance in storage
    const HelioCalibrationData *getUserCalibrationData(hkey_t key) const;

    // Returns if there are any user calibrations in storage
    inline bool hasUserCalibrations() const { return _calibrationData.size(); };

protected:
    Map<hkey_t, HelioCalibrationData *, HELIO_CAL_CALIBS_MAXSIZE> _calibrationData; // Loaded user calibration data
};


// Object Registration Storage
// Stores objects in main system store, which is used for SharedPtr<> lookups as well as
// notifying appropriate modules upon entry-to/exit-from the system.
class HelioObjectRegistration {
public:
    // Adds object to system, returning success
    bool registerObject(SharedPtr<HelioObject> obj);
    // Removes object from system, returning success
    bool unregisterObject(SharedPtr<HelioObject> obj);

    // Searches for object by id key (nullptr return = no obj by that id, position index may use HELIO_POS_SEARCH* defines)
    SharedPtr<HelioObject> objectById(HelioIdentity id) const;

    // Finds first position either open or taken, given the id type
    hposi_t firstPosition(HelioIdentity id, bool taken);
    // Finds first position taken, given the id type
    inline hposi_t firstPositionTaken(HelioIdentity id) { return firstPosition(id, true); }
    // Finds first position open, given the id type
    inline hposi_t firstPositionOpen(HelioIdentity id) { return firstPosition(id, false); }

protected:
    Map<hkey_t, SharedPtr<HelioObject>, HELIO_SYS_OBJECTS_MAXSIZE> _objects; // Shared object collection, key'ed by HelioIdentity

    SharedPtr<HelioObject> objectById_Col(const HelioIdentity &id) const;
};


// Pin Handlers Storage
// Stores various pin-related system data on a shared pin # basis. Covers:
// - Pin locks: used for async shared resource management
// - Pin muxers: used for i/o pin multiplexing across a shared address bus
// - Pin expanders: used for i/o virtual pin expanding across an i2c interface
// - Pin OneWire: used for digital sensor pin's OneWire owner
class HelioPinHandlers {
public:
    // Attempts to get a lock on pin #, to prevent multi-device comm overlap (e.g. for OneWire comms).
    bool tryGetPinLock(pintype_t pin, millis_t wait = 150);
    // Returns a locked pin lock for the given pin. Only call if pin lock was successfully locked.
    inline void returnPinLock(pintype_t pin) { _pinLocks.erase(pin); }

    // Sets pin muxer for pin #.
    inline void setPinMuxer(pintype_t pin, SharedPtr<HelioPinMuxer> pinMuxer) { _pinMuxers[pin] = pinMuxer; }
    // Returns pin muxer for pin #.
    inline SharedPtr<HelioPinMuxer> getPinMuxer(pintype_t pin) { return _pinMuxers[pin]; }
    // Deactivates all pin muxers. Called before selecting another channel if pin muxers are assumed
    // to have a shared address bus (based on HELIO_MUXERS_SHARED_ADDR_BUS setting).
    void deactivatePinMuxers();

#ifndef HELIO_DISABLE_MULTITASKING

    // Sets pin expander for index.
    inline void setPinExpander(hposi_t index, SharedPtr<HelioPinExpander> pinExpander) { _pinExpanders[index] = pinExpander; }
    // Returns expander for index.
    inline SharedPtr<HelioPinExpander> getPinExpander(hposi_t index) { return _pinExpanders[index]; }

#endif // /ifndef HELIO_DISABLE_MULTITASKING

    // OneWire instance for given pin (lazily instantiated)
    OneWire *getOneWireForPin(pintype_t pin);
    // Drops OneWire instance for given pin (if created)
    void dropOneWireForPin(pintype_t pin);

protected:
    Map<pintype_t, OneWire *, HELIO_SYS_ONEWIRES_MAXSIZE> _pinOneWire; // Pin OneWire mapping
    Map<pintype_t, pintype_t, HELIO_SYS_PINLOCKS_MAXSIZE> _pinLocks; // Pin locks mapping (existence = locked)
    Map<pintype_t, SharedPtr<HelioPinMuxer>, HELIO_SYS_PINMUXERS_MAXSIZE> _pinMuxers; // Pin muxers mapping
#ifndef HELIO_DISABLE_MULTITASKING
    Map<hposi_t, SharedPtr<HelioPinExpander>, HELIO_SYS_PINEXPANDERS_MAXSIZE> _pinExpanders; // Pin expanders mapping
#endif
};

#endif // /ifndef HelioModules_H

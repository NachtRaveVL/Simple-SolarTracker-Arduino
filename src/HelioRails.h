/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Power Rails
*/

#ifndef HelioRails_H
#define HelioRails_H

class HelioRail;
class HelioSimpleRail;
class HelioRegulatedRail;

struct HelioRailData;
struct HelioSimpleRailData;
struct HelioRegulatedRailData;

#include "Helioduino.h"

// Creates rail object from passed rail data (return ownership transfer - user code *must* delete returned object)
extern HelioRail *newRailObjectFromData(const HelioRailData *dataIn);


// Power Rail Base
// This is the base class for all power rails, which defines how the rail is identified,
// where it lives, what's attached to it, and who can activate under it.
class HelioRail : public HelioObject,
                  public HelioRailObjectInterface,
                  public HelioPowerUnitsInterfaceStorage {
public:
    const enum : signed char { Simple, Regulated, Unknown = -1 } classType; // Power rail class (custom RTTI)
    inline bool isSimpleClass() const { return classType == Simple; }
    inline bool isRegulatedClass() const { return classType == Regulated; }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioRail(Helio_RailType railType,
              hposi_t railIndex,
              int classType = Unknown);
    HelioRail(const HelioRailData *dataIn);
    virtual ~HelioRail();

    virtual void update() override;

    virtual bool addLinkage(HelioObject *obj) override;
    virtual bool removeLinkage(HelioObject *obj) override;

    inline Helio_RailType getRailType() const { return _id.objTypeAs.railType; }
    inline hposi_t getRailIndex() const { return _id.posIndex; }
    inline float getRailVoltage() const { return getRailVoltageFromType(getRailType()); }

    Signal<HelioRail *, HELIO_RAIL_SIGNAL_SLOTS> &getCapacitySignal();

protected:
    Helio_TriggerState _limitState;                         // Limit state (last handled)

    Signal<HelioRail *, HELIO_RAIL_SIGNAL_SLOTS> _capacitySignal; // Capacity changed signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    void handleLimit(Helio_TriggerState limitState);
    friend class HelioRegulatedRail;
};

// Simple Power Rail
// Basic power rail that tracks # of devices turned on, with a limit to how many
// can be on at the same time. Crude, but effective, especially when all devices
// along the rail will use about the same amount of power anyways.
class HelioSimpleRail : public HelioRail {
public:
    HelioSimpleRail(Helio_RailType railType,
                    hposi_t railIndex,
                    int maxActiveAtOnce = 2,
                    int classType = Simple);
    HelioSimpleRail(const HelioSimpleRailData *dataIn);

    virtual bool canActivate(HelioActuator *actuator) override;
    virtual float getCapacity(bool poll = false) override;

    virtual void setPowerUnits(Helio_UnitsType powerUnits) override;

    inline int getActiveCount() { return _activeCount; }

protected:
    int _activeCount;                                       // Current active count
    int _maxActiveAtOnce;                                   // Max active count

    virtual void saveToData(HelioData *dataOut) override;

    void handleActivation(HelioActuator *actuator);
    friend class HelioRail;
};

// Regulated Power Rail
// Power rail that has a max power rating and power sensor that can track power
// usage, with limit trigger for over-power state limiting actuator activation.
class HelioRegulatedRail : public HelioRail,
                           public HelioPowerUsageSensorAttachmentInterface,
                           public HelioLimitTriggerAttachmentInterface {
public:
    HelioRegulatedRail(Helio_RailType railType,
                       hposi_t railIndex,
                       float maxPower,
                       int classType = Regulated);
    HelioRegulatedRail(const HelioRegulatedRailData *dataIn);

    virtual void update() override;
    virtual SharedPtr<HelioObjInterface> getSharedPtrFor(const HelioObjInterface *obj) const override;

    virtual bool canActivate(HelioActuator *actuator) override;
    virtual float getCapacity(bool poll = false) override;

    virtual void setPowerUnits(Helio_UnitsType powerUnits) override;

    virtual HelioSensorAttachment &getPowerUsageSensorAttachment() override;

    virtual HelioTriggerAttachment &getLimitTriggerAttachment() override;

    inline float getMaxPower() const { return _maxPower; }

protected:
    float _maxPower;                                        // Maximum power
    HelioSensorAttachment _powerUsage;                      // Power usage sensor attachment
    HelioTriggerAttachment _limitTrigger;                   // Power limit trigger attachment

    virtual void saveToData(HelioData *dataOut) override;

    void handleActivation(HelioActuator *actuator);
    friend class HelioRail;

    void handlePower(const HelioMeasurement *measurement);
};


// Rail Serialization Data
struct HelioRailData : public HelioObjectData
{
    Helio_UnitsType powerUnits;                             // Power units

    HelioRailData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Simple Rail Serialization Data
struct HelioSimpleRailData : public HelioRailData
{
    int maxActiveAtOnce;                                    // Max active count

    HelioSimpleRailData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Regulated Rail Serialization Data
struct HelioRegulatedRailData : public HelioRailData
{
    float maxPower;                                         // Maximum power
    char powerUsageSensor[HELIO_NAME_MAXSIZE];              // Power usage sensor
    HelioTriggerSubData limitTrigger;                       // Power limit trigger

    HelioRegulatedRailData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioRails_H

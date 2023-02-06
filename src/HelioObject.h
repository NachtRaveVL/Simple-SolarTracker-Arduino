/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Object
*/

#ifndef HelioObject_H
#define HelioObject_H

struct HelioIdentity;
class HelioObject;
class HelioSubObject;

struct HelioObjectData;

#include "Helioduino.h"
#include "HelioData.h"

// Creates object from passed object data (return ownership transfer - user code *must* delete returned object)
extern HelioObject *newObjectFromData(const HelioData *dataIn);

// Shortcut to get shared pointer for object with static pointer cast built-in.
template<class T = HelioObjInterface> inline SharedPtr<T> getSharedPtr(const HelioObjInterface *obj) { return obj ? reinterpret_pointer_cast<T>(obj->getSharedPtr()) : nullptr; }


// Object Identity
// Simple class for referencing an object in the Helio system.
// This class is mainly used to simplify object key generation, which is used when we
// want to uniquely refer to objects in the Helio system.
struct HelioIdentity {
    enum : signed char { Actuator, Sensor, Panel, Rail, Unknown = -1 } type; // Object type (custom RTTI)
    inline bool isActuatorType() const { return type == Actuator; }
    inline bool isSensorType() const { return type == Sensor; }
    inline bool isPanelType() const { return type == Panel; }
    inline bool isRailType() const { return type == Rail; }
    inline bool isUnknownType() const { return type <= Unknown; }

    union {
        Helio_ActuatorType actuatorType;                    // As actuator type enumeration
        Helio_SensorType sensorType;                        // As sensor type enumeration
        Helio_PanelType panelType;                          // As panel type enumeration
        Helio_RailType railType;                            // As rail type enumeration
    } objTypeAs;                                            // Enumeration type union
    hposi_t posIndex;                                       // Position index
    String keyString;                                       // String key
    hkey_t key;                                             // UInt Key

    // Default constructor (incomplete id)
    HelioIdentity();

    // Copy key (incomplete id)
    HelioIdentity(hkey_t key);

    // Copy into keyStr (incomplete id)
    HelioIdentity(const char *idKeyStr);
    // Copy into keyStr (incomplete id)
    HelioIdentity(String idKey);

    // Copy id with new position index
    HelioIdentity(const HelioIdentity &id,
                  hposi_t positionIndex);

    // Actuator id constructor
    HelioIdentity(Helio_ActuatorType actuatorType,
                  hposi_t positionIndex = HELIO_POS_SEARCH_FROMBEG);
    // Sensor id constructor
    HelioIdentity(Helio_SensorType sensorType,
                  hposi_t positionIndex = HELIO_POS_SEARCH_FROMBEG);
    // Panel id constructor
    HelioIdentity(Helio_PanelType panelType,
                  hposi_t positionIndex = HELIO_POS_SEARCH_FROMBEG);
    // Rail id constructor
    HelioIdentity(Helio_RailType railType,
                  hposi_t positionIndex = HELIO_POS_SEARCH_FROMBEG);

    // Data constructor
    HelioIdentity(const HelioData *dataIn);

    // Used to update key value after modification, returning new key by convenience
    hkey_t regenKey();

    inline operator bool() const { return key != (hkey_t)-1; }
    inline bool operator==(const HelioIdentity &otherId) const { return key == otherId.key; }
    inline bool operator!=(const HelioIdentity &otherId) const { return key != otherId.key; }
};


// Object Base
// A simple base class for referring to objects in the Helio system.
class HelioObject : public HelioObjInterface {
public:
    inline bool isActuatorType() const { return _id.isActuatorType(); }
    inline bool isSensorType() const { return _id.isSensorType(); }
    inline bool isPanelType() const { return _id.isPanelType(); }
    inline bool isRailType() const { return _id.isRailType(); }
    inline bool isUnknownType() const { return _id.isUnknownType(); }

    HelioObject(HelioIdentity id);                          // Standard constructor
    HelioObject(const HelioData *dataIn);                   // Data constructor
    virtual ~HelioObject();                                 // Destructor

    virtual void update();                                  // Called over intervals of time by runloop
    virtual void handleLowMemory();                         // Called upon low memory condition to try and free memory up

    HelioData *newSaveData();                               // Saves object state to proper backing data

    void allocateLinkages(size_t size = 1);                 // Allocates linkage list of specified size (reallocates)
    virtual bool addLinkage(HelioObject *obj) override;     // Adds linkage to this object, returns true upon initial add
    virtual bool removeLinkage(HelioObject *obj) override;  // Removes linkage from this object, returns true upon last remove
    bool hasLinkage(HelioObject *obj) const;                // Checks object linkage to this object

    // Returns the linkages this object contains, along with refcount for how many times it has registered itself as linked (via attachment points).
    // Objects are considered strong pointers, since existence -> SharedPtr ref to this instance exists.
    inline Pair<uint8_t, Pair<HelioObject *, int8_t> *> getLinkages() const { return make_pair(_linksSize, _links); }

    virtual HelioIdentity getId() const override;           // Returns the unique Identity of the object
    virtual hkey_t getKey() const override;          // Returns the unique key of the object
    virtual String getKeyString() const override;           // Returns the key string of the object
    virtual SharedPtr<HelioObjInterface> getSharedPtr() const override; // Returns the SharedPtr instance of the object

protected:
    HelioIdentity _id;                                      // Object id
    uint8_t _linksSize;                                     // Size of object linkages
    Pair<HelioObject *, int8_t> *_links;                    // Object linkages (owned, lazily allocated)

    virtual HelioData *allocateData() const;                // Only up to base type classes (sensor, crop, etc.) does this need overriden
    virtual void saveToData(HelioData *dataOut);            // *ALL* derived classes must override and implement

private:
    HelioObject() = default;                                // Private constructor to disable derived/public access
};


// Sub Object Base
// A base class for sub objects that are typically found embedded in bigger main objects,
// but want to replicate some of the same functionality. Not required to be inherited from.
class HelioSubObject : public HelioObjInterface {
public:
    virtual HelioIdentity getId() const override;
    virtual hkey_t getKey() const override;
    virtual String getKeyString() const override;
    virtual SharedPtr<HelioObjInterface> getSharedPtr() const override;

    virtual bool addLinkage(HelioObject *obj) override;
    virtual bool removeLinkage(HelioObject *obj) override;
};


// Object Serialization Data Intermediate
// Intermediate data class for object data.
struct HelioObjectData : public HelioData {
    char name[HELIO_NAME_MAXSIZE];

    HelioObjectData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioObject_H

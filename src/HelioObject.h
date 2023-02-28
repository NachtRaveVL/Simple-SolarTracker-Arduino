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
        hid_t idType;                                       // As standard id type enumeration
    } objTypeAs;                                            // Object type union
    hposi_t posIndex;                                       // Position index
    String keyString;                                       // String key
    hkey_t key;                                             // UInt Key

    // Default/copy key (incomplete id)
    inline HelioIdentity(hkey_t key = -1) : type(Unknown), objTypeAs{.idType=Unknown}, posIndex(-1), keyString(), key(key) { ; }
    // Copy into keyStr (incomplete id)
    inline HelioIdentity(const char *idKeyStr) : type(Unknown), objTypeAs{.idType=Unknown}, posIndex(-1), keyString(idKeyStr), key(stringHash(idKeyStr)) { ; }
    // Copy into keyStr (incomplete id)
    inline HelioIdentity(String idKey) : type(Unknown), objTypeAs{.idType=Unknown}, posIndex(-1), keyString(idKey), key(stringHash(idKey.c_str())) { ; }

    // Copy id with new position index
    inline HelioIdentity(const HelioIdentity &id, hposi_t positionIndex) : type(id.type), objTypeAs{.idType=id.objTypeAs.idType}, posIndex(positionIndex), keyString(), key(hkey_none) { regenKey(); }

    // Actuator id constructor
    inline HelioIdentity(Helio_ActuatorType actuatorTypeIn,
                         hposi_t positionIndex = HELIO_POS_SEARCH_FROMBEG) : type(Actuator), objTypeAs{.actuatorType=actuatorTypeIn}, posIndex(positionIndex), keyString(), key(hkey_none) { regenKey(); }
    // Sensor id constructor
    inline HelioIdentity(Helio_SensorType sensorTypeIn,
                         hposi_t positionIndex = HELIO_POS_SEARCH_FROMBEG) : type(Sensor), objTypeAs{.sensorType=sensorTypeIn}, posIndex(positionIndex), keyString(), key(hkey_none) { regenKey(); }
    // Panel id constructor
    inline HelioIdentity(Helio_PanelType panelTypeIn,
                         hposi_t positionIndex = HELIO_POS_SEARCH_FROMBEG) : type(Panel), objTypeAs{.panelType=panelTypeIn}, posIndex(positionIndex), keyString(), key(hkey_none) { regenKey(); }
    // Rail id constructor
    inline HelioIdentity(Helio_RailType railTypeIn,
                         hposi_t positionIndex = HELIO_POS_SEARCH_FROMBEG) : type(Rail), objTypeAs{.railType=railTypeIn}, posIndex(positionIndex), keyString(), key(hkey_none) { regenKey(); }

    // Data constructor
    inline HelioIdentity(const HelioData *dataIn) : type((typeof(type))(dataIn->id.object.idType)), objTypeAs{.idType=dataIn->id.object.objType}, posIndex(dataIn->id.object.posIndex), keyString(), key(hkey_none) { regenKey(); }

    // Used to update key value after modification, returning new key by convenience
    hkey_t regenKey();

    inline operator bool() const { return key != hkey_none; }
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

    inline HelioObject(HelioIdentity id) : _id(id), _revision(-1), _linksSize(0), _links(nullptr) { ; }
    inline HelioObject(const HelioData *data) : _id(data), _revision(data->_revision), _linksSize(0), _links(nullptr) { ; }
    virtual ~HelioObject();

    // Called over intervals of time by runloop
    virtual void update();
    // Called upon low memory condition to try and free memory up
    virtual void handleLowMemory();

    // Saves object state to proper backing data
    HelioData *newSaveData();

    // (Re)allocates linkage list of specified size
    void allocateLinkages(size_t size = 1);
    // Adds linkage to this object, returns true upon initial add
    virtual bool addLinkage(HelioObject *obj);
    // Removes linkage from this object, returns true upon last remove
    virtual bool removeLinkage(HelioObject *obj);
    // Checks object linkage to this object
    bool hasLinkage(HelioObject *obj) const;

    // Returns the linkages this object contains, along with refcount for how many times it has registered itself as linked (via attachment points).
    // Objects are considered strong pointers, since existence -> SharedPtr ref to this instance exists.
    inline Pair<uint8_t, Pair<HelioObject *, int8_t> *> getLinkages() const { return make_pair(_linksSize, _links); }

    // Unresolves any dlinks to obj prior to caching
    virtual void unresolveAny(HelioObject *obj) override;
    // Unresolves this instance from any dlinks
    inline void unresolve() { unresolveAny(this); }

    // Returns the unique Identity of the object
    virtual HelioIdentity getId() const override;
    // Returns the unique key of the object
    virtual hkey_t getKey() const override;
    // Returns the key string of the object
    virtual String getKeyString() const override;
    // Returns the SharedPtr instance for this object
    virtual SharedPtr<HelioObjInterface> getSharedPtr() const override;
    // Returns the SharedPtr instance for passed object
    virtual SharedPtr<HelioObjInterface> getSharedPtrFor(const HelioObjInterface *obj) const override;
    // Returns true for object
    virtual bool isObject() const override;

    // Returns revision #
    inline uint8_t getRevision() const { return abs(_revision); }
    // If revision has been modified since last saved
    inline bool isModified() const { return _revision < 0; }
    // Bumps revision # if not already modified, and sets modified flag (called after modifying data)
    inline void bumpRevisionIfNeeded() { if (!isModified()) { _revision = -(abs(_revision) + 1); } }
    // Unsets modified flag from revision (called after save-out)
    inline void unsetModified() { _revision = abs(_revision); }

protected:
    HelioIdentity _id;                                      // Object id
    int8_t _revision;                                       // Revision # of stored data (uses -vals for modified flag)
    uint8_t _linksSize;                                     // Number of object linkages
    Pair<HelioObject *, int8_t> *_links;                    // Object linkages array (owned, lazily allocated/grown/shrunk)

    virtual HelioData *allocateData() const;                // Only up to base type classes (sensor, panel, etc.) does this need overriden
    virtual void saveToData(HelioData *dataOut);            // *ALL* derived classes must override and implement

private:
    // Private constructor to disable derived/public access
    inline HelioObject() : _id(), _revision(-1), _linksSize(0), _links(nullptr) { ; }
};


// Sub Object Base
// A base class for sub objects that are typically found embedded in bigger main objects,
// but want to replicate some of the same functionality. Not required to be inherited from.
class HelioSubObject : public HelioObjInterface {
public:
    inline HelioSubObject(HelioObjInterface *parent = nullptr) : _parent(parent) { ; }

    virtual void setParent(HelioObjInterface *parent);
    inline HelioObjInterface *getParent() const { return _parent; }

    virtual void unresolveAny(HelioObject *obj) override;

    virtual HelioIdentity getId() const override;
    virtual hkey_t getKey() const override;
    virtual String getKeyString() const override;
    virtual SharedPtr<HelioObjInterface> getSharedPtr() const override;
    virtual SharedPtr<HelioObjInterface> getSharedPtrFor(const HelioObjInterface *obj) const override;
    virtual bool isObject() const override;

    inline uint8_t getRevision() const { return _parent && _parent->isObject() ? ((HelioObject *)_parent)->getRevision() : 0; }
    inline bool isModified() const { return _parent && _parent->isObject() ? ((HelioObject *)_parent)->isModified() : false; }
    inline void bumpRevisionIfNeeded() { if (_parent && _parent->isObject()) { ((HelioObject *)_parent)->bumpRevisionIfNeeded(); } }
    inline void unsetModified() { ; }

protected:
    HelioObjInterface *_parent;                             // Parent object pointer (reverse ownership)
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

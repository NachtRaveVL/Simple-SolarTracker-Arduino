/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Data
*/

#ifndef HelioData_H
#define HelioData_H

struct HelioData;
struct HelioSubData;

#include "Helioduino.h"

// Serializes Helio data structure to a binary output stream (essentially a memcpy), with optional skipBytes
extern size_t serializeDataToBinaryStream(const HelioData *data, Stream *streamOut, size_t skipBytes = sizeof(void*));
// Deserializes Helio data structure from a binary input stream (essentially a memcpy), with optional skipBytes
extern size_t deserializeDataFromBinaryStream(HelioData *data, Stream *streamIn, size_t skipBytes = sizeof(void*));

// Creates a new Helio data object corresponding to a binary input stream (return ownership transfer - user code *must* delete returned data)
extern HelioData *newDataFromBinaryStream(Stream *streamIn);
// Creates a new Helio data object corresponding to an input JSON element (return ownership transfer - user code *must* delete returned data)
extern HelioData *newDataFromJSONObject(JsonObjectConst &objectIn);


// Data Base
// Base class for serializable (JSON+Binary) storage data, used to define the base
// header of all data stored internally.
// NOTE: NON-CONST VALUE TYPES ONLY. All data *MUST* be able to use default operator=.
struct HelioData : public HelioJSONSerializableInterface {
    union {
        char chars[4];                                      // Standalone data structure 4-char identifier
        struct {
          int8_t idType;                                    // Object ID type enum value (e.g. actuator, sensor, etc.)
          int8_t objType;                                   // Object type enum value (e.g. actuatorType, sensorType, etc.)
          int8_t posIndex;                                  // Object position index # (zero-ordinal)
          int8_t classType;                                 // Object class type enum value (e.g. pump, dht1w, etc.)
        } object;
    } id;                                                   // Identifier union
    uint16_t _size;                                         // The size (in bytes) of the data
    uint8_t _version;                                       // Version # of data container
    uint8_t _revision;                                      // Revision # of stored data
    bool _modified;                                         // Flag tracking modified status (reset to false after save)

    inline bool isStandardData() const { return id.chars[0] == 'S'; }
    inline bool isSystemData() const { return id.chars[0] == 'S' && id.chars[1] == 'S' && id.chars[2] == 'Y' && id.chars[3] == 'S'; }
    inline bool isCalibrationData() const { return id.chars[0] == 'S' && id.chars[1] == 'C' && id.chars[2] == 'A' && id.chars[3] == 'L'; }
    inline bool isObjectData() const { return !isStandardData() && id.object.idType >= 0; }

    HelioData();                                            // Default constructor
    HelioData(char id0,                                     // 4-char identifier, index 0
              char id1,                                     // 4-char identifier, index 1
              char id2,                                     // 4-char identifier, index 2
              char id3,                                     // 4-char identifier, index 3
              uint8_t version = 1,                          // Data structure version #
              uint8_t revision = 1);                        // Stored data revision #
    HelioData(int8_t idType,                                // ID type enum value
              int8_t objType,                               // Object type enum value
              int8_t posIndex,                              // Object position index #
              int8_t classType,                             // Class type enum value
              uint8_t version = 1,                          // Data structure version #
              uint8_t revision = 1);                        // Stored data revision #
    HelioData(const HelioIdentity &id);                     // Identity constructor

    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;

    inline void _bumpRev() { _revision += 1; _setModded(); }
    inline void _bumpRevIfNotAlreadyModded() { if (!_modified) { _bumpRev(); } } // Should be called before modifying data
    inline void _setModded() { _modified = true; }          // Should be called after modifying any data
    inline void _unsetModded() { _modified = false; }       // Should be called after save-out
};


// Sub Data Base
// Sub-data exists inside of regular data for smaller objects that don't require the
// entire data object hierarchy, useful for triggers, measurements, etc.
// NOTE: NON-CONST VALUE TYPES ONLY, NO VIRTUALS. All data *MUST* be able to use default operator=.
struct HelioSubData {
    int8_t type;                                            // Sub data type (or -1 if unused).

    HelioSubData();
    HelioSubData(int8_t dataType);
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};


// Internal use, but must contain all ways for all data types to be new'ed
extern HelioData *_allocateDataFromBaseDecode(const HelioData &baseDecode);
extern HelioData *_allocateDataForObjType(int8_t idType, int8_t classType);

#endif // /ifndef HelioData_H

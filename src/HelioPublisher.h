
/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Publisher
*/

#ifndef HelioPublisher_H
#define HelioPublisher_H

class HelioPublisher;
struct HelioPublisherSubData;
struct HelioDataColumn;

#include "Helioduino.h"

// Data Publisher
// The Publisher allows for data collection and publishing capabilities. The data output
// is based on a simple table of time and measured value. Each time segment, called a
// polling frame (and controlled by the polling rate interval), collects data from all
// sensors into a data row, with the appropriate total number of columns. At time of
// either all sensors having reported in for their frame #, or the frame # proceeding
// to advance (in which case the existing value is recycled), the table's row is
// submitted to configured publishing services.
// Publishing to SD card .csv data files (via SPI card reader) is supported as is logging to
// WiFiStorage .csv data files (via OS/OTA filesystem / WiFiNINA_Generic only). MQTT is also
// supported but requires additional setup.
class HelioPublisher {
public:
    HelioPublisher();
    ~HelioPublisher();

    void update();
 
    bool beginPublishingToSDCard(String dataFilePrefix);
    inline bool isPublishingToSDCard() const;

#ifdef HELIO_USE_WIFI_STORAGE
    bool beginPublishingToWiFiStorage(String dataFilePrefix);
    inline bool isPublishingToWiFiStorage() const;
#endif

#ifdef HELIO_USE_MQTT
    bool beginPublishingToMQTTClient(MQTTClient &client);
    inline bool isPublishingToMQTTClient() const;
#endif

    void publishData(Helio_PositionIndex columnIndex, HelioSingleMeasurement measurement);

    inline void setNeedsTabulation();
    inline bool needsTabulation() { return _needsTabulation; }

    inline bool isPublishingEnabled() const;
    Helio_PositionIndex getColumnIndexStart(Helio_KeyType sensorKey);

    Signal<Pair<uint8_t, const HelioDataColumn *>, HELIO_PUBLISH_STATE_SLOTS> &getPublishSignal();

    void notifyDayChanged();

protected:
#if HELIO_SYS_LEAVE_FILES_OPEN
    File *_dataFileSD;                                      // SD card log file instance (owned)
#ifdef HELIO_USE_WIFI_STORAGE
    WiFiStorageFile *_dataFileWS;                           // WiFiStorageFile log file instance (owned)
#endif
#endif
#ifdef HELIO_USE_MQTT
    MQTTClient *_mqttClient;                                // MQTT client object (strong)
#endif
    String _dataFilename;                                   // Resolved data file name (based on day)
    Helio_PollingFrame _pollingFrame;                       // Polling frame that publishing is caught up to
    bool _needsTabulation;                                  // Needs tabulation tracking flag
    uint8_t _columnCount;                                   // Data columns count
    HelioDataColumn *_dataColumns;                          // Data columns (owned)

    Signal<Pair<uint8_t, const HelioDataColumn *>, HELIO_PUBLISH_STATE_SLOTS> _publishSignal; // Data publishing signal

    friend class Helioduino;

    inline HelioPublisherSubData *publisherData() const;
    inline bool hasPublisherData() const;

    void advancePollingFrame();
    friend void dataLoop();

    void checkCanPublish();
    void publish(time_t timestamp);

    void performTabulation();

    void resetDataFile();
    void cleanupOldestData(bool force = false);
};

// Publisher Data Column
// Data column worth of storage. Intended to be array allocated.
struct HelioDataColumn {
    Helio_KeyType sensorKey;                                // Key to sensor object
    HelioSingleMeasurement measurement;                     // Storage polling frame measurement
};


// Publisher Serialization Sub Data
// A part of HSYS system data.
struct HelioPublisherSubData : public HelioSubData {
    char dataFilePrefix[16];                                // Base data file name prefix / folder (default: "data/hy")
    bool pubToSDCard;                                       // If publishing sensor data to SD card is enabled (default: false)
    bool pubToWiFiStorage;                                  // If publishing sensor data to WiFiStorage is enabled (default: false)

    HelioPublisherSubData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioPublisher_H

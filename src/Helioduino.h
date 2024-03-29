/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    This permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.

    Simple-SolarTracker-Arduino - Version 0.6.8.0
*/

#ifndef Helioduino_H
#define Helioduino_H

// Library Setup

// NOTE: It is recommended to use custom build flags instead of editing this file directly.

// Uncomment or -D this define to completely disable usage of any multitasking commands and libraries. Not recommended.
//#define HELIO_DISABLE_MULTITASKING              // https://github.com/davetcc/TaskManagerIO

// Uncomment or -D this define to disable usage of tcMenu library, which will disable all GUI control. Not recommended.
//#define HELIO_DISABLE_GUI                       // https://github.com/davetcc/tcMenu

// Uncomment or -D this define to enable usage of the platform WiFi library, which enables networking capabilities.
//#define HELIO_ENABLE_WIFI                       // https://reference.arduino.cc/reference/en/libraries/wifi/

// Uncomment or -D this define to enable usage of the external serial AT WiFi library, which enables networking capabilities.
//#define HELIO_ENABLE_AT_WIFI                    // https://github.com/jandrassy/WiFiEspAT

// Uncomment or -D this define to enable usage of the platform Ethernet library, which enables networking capabilities.
//#define HELIO_ENABLE_ETHERNET                   // https://reference.arduino.cc/reference/en/libraries/ethernet/

// Uncomment or -D this define to enable usage of the Arduino MQTT library, which enables IoT data publishing capabilities.
//#define HELIO_ENABLE_MQTT                       // https://github.com/256dpi/arduino-mqtt

// Uncomment or -D this define to enable usage of the Adafruit GPS library, which enables GPS capabilities.
//#define HELIO_ENABLE_GPS                        // https://github.com/adafruit/Adafruit_GPS

// Uncomment or -D this define to enable external data storage (SD card or EEPROM) to save on sketch size. Required for constrained devices.
//#define HELIO_DISABLE_BUILTIN_DATA              // Disables library data existing in Flash, see DataWriter example for exporting details

// Uncomment or -D this define to enable debug output (treats Serial output as attached to serial monitor, waiting on start for connection).
//#define HELIO_ENABLE_DEBUG_OUTPUT

// Uncomment or -D this define to enable verbose debug output (note: adds considerable size to compiled sketch).
//#define HELIO_ENABLE_VERBOSE_DEBUG

// Uncomment or -D this define to enable debug assertions (note: adds significant size to compiled sketch).
//#define HELIO_ENABLE_DEBUG_ASSERTIONS


#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#ifdef NDEBUG
#ifdef HELIO_ENABLE_DEBUG_OUTPUT
#undef HELIO_ENABLE_DEBUG_OUTPUT
#endif
#ifdef HELIO_ENABLE_VERBOSE_DEBUG
#undef HELIO_ENABLE_VERBOSE_DEBUG
#endif
#ifdef HELIO_ENABLE_DEBUG_ASSERTIONS
#undef HELIO_ENABLE_DEBUG_ASSERTIONS
#endif
#endif // /ifdef NDEBUG

#if !defined(USE_SW_SERIAL)
typedef HardwareSerial SerialClass;
#else
#include <SoftwareSerial.h>             // https://www.arduino.cc/en/Reference/softwareSerial
#define HELIO_USE_SOFTWARE_SERIAL
typedef SoftwareSerial SerialClass;
#endif

#ifdef ESP32
typedef SDFileSystemClass SDClass;
#endif
#ifdef ESP8266
typedef SerialConfig uartmode_t;
#else
typedef int uartmode_t;
#endif

#ifdef HELIO_ENABLE_WIFI
#if defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>                    // https://github.com/arduino-libraries/WiFi101
#else
#include <WiFiNINA_Generic.h>           // https://github.com/khoih-prog/WiFiNINA_Generic
#define HELIO_USE_WIFI_STORAGE
#endif
#define HELIO_USE_WIFI
#define HELIO_USE_NET
#elif defined(HELIO_ENABLE_AT_WIFI)
#include "WiFiEspAT.h"                  // WiFi ESP AT library
#define HELIO_USE_AT_WIFI
#define HELIO_USE_WIFI
#define HELIO_USE_NET
#elif defined(HELIO_ENABLE_ETHERNET)
#include <Ethernet.h>                   // https://github.com/arduino-libraries/Ethernet
#define HELIO_USE_ETHERNET
#define HELIO_USE_NET
#endif // /ifdef HELIO_ENABLE_WIFI

#ifndef HELIO_DISABLE_MULTITASKING
#include "TaskManagerIO.h"              // Task Manager library
#include "IoAbstraction.h"              // IoAbstraction library
#define HELIO_USE_MULTITASKING
#else
#ifndef HELIO_DISABLE_GUI
#define HELIO_DISABLE_GUI
#endif
#define secondsToMillis(val) ((val)*1000U)
#if defined(ARDUINO_ARCH_MBED)
typedef uint32_t pintype_t;
#else
typedef uint8_t pintype_t;
#endif
#endif

#if defined(HELIO_ENABLE_DEBUG_OUTPUT) && defined(HELIO_ENABLE_VERBOSE_DEBUG)
#define HELIO_USE_VERBOSE_OUTPUT
#endif
#if defined(HELIO_ENABLE_DEBUG_OUTPUT) && defined(HELIO_ENABLE_DEBUG_ASSERTIONS)
#define HELIO_SOFT_ASSERT(cond,msg)     softAssert((bool)(cond), String((msg)), __FILE__, __func__, __LINE__)
#define HELIO_HARD_ASSERT(cond,msg)     hardAssert((bool)(cond), String((msg)), __FILE__, __func__, __LINE__)
#define HELIO_USE_DEBUG_ASSERTIONS
#else
#define HELIO_SOFT_ASSERT(cond,msg)     ((void)0)
#define HELIO_HARD_ASSERT(cond,msg)     ((void)0)
#endif

#ifdef HELIO_ENABLE_GPS
#include "Adafruit_GPS.h"               // GPS library
#define HELIO_USE_GPS
typedef Adafruit_GPS GPSClass;
#endif
#include "ArduinoJson.h"                // JSON library
#include "ArxContainer.h"               // STL-like container library
#include "ArxSmartPtr.h"                // Shared pointer library
#include "DHT.h"                        // DHT* air temp/humidity probe
#include "I2C_eeprom.h"                 // i2c EEPROM library
#ifdef HELIO_ENABLE_MQTT
#include "MQTT.h"                       // MQTT library
#define HELIO_USE_MQTT
#endif
#include "OneWire.h"                    // OneWire library
#include "RTClib.h"                     // i2c RTC library
#include "SolarCalculator.h"            // Solar calculator library
#include "TimeLib.h"                    // Time library
#ifndef HELIO_DISABLE_GUI
#include "tcMenu.h"                     // tcMenu library
#define HELIO_USE_GUI
#endif

#if !(defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SPI)) && (SPI_INTERFACES_COUNT > 0 || SPI_HOWMANY > 0)
#define HELIO_USE_SPI                   &SPI
#else
#define HELIO_USE_SPI                   nullptr
#endif
#if !(defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SPI1)) && (SPI_INTERFACES_COUNT > 1 || SPI_HOWMANY > 1)
#define HELIO_USE_SPI1                  &SPI1
#else
#define HELIO_USE_SPI1                  nullptr
#endif
#if !(defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_TWOWIRE)) && (WIRE_INTERFACES_COUNT > 0 || WIRE_HOWMANY > 0)
#define HELIO_USE_WIRE                  &Wire
#else
#define HELIO_USE_WIRE                  nullptr
#endif
#if !(defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_TWOWIRE1)) && (WIRE_INTERFACES_COUNT > 1 || WIRE_HOWMANY > 1)
#define HELIO_USE_WIRE1                 &Wire1
#else
#define HELIO_USE_WIRE1                 nullptr
#endif
#if !(defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SERIAL1)) && (SERIAL_HOWMANY > 1 || defined(HWSERIAL1) || defined(HAVE_HWSERIAL1) || defined(PIN_SERIAL1_RX) || defined(SERIAL2_RX) || defined(Serial1))
#define HELIO_USE_SERIAL1               &Serial1
#else
#define HELIO_USE_SERIAL1               nullptr
#endif

#include "HelioDefines.h"
#include "shared/HelioUIDefines.h"

#if ARX_HAVE_LIBSTDCPLUSPLUS >= 201103L // Have libstdc++11
#include "ArxSmartPtr/shared_ptr.h"     // Forced shared pointer library
using namespace std;
template<typename T, size_t N = HELIO_DEFAULT_MAXSIZE> using Vector = std::vector<T>;
template<class T1, class T2> using Pair = std::pair<T1,T2>;
template<typename K, typename V, size_t N = HELIO_DEFAULT_MAXSIZE> using Map = std::map<K,V>;
#else
using namespace arx;
template<typename T, size_t N = ARX_VECTOR_DEFAULT_SIZE> using Vector = arx::vector<T,N>;
template<class T1, class T2> using Pair = arx::pair<T1,T2>;
template<typename K, typename V, size_t N = ARX_MAP_DEFAULT_SIZE> using Map = arx::map<K,V,N>;
#endif
using namespace arx::stdx;
template <typename T> using SharedPtr = arx::stdx::shared_ptr<T>;

inline time_t unixNow();
inline DateTime localNow();
inline millis_t nzMillis();
extern void handleInterrupt(pintype_t);
extern hkey_t stringHash(String);
extern String addressToString(uintptr_t);
extern void controlLoop();
extern void dataLoop();
extern void miscLoop();

#include "HelioStrings.h"
#include "HelioInlines.hh"
#include "HelioCallback.hh"
#include "HelioInterfaces.h"
#include "HelioActivation.h"
#include "HelioAttachments.h"
#include "HelioData.h"
#include "HelioObject.h"
#include "HelioMeasurements.h"
#include "HelioPins.h"
#include "HelioUtils.h"
#include "HelioDatas.h"
#include "shared/HelioUIData.h"
#include "HelioStreams.h"
#include "HelioTriggers.h"
#include "HelioDrivers.h"
#include "HelioActuators.h"
#include "HelioSensors.h"
#include "HelioPanels.h"
#include "HelioRails.h"
#include "HelioModules.h"
#include "HelioScheduler.h"
#include "HelioLogger.h"
#include "HelioPublisher.h"
#include "HelioFactory.h"


// Helioduino Controller
// Main controller interface of the Helioduino solar tracker system.
class Helioduino : public HelioFactory, public HelioCalibrations, public HelioObjectRegistration, public HelioPinHandlers {
public:
    HelioScheduler scheduler;                                       // Scheduler public instance
    HelioLogger logger;                                             // Logger public instance
    HelioPublisher publisher;                                       // Publisher public instance

    // Controller constructor. Typically called during class instantiation, before setup().
    Helioduino(pintype_t piezoBuzzerPin = -1,                       // Piezo buzzer pin, else -1
               Helio_EEPROMType eepromType = Helio_EEPROMType_None, // EEPROM device type/size, else None
               DeviceSetup eepromSetup = DeviceSetup(),             // EEPROM device setup (i2c only)
               Helio_RTCType rtcType = Helio_RTCType_None,          // RTC device type, else None
               DeviceSetup rtcSetup = DeviceSetup(),                // RTC device setup (i2c only)
               DeviceSetup sdSetup = DeviceSetup(),                 // SD card device setup (spi only)
               DeviceSetup netSetup = DeviceSetup(),                // Network device setup (spi/uart)
               DeviceSetup gpsSetup = DeviceSetup(),                // GPS device setup (uart/i2c/spi)
               pintype_t *ctrlInputPins = nullptr,                  // Control input pins, else nullptr
               DeviceSetup displaySetup = DeviceSetup());           // Display device setup (i2c/spi)
    // Library destructor. Just in case.
    ~Helioduino();

    // Initializes default empty system. Typically called near top of setup().
    // See individual enums for more info.
    void init(Helio_SystemMode systemMode = Helio_SystemMode_Tracking,                  // What mode of panel orientation is performed
              Helio_MeasurementMode measureMode = Helio_MeasurementMode_Default,        // What units of measurement should be used
              Helio_DisplayOutputMode dispOutMode = Helio_DisplayOutputMode_Disabled,   // What display output mode should be used
              Helio_ControlInputMode ctrlInMode = Helio_ControlInputMode_Disabled);     // What control input mode should be used

    // Initializes system from EEPROM save, returning success flag
    // Set system data address with setSystemEEPROMAddress
    bool initFromEEPROM(bool jsonFormat = false);
    // Initializes system from SD card file save, returning success flag
    // Set config file name with setSystemConfigFilename
    bool initFromSDCard(bool jsonFormat = true);
#ifdef HELIO_USE_WIFI_STORAGE
    // Initializes system from a WiFiStorage file save, returning success flag
    // Set config file name with setSystemConfigFilename
    bool initFromWiFiStorage(bool jsonFormat = true);
#endif
    // Initializes system from custom JSON-based stream, returning success flag
    bool initFromJSONStream(Stream *streamIn);
    // Initializes system from custom binary stream, returning success flag
    bool initFromBinaryStream(Stream *streamIn);

    // Saves current system setup to EEPROM save, returning success flag
    // Set system data address with setSystemEEPROMAddress
    bool saveToEEPROM(bool jsonFormat = false);
    // Saves current system setup to SD card file save, returning success flag
    // Set config file name with setSystemConfigFilename
    bool saveToSDCard(bool jsonFormat = true);
#ifdef HELIO_USE_WIFI_STORAGE
    // Saves current system setup to WiFiStorage file save, returning success flag
    // Set config file name with setSystemConfigFilename
    bool saveToWiFiStorage(bool jsonFormat = true);
#endif
    // Saves current system setup to custom JSON-based stream, returning success flag
    bool saveToJSONStream(Stream *streamOut, bool compact = true);
    // Saves current system setup to custom binary stream, returning success flag
    bool saveToBinaryStream(Stream *streamOut);

    // System Operation.

    // Launches system into operational mode. Typically called near end of setup().
    void launch();

    // Suspends the system from operational mode (disables all run-loops). Typically used during system setup UI.
    // Resume operation by a call to launch().
    void suspend();

    // Update method. Typically called in loop().
    void update();

    // System Logging.

    // Enables system logging to the SD card. Log file names will append YYMMDD.txt to the specified prefix. Returns success flag.
    inline bool enableSysLoggingToSDCard(String logFilePrefix) { return logger.beginLoggingToSDCard(logFilePrefix); }
#ifdef HELIO_USE_WIFI_STORAGE
    // Enables system logging to WiFiStorage. Log file names will append YYMMDD.txt to the specified prefix. Returns success flag.
    inline bool enableSysLoggingToWiFiStorage(String logFilePrefix) { return logger.beginLoggingToWiFiStorage(logFilePrefix); }
#endif

    // Data Publishing.

    // Enables data publishing to the SD card. Data file names will append YYMMDD.csv to the specified prefix. Returns success flag.
    inline bool enableDataPublishingToSDCard(String dataFilePrefix) { return publisher.beginPublishingToSDCard(dataFilePrefix); }
#ifdef HELIO_USE_WIFI_STORAGE
    // Enables data publishing to WiFiStorage. Data file names will append YYMMDD.csv to the specified prefix. Returns success flag.
    inline bool enableDataPublishingToWiFiStorage(String dataFilePrefix) { return publisher.beginPublishingToWiFiStorage(dataFilePrefix); }
#endif
#ifdef HELIO_USE_MQTT
    // Enables data publishing to MQTT broker. Client is expected to be began/connected (with proper broker address/net client) *before* calling this method. Returns success flag.
    inline bool enableDataPublishingToMQTTClient(MQTTClient &client) { return publisher.beginPublishingToMQTTClient(client); }
#endif

    // User Interface.

#ifdef HELIO_USE_GUI
    // Enables UI to run with passed instance.
    // Minimal/RO UI only allows the user to edit existing objects, not create nor delete them.
    // Full/RW UI allows the user to add/remove system objects, customize features, change settings, etc.
    // Note: Be sure to manually include the appropriate UI system header file (e.g. #include "min/HelioduinoUI.h") in Arduino sketch.
    inline bool enableUI(HelioUIInterface *ui) { _activeUIInstance = ui; _uiData = ui->init(_uiData); return ui->begin(); }
#endif

    // Mutators.

    // Sets scheduler scheduling needed flag
    inline void setNeedsScheduling() { scheduler.setNeedsScheduling(); }
    // Sets publisher tabulation needed flag
    inline void setNeedsTabulation() { publisher.setNeedsTabulation(); }
    // Sets active UI redraw needed flag
    inline void setNeedsRedraw() {
        #ifdef HELIO_USE_GUI
            if (_activeUIInstance) { _activeUIInstance->setNeedsRedraw(); }
        #endif
    }

    // Sets display name of system (HELIO_NAME_MAXSIZE size limit)
    void setSystemName(String systemName);
    // Sets system time zone offset from UTC
    void setTimeZoneOffset(int8_t hoursOffset, int8_t minsOffset);
    // Sets system time zone offset from UTC, in standard hours
    inline void setTimeZoneOffset(int hoursOffset) { setTimeZoneOffset(hoursOffset, 0); }
    // Sets system time zone offset from UTC, in fractional hours
    inline void setTimeZoneOffset(float hoursOffset) { setTimeZoneOffset((int8_t)hoursOffset, (fabsf(hoursOffset) - floorf(fabsf(hoursOffset))) * signbit(hoursOffset) ? -60.0f : 60.0f); }
    // Sets system polling interval, in milliseconds (does not enable polling, see enable publishing methods)
    void setPollingInterval(uint16_t pollingInterval);
    // Sets system autosave enable mode and optional fallback mode and interval, in minutes.
    void setAutosaveEnabled(Helio_Autosave autosaveEnabled, Helio_Autosave autosaveFallback = Helio_Autosave_Disabled, uint16_t autosaveInterval = HELIO_SYS_AUTOSAVE_INTERVAL);
    // Sets system config file as used in init and save by SD card.
    inline void setSystemConfigFilename(String configFilename) { _sysConfigFilename = configFilename; }
    // Sets EEPROM system data address as used in init and save by EEPROM.
    inline void setSystemDataAddress(uint16_t sysDataAddress) { _sysDataAddress = sysDataAddress; }
    // Sets the RTC's time to the passed time, with respect to set timezone. Will trigger significant time event.
    void setRTCTime(DateTime time);
#ifdef HELIO_USE_WIFI
    // Sets WiFi connection's SSID/pass combo (note: password is stored encrypted, but is not hack-proof)
    void setWiFiConnection(String ssid, String pass);
#endif
#ifdef HELIO_USE_ETHERNET
    // Sets Ethernet connection's MAC address
    void setEthernetConnection(const uint8_t *macAddress);
#endif
    // Sets system location (lat/long/alt, note: only triggers update if significant or forced)
    void setSystemLocation(double latitude, double longitude, double altitude = DBL_UNDEF, bool isSigChange = false);

    // Accessors.

    // EEPROM device size, in bytes (default: 0)
    inline uint32_t getEEPROMSize() const { return _eepromType != Helio_EEPROMType_None ? (((int)_eepromType) << 7) : 0; }
    // EEPROM device setup configuration
    inline const DeviceSetup &getEEPROMSetup() const { return _eepromSetup; }
    // RTC device setup configuration
    inline const DeviceSetup &getRTCSetup() const { return _rtcSetup; }
    // SD card device setup configuration
    inline const DeviceSetup &getSDCardSetup() const { return _sdSetup; }
#ifdef HELIO_USE_NET
    // Network device setup configuration
    inline const DeviceSetup &getNetworkSetup() const { return _netSetup; }
#endif
#ifdef HELIO_USE_GPS
    // GPS device setup configuration
    inline const DeviceSetup &getGPSSetup() const { return _gpsSetup; }
#endif
#ifdef HELIO_USE_GUI
    // LCD output device setup configuration
    inline const DeviceSetup &getDisplaySetup() const { return _displaySetup; }
    // Returns control input pins ribbon
    Pair<uint8_t, const pintype_t *> getControlInputPins() const;
#endif

    // EEPROM instance (lazily instantiated, nullptr return -> failure/no device)
    I2C_eeprom *getEEPROM(bool begin = true);
    // Real time clock instance (lazily instantiated, nullptr return -> failure/no device)
    HelioRTCInterface *getRTC(bool begin = true);
    // SD card instance (user code *must* call endSDCard(inst) to return interface, possibly lazily instantiated, nullptr return -> failure/no device)
    SDClass *getSDCard(bool begin = true);
    // Ends SD card transaction with proper regards to platform once all instances returned (note: some instancing may be expected to never return)
    void endSDCard(SDClass *sd = nullptr);
#ifdef HELIO_USE_WIFI
    // WiFi instance (nullptr return -> failure/no device, note: this method may block for up to a minute)
    inline WiFiClass *getWiFi(bool begin = true);
    // WiFi instance with fallback ssid/pass combo (nullptr return -> failure/no device, note: this method may block for up to a minute)
    WiFiClass *getWiFi(String ssid, String pass, bool begin = true);
#endif
#ifdef HELIO_USE_ETHERNET
    // Ethernet instance (nullptr return -> failure/no device, note: this method may block for up to a minute)
    inline EthernetClass *getEthernet(bool begin = true);
    // Ethernet instance with fallback MAC address (nullptr return -> failure/no device, note: this method may block for up to a minute)
    EthernetClass *getEthernet(const uint8_t *macAddress, bool begin = true);
#endif
#ifdef HELIO_USE_GPS
    // GPS instance (nullptr return -> failure/no device)
    GPSClass *getGPS(bool begin = true);
#endif

    // Whenever the system is in operational mode (has been launched), or not
    inline bool inOperationalMode() const { return !_suspend; }
    // System type mode (default: Tracking)
    Helio_SystemMode getSystemMode() const;
    // System measurement mode (default: Metric)
    Helio_MeasurementMode getMeasurementMode() const;
    // System LCD output mode (default: Disabled)
    Helio_DisplayOutputMode getDisplayOutputMode() const;
    // System control input mode (default: Disabled)
    Helio_ControlInputMode getControlInputMode() const;
    // System display name (default: "Helioduino")
    String getSystemName() const;
    // System display name (default: "Helioduino"), as constant chars
    inline const char *getSystemNameChars() const { return _systemData ? _systemData->systemName : nullptr; }
    // System time zone offset from UTC (default: +0/UTC), in total offset seconds
    time_t getTimeZoneOffset() const;
    // Whenever the system booted up with the RTC battery failure flag set (meaning the time is not set correctly)
    inline bool getRTCBatteryFailure() const { return _rtcBattFail; }
    // System sensor polling interval (time between sensor reads), in milliseconds (default: HELIO_DATA_LOOP_INTERVAL)
    uint16_t getPollingInterval() const;
    // System polling frame number for sensor frame tracking
    inline hframe_t getPollingFrame() const { return _pollingFrame; }
    // Determines if a given frame # is out of date (true) or current (false), with optional frame # allowance
    bool isPollingFrameOld(hframe_t frame, hframe_t allowance = 0) const;
    // Returns if system autosaves are enabled or not
    bool isAutosaveEnabled() const;
    // Returns if system fallback autosaves are enabled or not
    bool isAutosaveFallbackEnabled() const;
    // System config file used in init and save by SD card
    inline String getSystemConfigFile() const { return _sysConfigFilename; }
    // System data address used in init and save by EEPROM
    inline uint16_t getSystemDataAddress() const { return _sysDataAddress; }
#ifdef HELIO_USE_WIFI
    // SSID for WiFi connection
    String getWiFiSSID() const;
    // Password for WiFi connection (plaintext)
    String getWiFiPassword() const;
#endif
#ifdef HELIO_USE_ETHERNET
    // MAC address for Ethernet connection
    const uint8_t *getMACAddress() const;
#endif
    // System location (lat/long/alt)
    Location getSystemLocation() const;

    // Misc.

    // Called to notify system when RTC time is updated (also clears RTC battery failure flag)
    void notifyRTCTimeUpdated();

    // Called by scheduler to announce that date conditions have changed (significant time event)
    void notifyDayChanged();

protected:
    static Helioduino *_activeInstance;                     // Current active instance (set after init, weak)
#ifdef HELIO_USE_GUI
    HelioUIInterface *_activeUIInstance;                    // Current active UI instance (owned)
    HelioUIData *_uiData;                                   // UI data (owned)
#endif
    HelioSystemData *_systemData;                           // System data (owned, saved to storage)

    const pintype_t _piezoBuzzerPin;                        // Piezo buzzer pin (default: Disabled)
    const Helio_EEPROMType _eepromType;                     // EEPROM device type
    const DeviceSetup _eepromSetup;                         // EEPROM device setup
    const Helio_RTCType _rtcType;                           // RTC device type
    const DeviceSetup _rtcSetup;                            // RTC device setup
    const DeviceSetup _sdSetup;                             // SD card device setup
#ifdef HELIO_USE_NET
    const DeviceSetup _netSetup;                            // Network device setup
#endif
#ifdef HELIO_USE_GPS
    const DeviceSetup _gpsSetup;                            // GPS device setup
#endif
#ifdef HELIO_USE_GUI
    const pintype_t *_ctrlInputPins;                        // Control input pin mapping (weak, default: Disabled/nullptr)
    const DeviceSetup _displaySetup;                        // Display device setup
#endif

    I2C_eeprom *_eeprom;                                    // EEPROM instance (owned, lazy)
    HelioRTCInterface *_rtc;                                // Real time clock instance (owned, lazy)
    SDClass *_sd;                                           // SD card instance (owned/strong, lazy/supplied, default: SD)
    int8_t _sdOut;                                          // Number of SD card instances out
#ifdef HELIO_USE_GPS
    GPSClass *_gps;                                         // GPS instance (owned, lazy)
#endif

    bool _eepromBegan;                                      // Status of EEPROM begin() call
    bool _rtcBegan;                                         // Status of RTC begin() call
    bool _rtcBattFail;                                      // Status of RTC battery failure flag
    bool _sdBegan;                                          // Status of SD begin() call
#ifdef HELIO_USE_NET
    bool _netBegan;                                         // Status of WiFi/Ethernet begin() call
#endif
#ifdef HELIO_USE_GPS
    bool _gpsBegan;                                         // Status of GPS begin() call
#endif

#ifdef HELIO_USE_MULTITASKING
    taskid_t _controlTaskId;                                // Control task Id if created, else TASKMGR_INVALIDID
    taskid_t _dataTaskId;                                   // Data polling task Id if created, else TASKMGR_INVALIDID
    taskid_t _miscTaskId;                                   // Misc task Id if created, else TASKMGR_INVALIDID
#endif
    bool _suspend;                                          // If system is currently suspended from operation
    hframe_t _pollingFrame;                                 // Current data polling frame # (index 0 reserved for disabled/undef, advanced by publisher)
    time_t _lastSpaceCheck;                                 // Last date storage media free space was checked, if able (UTC)
    time_t _lastAutosave;                                   // Last date autosave was performed, if able (UTC)
    String _sysConfigFilename;                              // System config filename used in serialization (default: "Helioduino.cfg")
    uint16_t _sysDataAddress;                               // EEPROM system data address used in serialization (default: -1/disabled)

    void allocateEEPROM();
    void deallocateEEPROM();
    void allocateRTC();
    void deallocateRTC();
    void allocateSD();
    void deallocateSD();
#ifdef HELIO_USE_GPS
    void allocateGPS();
    void deallocateGPS();
#endif

    void commonPreInit();
    void commonPostInit();
    void commonPostSave();

    friend SharedPtr<HelioObjInterface> HelioDLinkObject::resolveObject();
    friend void controlLoop();
    friend void dataLoop();
    friend void miscLoop();
    friend void handleInterrupt(pintype_t pin);

    void checkFreeMemory();
    void broadcastLowMemory();
    void checkFreeSpace();
    void checkAutosave();

    friend Helioduino *::getController();
    friend HelioScheduler *::getScheduler();
    friend HelioLogger *::getLogger();
    friend HelioPublisher *::getPublisher();
#ifdef HELIO_USE_GUI
    friend HelioUIInterface *::getUI();
#endif
    friend class HelioScheduler;
    friend class HelioLogger;
    friend class HelioPublisher;
};

// Template implementations
#include "HelioInterfaces.hpp"
#include "Helioduino.hpp"
#include "HelioAttachments.hpp"
#include "HelioUtils.hpp"

#endif // /ifndef Helioduino_H

// Simple-SolarTracking-Arduino Dual-Axis (DA) Tracking Example
//
// The Dual-Axis Tracking Example sketch is the standard implementation. It can be
// easily extended to include other functionality if desired.

#ifdef USE_SW_SERIAL
#include "SoftwareSerial.h"
SoftwareSerial SWSerial(RX, TX);                        // Replace with Rx/Tx pins of your choice
#define Serial1 SWSerial
#endif

#include <Helioduino.h>

/// Pins & Class Instances
#define SETUP_PIEZO_BUZZER_PIN          -1              // Piezo buzzer pin, else -1
#define SETUP_EEPROM_DEVICE_TYPE        None            // EEPROM device type/size (24LC01, 24LC02, 24LC04, 24LC08, 24LC16, 24LC32, 24LC64, 24LC128, 24LC256, 24LC512, None)
#define SETUP_EEPROM_I2C_ADDR           0b000           // EEPROM i2c address
#define SETUP_RTC_I2C_ADDR              0b000           // RTC i2c address (only 0b000 can be used atm)
#define SETUP_RTC_DEVICE_TYPE           None            // RTC device type (DS1307, DS3231, PCF8523, PCF8563, None)
#define SETUP_SD_CARD_SPI               SPI             // SD card SPI class instance
#define SETUP_SD_CARD_SPI_CS            -1              // SD card CS pin, else -1
#define SETUP_SD_CARD_SPI_SPEED         F_SPD           // SD card SPI speed, in Hz (ignored on Teensy)
#define SETUP_LCD_I2C_ADDR              0b000           // LCD i2c address
#define SETUP_CTRL_INPUT_PINS           {(pintype_t)-1} // Control input pin ribbon, else {-1}
#define SETUP_I2C_WIRE                  Wire            // I2C wire class instance
#define SETUP_I2C_SPEED                 400000U         // I2C speed, in Hz
#define SETUP_ESP_I2C_SDA               SDA             // I2C SDA pin, if on ESP
#define SETUP_ESP_I2C_SCL               SCL             // I2C SCL pin, if on ESP

// WiFi Settings                                        (note: define HELIO_ENABLE_WIFI or HELIO_ENABLE_AT_WIFI to enable WiFi)
// #include "secrets.h"                                 // Pro-tip: Put sensitive password information into a custom secrets.h
#define SETUP_WIFI_SSID                 "CHANGE_ME"     // WiFi SSID
#define SETUP_WIFI_PASS                 "CHANGE_ME"     // WiFi passphrase
#define SETUP_WIFI_SPI                  SPIWIFI         // WiFi SPI class instance, if using spi
#define SETUP_WIFI_SPI_CS               SPIWIFI_SS      // WiFi CS pin, if using spi
#define SETUP_WIFI_SERIAL               Serial1         // WiFi serial class instance, if using serial

// Ethernet Settings                                    (note: define HELIO_ENABLE_ETHERNET to enable Ethernet)
#define SETUP_ETHERNET_MAC              { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED } // Ethernet MAC address
#define SETUP_ETHERNET_SPI              SPI1            // Ethernet SPI class instance
#define SETUP_ETHERNET_SPI_CS           SS1             // Ethernet CS pin

// GPS Settings                                         (note: define HELIO_ENABLE_GPS to enable GPS)
#define SETUP_GPS_TYPE                  None            // Type of GPS (Serial, I2C, SPI, None)
#define SETUP_GPS_SERIAL                Serial1         // GPS serial class instance, if using serial
#define SETUP_GPS_I2C_ADDR              0b000           // GPS i2c address, if using i2c
#define SETUP_GPS_SPI                   SPI             // GPS SPI class instance, if using spi
#define SETUP_GPS_SPI_CS                SS              // GPS CS pin, if using spi

// System Settings
#define SETUP_SYSTEM_MODE               Tracking        // System run mode (Tracking, Balancing)
#define SETUP_MEASURE_MODE              Default         // System measurement mode (Default, Imperial, Metric, Scientific)
#define SETUP_LCD_OUT_MODE              Disabled        // System LCD output mode (Disabled, 20x4LCD, 20x4LCD_Swapped, 16x2LCD, 16x2LCD_Swapped)
#define SETUP_CTRL_IN_MODE              Disabled        // System control input mode (Disabled, 2x2Matrix, 4xButton, 6xButton, RotaryEncoder)
#define SETUP_SYS_UI_MODE               Minimal         // System user interface mode (Disabled, Minimal, Full)
#define SETUP_SYS_NAME                  "Helioduino"    // System name
#define SETUP_SYS_TIMEZONE              +0              // System timezone offset
#define SETUP_SYS_LOGLEVEL              All             // System log level filter (All, Warnings, Errors, None)
#define SETUP_SYS_STATIC_LAT            DBL_UNDEF       // System static latitude (if not using GPS, else DBL_UNDEF), in degrees
#define SETUP_SYS_STATIC_LONG           DBL_UNDEF       // System static longitude (if not using GPS, else DBL_UNDEF), in minutes
#define SETUP_SYS_STATIC_ALT            DBL_UNDEF       // System static altitude (if not using GPS, else DBL_UNDEF), in meters above sea level (msl)

// System Saves Settings                                (note: only one primary and one fallback mechanism may be enabled at a time)
#define SETUP_SAVES_CONFIG_FILE         "helioduino.cfg" // System config file name for system saves
#define SETUP_SAVES_SD_CARD_MODE        Disabled        // If saving/loading from SD card is enable (Primary, Fallback, Disabled)
#define SETUP_SAVES_EEPROM_MODE         Disabled        // If saving/loading from EEPROM is enabled (Primary, Fallback, Disabled)
#define SETUP_SAVES_WIFISTORAGE_MODE    Disabled        // If saving/loading from WiFiStorage (OS/OTA filesystem / WiFiNINA_Generic only) is enabled (Primary, Fallback, Disabled)

// Logging & Data Publishing Settings
#define SETUP_LOG_FILE_PREFIX           "logs/he"       // System logs file prefix (appended with YYMMDD.txt)
#define SETUP_DATA_FILE_PREFIX          "data/he"       // System data publishing files prefix (appended with YYMMDD.csv)
#define SETUP_DATA_SD_ENABLE            false           // If system data publishing is enabled to SD card
#define SETUP_LOG_SD_ENABLE             false           // If system logging is enabled to SD card
#define SETUP_DATA_WIFISTORAGE_ENABLE   false           // If system data publishing is enabled to WiFiStorage (OS/OTA filesystem / WiFiNINA_Generic only)
#define SETUP_LOG_WIFISTORAGE_ENABLE    false           // If system logging is enabled to WiFiStorage (OS/OTA filesystem / WiFiNINA_Generic only)

// MQTT Settings                                        (note: define HELIO_ENABLE_MQTT to enable MQTT)
#define SETUP_MQTT_BROKER_CONNECT_BY    Hostname        // Which style of address broker uses (Hostname, IPAddress)
#define SETUP_MQTT_BROKER_HOSTNAME      "hostname"      // Hostname that MQTT broker exists at
#define SETUP_MQTT_BROKER_IPADDR        { 192, 168, 1, 2 } // IP address that MQTT broker exists at
#define SETUP_MQTT_BROKER_PORT          1883            // Port number that MQTT broker exists at

// External Data Settings
#define SETUP_EXTDATA_SD_ENABLE         false           // If data should be read from an external SD card (searched first for panels lib data)
#define SETUP_EXTDATA_SD_LIB_PREFIX     "lib/"          // Library data folder/data file prefix (appended with {type}##.dat)
#define SETUP_EXTDATA_EEPROM_ENABLE     false           // If data should be read from an external EEPROM (searched first for strings data)

// External EEPROM Settings
#define SETUP_EEPROM_SYSDATA_ADDR       0x2e50          // System data memory offset for EEPROM saves (from Data Writer output)
#define SETUP_EEPROM_STRINGS_ADDR       0x0000          // Start address for strings data (from Data Writer output)

// Device Setup
#define SETUP_AC_USAGE_SENSOR_PIN       -1              // AC power usage meter sensor pin (analog), else -1
#define SETUP_DC_USAGE_SENSOR_PIN       -1              // DC power usage meter sensor pin (analog), else -1
#define SETUP_DC_GEN_SENSOR_PIN         -1              // DC power generation meter sensor pin (analog), else -1
#define SETUP_DHT_AIR_TEMP_HUMID_PIN    -1              // DHT* air temp sensor data pin (digital), else -1
#define SETUP_DHT_SENSOR_TYPE           None            // DHT sensor type enum (DHT11, DHT12, DHT21, DHT22, AM2301, None)
#define SETUP_ICE_INDICATOR_PIN         -1              // Ice indicator pin (digital), else -1
#define SETUP_LIN_ACT1_AXIS1_PINA       -1              // Vertical axis linear actuator #1 pin A (digital), else -1
#define SETUP_LIN_ACT1_AXIS1_PINB       -1              // Vertical axis linear actuator #1 pin B (digital), else -1
#define SETUP_LIN_ACT1_MIN_ENDSTOP_PIN  -1              // Vertical axis linear actuator #1 min endstop pin (digital), else -1
#define SETUP_LIN_ACT1_MAX_ENDSTOP_PIN  -1              // Vertical axis linear actuator #1 max endstop pin (digital), else -1
#define SETUP_LIN_ACT2_AXIS1_PINA       -1              // Vertical axis linear actuator #2 pin A (digital), else -1
#define SETUP_LIN_ACT2_AXIS1_PINB       -1              // Vertical axis linear actuator #2 pin B (digital), else -1
#define SETUP_LIN_ACT2_MIN_ENDSTOP_PIN  -1              // Vertical axis linear actuator #2 min endstop pin (digital), else -1
#define SETUP_LIN_ACT2_MAX_ENDSTOP_PIN  -1              // Vertical axis linear actuator #2 max endstop pin (digital), else -1
#define SETUP_PANEL_BRAKE_PIN           -1              // Panel brake relay pin (digital), else -1
#define SETUP_PANEL_HEATER_PIN          -1              // Panel heater relay pin (digital), else -1
#define SETUP_PANEL_SPRAYER_PIN         -1              // Panel sprayer relay pin (digital), else -1
#define SETUP_PANEL_TILT_SENSOR_PIN     -1              // Panel tilt angle sensor pin (analog), else -1
#define SETUP_POS_SERVO_AXIS0_PIN       -1              // Horizontal axis positional servo pin (analog), else -1
#define SETUP_POS_SERVO_AXIS1_PIN       -1              // Vertical axis positional servo pin (analog), else -1
#define SETUP_WIND_SPEED_SENSOR_PIN     -1              // Wind speed sensor pin (analog), else -1

// Base Setup
#define SETUP_AC_POWER_RAIL_TYPE        AC110V          // Rail power type used for actuator AC rail (AC110V, AC220V)
#define SETUP_DC_POWER_RAIL_TYPE        DC12V           // Rail power type used for actuator DC rail (DC3V3, DC5V, DC12V, DC24V, DC48V)
#define SETUP_AC_SUPPLY_POWER           0               // Maximum AC supply power wattage, else 0 if not known (-> use simple rails)
#define SETUP_DC_SUPPLY_POWER           0               // Maximum DC supply power wattage, else 0 if not known (-> use simple rails)

// Panel Setup
#define SETUP_PANEL_TYPE                Gimballed       // Panel type (Horizontal, Vertical, Gimballed, Equatorial)
#define SETUP_PANEL_HOME                {0.0f,0.0f}     // Panel home position (azi,ele or RA,dec)
#define SETUP_PANEL_OFFSET              {0.0f,0.0f}     // Panel offset position (azi,ele or RA,dec)

#if defined(HELIO_USE_WIFI)
WiFiClient netClient;
#elif defined(HELIO_USE_ETHERNET)
EthernetClient netClient;
#endif
#ifdef HELIO_USE_MQTT
MQTTClient mqttClient;
#endif

#if defined(HELIO_USE_GUI) && SETUP_LCD_OUT_MODE != Disabled
#if SETUP_SYS_UI_MODE == Minimal
#include "min/HelioduinoUI.h"
typedef HelioduinoMinUI HelioduinoUI;
#elif SETUP_SYS_UI_MODE == Full
#include "full/HelioduinoUI.h"
typedef HelioduinoFullUI HelioduinoUI;
#endif
#endif

// Pre-init checks
#if (SETUP_SAVES_WIFISTORAGE_MODE != Disabled || SETUP_DATA_WIFISTORAGE_ENABLE || SETUP_LOG_WIFISTORAGE_ENABLE) && !defined(HELIO_USE_WIFI_STORAGE)
#warning The HELIO_ENABLE_WIFI flag is expected to be defined as well as WiFiNINA_Generic.h included in order to run this sketch with WiFiStorage features enabled
#endif
#if (SETUP_SAVES_SD_CARD_MODE != Disabled || SETUP_DATA_SD_ENABLE || SETUP_LOG_SD_ENABLE || SETUP_EXTDATA_SD_ENABLE) && SETUP_SD_CARD_SPI_CS == -1
#warning The SETUP_SD_CARD_SPI_CS define is expected to be set to a valid pin in order to run this sketch with SD card features enabled
#endif
#if (SETUP_SAVES_EEPROM_MODE != Disabled || SETUP_EXTDATA_EEPROM_ENABLE) && SETUP_EEPROM_DEVICE_SIZE == 0
#warning The SETUP_EEPROM_DEVICE_SIZE define is expected to be set to a valid size in order to run this sketch with EEPROM features enabled
#endif

pintype_t _SETUP_CTRL_INPUT_PINS[] = SETUP_CTRL_INPUT_PINS;
Helioduino helioController((pintype_t)SETUP_PIEZO_BUZZER_PIN,
                           JOIN(Helio_EEPROMType,SETUP_EEPROM_DEVICE_TYPE),
                           I2CDeviceSetup((uint8_t)SETUP_EEPROM_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
                           JOIN(Helio_RTCType,SETUP_RTC_DEVICE_TYPE),
                           I2CDeviceSetup((uint8_t)SETUP_RTC_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
                           SPIDeviceSetup((pintype_t)SETUP_SD_CARD_SPI_CS, &SETUP_SD_CARD_SPI, SETUP_SD_CARD_SPI_SPEED),
#if defined(HELIO_USE_AT_WIFI)
                           UARTDeviceSetup(&SETUP_WIFI_SERIAL, HELIO_SYS_ATWIFI_SERIALBAUD),
#elif defined(HELIO_USE_WIFI)
                           SPIDeviceSetup((pintype_t)SETUP_WIFI_SPI_CS, &SETUP_WIFI_SPI),
#elif defined(HELIO_USE_ETHERNET)
                           SPIDeviceSetup((pintype_t)SETUP_ETHERNET_SPI_CS, &SETUP_ETHERNET_SPI),
#else
                           DeviceSetup(),
#endif
#if defined(HELIO_USE_GPS) && SETUP_GPS_TYPE == Serial
                           UARTDeviceSetup(&SETUP_GPS_SERIAL, HELIO_SYS_NMEAGPS_SERIALBAUD),
#elif defined(HELIO_USE_GPS) && SETUP_GPS_TYPE == I2C
                           I2CDeviceSetup(SETUP_GPS_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
#elif defined(HELIO_USE_GPS) && SETUP_GPS_TYPE == SPI
                           SPIDeviceSetup(SETUP_GPS_SPI_CS, &SETUP_GPS_SPI),
#else
                           DeviceSetup(),
#endif
                           _SETUP_CTRL_INPUT_PINS,
                           I2CDeviceSetup((uint8_t)SETUP_LCD_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED));

#if SETUP_TODO_PIN >= 0
#define SETUP_USE_AC_RAIL
#endif
#if SETUP_TODO_PIN >= 0
#define SETUP_USE_DC_RAIL
#endif
#if defined(ADC_RESOLUTION)
#define SETUP_USE_ANALOG_BITRES     ADC_RESOLUTION
#else
#define SETUP_USE_ANALOG_BITRES     10
#endif
#define SETUP_USE_ONEWIRE_BITRES    12

inline void setupOnce()
{
    helioController.setSystemName(F(SETUP_SYS_NAME));
    helioController.setTimeZoneOffset(SETUP_SYS_TIMEZONE);
    #ifdef HELIO_USE_WIFI
    {   String wifiSSID = F(SETUP_WIFI_SSID);
        String wifiPassword = F(SETUP_WIFI_PASS);
        helioController.setWiFiConnection(wifiSSID, wifiPassword);
    }
    #endif
    #ifdef HELIO_USE_ETHERNET
    {   uint8_t _SETUP_ETHERNET_MAC[] = SETUP_ETHERNET_MAC;
        helioController.setEthernetConnection(_SETUP_ETHERNET_MAC);
    }
    #endif
    getLogger()->setLogLevel(JOIN(Helio_LogLevel,SETUP_SYS_LOGLEVEL));
    #if !defined(HELIO_USE_GPS)
        helioController.setSystemLocation(SETUP_SYS_STATIC_LAT, SETUP_SYS_STATIC_LONG, SETUP_SYS_STATIC_ALT);
    #endif
    #if defined(HELIO_USE_WIFI_STORAGE) && SETUP_SAVES_WIFISTORAGE_MODE == Primary
        helioController.setAutosaveEnabled(Helio_Autosave_EnabledToWiFiStorageJson
    #elif SETUP_SD_CARD_SPI_CS >= 0 && SETUP_SAVES_SD_CARD_MODE == Primary
        helioController.setAutosaveEnabled(Helio_Autosave_EnabledToSDCardJson
    #elif SETUP_EEPROM_DEVICE_SIZE && SETUP_SAVES_EEPROM_MODE == Primary
        helioController.setAutosaveEnabled(Helio_Autosave_EnabledToEEPROMRaw
    #else
        helioController.setAutosaveEnabled(Helio_Autosave_Disabled
    #endif
    #if defined(HELIO_USE_WIFI_STORAGE) && SETUP_SAVES_WIFISTORAGE_MODE == Fallback
        , Helio_Autosave_EnabledToWiFiStorageJson);
    #elif SETUP_SD_CARD_SPI_CS >= 0 && SETUP_SAVES_SD_CARD_MODE == Fallback
        , Helio_Autosave_EnabledToSDCardJson);
    #elif SETUP_EEPROM_DEVICE_SIZE && SETUP_SAVES_EEPROM_MODE == Fallback
        , Helio_Autosave_EnabledToEEPROMRaw);
    #else
        );
    #endif
}

inline void setupAlways()
{
    #if SETUP_LOG_SD_ENABLE
        helioController.enableSysLoggingToSDCard(F(SETUP_LOG_FILE_PREFIX));
    #endif
    #if SETUP_DATA_SD_ENABLE
        helioController.enableDataPublishingToSDCard(F(SETUP_DATA_FILE_PREFIX));
    #endif
    #if defined(HELIO_USE_WIFI_STORAGE) && SETUP_LOG_WIFISTORAGE_ENABLE
        helioController.enableSysLoggingToWiFiStorage(F(SETUP_LOG_FILE_PREFIX));
    #endif
    #if defined(HELIO_USE_WIFI_STORAGE) && SETUP_DATA_WIFISTORAGE_ENABLE
        helioController.enableDataPublishingToWiFiStorage(F(SETUP_DATA_FILE_PREFIX));
    #endif
    #ifdef HELIO_USE_MQTT
        bool netBegan = false;
        #if defined(HELIO_USE_WIFI)
            netBegan = helioController.getWiFi();
        #elif defined(HELIO_USE_ETHERNET)
            netBegan = helioController.getEthernet();
        #endif
        if (netBegan) {
            #if SETUP_MQTT_BROKER_CONNECT_BY == Hostname
                mqttClient.begin(String(F(SETUP_MQTT_BROKER_HOSTNAME)).c_str(), SETUP_MQTT_BROKER_PORT, netClient);
            #elif SETUP_MQTT_BROKER_CONNECT_BY == IPAddress
            {   uint8_t ipAddr[4] = SETUP_MQTT_BROKER_IPADDR;
                IPAddress ip(ipAddr[0], ipAddr[1], ipAddr[2], ipAddr[3]);
                mqttClient.begin(ip, SETUP_MQTT_BROKER_PORT, netClient);
            }
            #endif
            helioController.enableDataPublishingToMQTTClient(mqttClient);
        }
    #endif
    #ifdef HELIO_USE_GPS
        helioController.getGPS()->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    #endif
}

inline void setupObjects()
{
    // Base Objects
    #ifdef SETUP_USE_AC_RAIL
        #if SETUP_AC_SUPPLY_POWER
            auto acRelayPower = helioController.addRegulatedPowerRail(JOIN(Helio_RailType,SETUP_AC_POWER_RAIL_TYPE),SETUP_AC_SUPPLY_POWER);
            #if SETUP_AC_USAGE_SENSOR_PIN >= 0
            {   auto powerMeter = helioController.addPowerLevelMeter(SETUP_AC_USAGE_SENSOR_PIN, SETUP_USE_ANALOG_BITRES);
                acRelayPower->setPowerSensor(powerMeter);
            }
            #endif
        #else
            auto acRelayPower = helioController.addSimplePowerRail(JOIN(Helio_RailType,SETUP_AC_POWER_RAIL_TYPE));
        #endif
    #endif
    #ifdef SETUP_USE_DC_RAIL
        #if SETUP_DC_SUPPLY_POWER
            auto dcRelayPower = helioController.addRegulatedPowerRail(JOIN(Helio_RailType,SETUP_DC_POWER_RAIL_TYPE),SETUP_DC_SUPPLY_POWER);
            #if SETUP_DC_USAGE_SENSOR_PIN >= 0
            {   auto powerMeter = helioController.addPowerLevelMeter(SETUP_DC_USAGE_SENSOR_PIN, SETUP_USE_ANALOG_BITRES);
                dcRelayPower->setPowerSensor(powerMeter);
            }
            #endif
        #else
            auto dcRelayPower = helioController.addSimplePowerRail(JOIN(Helio_RailType,SETUP_DC_POWER_RAIL_TYPE));
        #endif
    #endif
    auto panel = helioController.addSolarTrackingPanel(JOIN(Helio_PanelType,SETUP_PANEL_TYPE));
    {   float _SETUP_PANEL_HOME[] = SETUP_PANEL_HOME;
        float _SETUP_PANEL_OFFSET[] = SETUP_PANEL_OFFSET;
        panel->setHomePosition(_SETUP_PANEL_HOME);
        panel->setAxisOffset(_SETUP_PANEL_OFFSET);
    }

    // Analog Sensors
    // todo
    // #if SETUP_PH_METER_PIN >= 0
    // {   auto phMeter = helioController.addAnalogPhMeter(SETUP_PH_METER_PIN, SETUP_USE_ANALOG_BITRES);
    //     phMeter->setParentPanel(feedReservoir);
    //     feedReservoir->setWaterPHSensor(phMeter);
    // }
    // #endif

    // Digital Sensors
    #if SETUP_DHT_AIR_TEMP_HUMID_PIN >= 0
    {   auto dhtTemperatureSensor = helioController.addDHTTempHumiditySensor(SETUP_DHT_AIR_TEMP_HUMID_PIN, JOIN(Helio_DHTType,SETUP_DHT_SENSOR_TYPE));
        panel->setTemperatureSensor(dhtTemperatureSensor);
    }
    #endif

    // Binary->Distance Sensors
    // todo
    // #if SETUP_VOL_EMPTY_PIN >= 0
    // {   auto emptyIndicator = helioController.addLevelIndicator(SETUP_VOL_EMPTY_PIN);
    //     emptyIndicator->setParentPanel(feedReservoir);
    //     feedReservoir->setEmptyTrigger(new HelioMeasurementValueTrigger(emptyIndicator, 0.5, SETUP_VOL_INDICATOR_TYPE));
    // }
    // #endif

    // Distance->Angle 
    // todo
    // #if SETUP_VOL_LEVEL_PIN >= 0
    //     #if SETUP_VOL_LEVEL_TYPE == Ultrasonic
    //     {   auto distanceSensor = helioController.addUltrasonicDistanceSensor(SETUP_VOL_LEVEL_PIN, SETUP_USE_ANALOG_BITRES);
    //         distanceSensor->setParentPanel(feedReservoir);
    //         feedReservoir->setWaterVolumeSensor(distanceSensor);
    //         #if SETUP_VOL_FILLED_PIN < 0
    //             feedReservoir->setFilledTrigger(new HelioMeasurementValueTrigger(distanceSensor, HELIO_FEEDRES_FRACTION_FILLED, ACTIVE_ABOVE));
    //         #endif
    //         #if SETUP_VOL_EMPTY_PIN < 0
    //             feedReservoir->setEmptyTrigger(new HelioMeasurementValueTrigger(distanceSensor, HELIO_FEEDRES_FRACTION_EMPTY, ACTIVE_BELOW));
    //         #endif
    //     }
    //     #elif SETUP_VOL_LEVEL_TYPE == AnalogHeight
    //     {   auto heightMeter = helioController.addAnalogWaterHeightMeter(SETUP_VOL_LEVEL_PIN, SETUP_USE_ANALOG_BITRES);
    //         heightMeter->setParentPanel(feedReservoir);
    //         feedReservoir->setWaterVolumeSensor(heightMeter);
    //         #if SETUP_VOL_FILLED_PIN < 0
    //             feedReservoir->setFilledTrigger(new HelioMeasurementValueTrigger(heightMeter, HELIO_FEEDRES_FRACTION_FILLED, ACTIVE_ABOVE));
    //         #endif
    //         #if SETUP_VOL_EMPTY_PIN < 0
    //             feedReservoir->setEmptyTrigger(new HelioMeasurementValueTrigger(heightMeter, HELIO_FEEDRES_FRACTION_EMPTY, ACTIVE_BELOW));
    //         #endif
    //     }
    //     #endif
    // #endif

    // AC-Based Actuators
    // todo

    // DC-Based Actuators
    // todo
}

void setup() {
    // Setup base interfaces
    #ifdef HELIO_ENABLE_DEBUG_OUTPUT
        Serial.begin(115200);           // Begin USB Serial interface
        while (!Serial) { ; }           // Wait for USB Serial to connect
    #endif
    #if defined(ESP_PLATFORM)
        SETUP_I2C_WIRE.begin(SETUP_ESP_I2C_SDA, SETUP_ESP_I2C_SCL); // Begin i2c Wire for ESP
    #endif

    // Begin external data storage devices for panel, strings, and other data.
    #if SETUP_EXTDATA_EEPROM_ENABLE
        beginStringsFromEEPROM(SETUP_EEPROM_STRINGS_ADDR);
    #endif
    #if SETUP_EXTDATA_SD_ENABLE
        beginStringsFromSDCard(String(F(SETUP_EXTDATA_SD_LIB_PREFIX)));
    #endif

    // Sets system config name used in any of the following inits.
    #if (defined(HELIO_USE_WIFI_STORAGE) && SETUP_SAVES_WIFISTORAGE_MODE != Disabled) || \
        (SETUP_SD_CARD_SPI_CS >= 0 && SETUP_SAVES_SD_CARD_MODE != Disabled)
        helioController.setSystemConfigFilename(F(SETUP_SAVES_CONFIG_FILE));
    #endif
    // Sets the EEPROM memory address for system data.
    #if SETUP_EEPROM_DEVICE_SIZE && SETUP_SAVES_EEPROM_MODE != Disabled
        helioController.setSystemDataAddress(SETUP_EEPROM_SYSDATA_ADDR);
    #endif

    // Initializes controller with first initialization method that successfully returns.
    if (!(false
        #if defined(HELIO_USE_WIFI_STORAGE) && SETUP_SAVES_WIFISTORAGE_MODE == Primary
            || helioController.initFromWiFiStorage()
        #elif SETUP_SD_CARD_SPI_CS >= 0 && SETUP_SAVES_SD_CARD_MODE == Primary
            || helioController.initFromSDCard()
        #elif SETUP_EEPROM_DEVICE_SIZE && SETUP_SAVES_EEPROM_MODE == Primary
            || helioController.initFromEEPROM()
        #endif
        #if defined(HELIO_USE_WIFI_STORAGE) && SETUP_SAVES_WIFISTORAGE_MODE == Fallback
            || helioController.initFromWiFiStorage()
        #elif SETUP_SD_CARD_SPI_CS >= 0 && SETUP_SAVES_SD_CARD_MODE == Fallback
            || helioController.initFromSDCard()
        #elif SETUP_EEPROM_DEVICE_SIZE && SETUP_SAVES_EEPROM_MODE == Fallback
            || helioController.initFromEEPROM()
        #endif
        )) {
        // First time running controller, set up default initial empty environment.
        helioController.init(JOIN(Helio_SystemMode,SETUP_SYSTEM_MODE),
                             JOIN(Helio_MeasurementMode,SETUP_MEASURE_MODE),
                             JOIN(Helio_DisplayOutputMode,SETUP_LCD_OUT_MODE),
                             JOIN(Helio_ControlInputMode,SETUP_CTRL_IN_MODE));

        setupOnce();
        setupAlways();
        setupObjects();
    } else {
        setupAlways();
    }

    #if defined(HELIO_USE_GUI) && SETUP_LCD_OUT_MODE != Disabled && SETUP_SYS_UI_MODE != Disabled
        helioController.enableUI(new HelioduinoUI());
    #endif

    // Launches controller into main operation.
    helioController.launch();
}

void loop()
{
    // Helioduino will manage most updates for us.
    helioController.update();
}

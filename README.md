# Helioduino
Helioduino: Simple Solar Tracker Automation Controller.

**Simple-SolarTracker-Arduino v0.6**

Simple automation controller for solar tracking systems.  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, Jan 3rd, 2023.

**UNDER ACTIVE DEVELOPMENT -- WORK IN PROGRESS**

This controller allows one to set up a system of panels, servos, LDRs, relays, and other objects useful in controlling both single and double axis sun tracking solar panel systems, and provides data monitoring & collection abilities while operating panel axis servos and/or linear actuators across the day as the sun moves to maintain optimal panel alignment. Works with a large variety of widely-available aquarium/hobbyist equipment, including popular GPS, RTC, EEPROM, SD card, WiFi, and other modules compatible with Arduino. Can be setup to calculate sun position accurately as possible or to auto-balance two opposing photoresistors per panel axis. With the right setup Helioduino can automatically do things like: drive large panels with linear actuators, use power sensing to auto-optimize daily panel offset, spray/wipe panels on routine to keep panels clean, deploy/retract panels at sunrise/sunset, or even provide panel heating during cold temperatures or when ice is detected.

Can be used with GPS and RTC modules for accurate sun angle measurements and sunrise/sunset timings, or through enabled WiFi/Ethernet from on-board or external ESP8266 WiFi module. Configured system can be saved/loaded to/from external EEPROM, SD card, or WiFiStorage-like device in JSON or binary, along with auto-save, recovery, and cleanup functionality, and can even use a piezo buzzer for audible system alerts. Actuator and sensor I/O pins can be multiplexed for pin-limited environments. Library data can be built into onboard Flash or exported alongside system/user data onto external storage. Supports sensor data and system event logging/publishing to external storage and MQTT, and can be extended to work with other JSON based Web APIs or Client-like derivatives. UI support pending, but will include system setup/configuration and monitoring abilities with basic LCD support via LiquidCrystal, or with advanced LCD and input controller support similar in operation to low-cost 3D printers [via tcMenu](https://github.com/davetcc/tcMenu).

Made primarily for Arduino microcontrollers / build environments, but should work with PlatformIO, Espressif, Teensy, STM32, Pico, and others - although one might experience turbulence until the bug reports get ironed out.

Dependencies include: Adafruit BusIO (dep of RTClib), Adafruit GPS Library (ext NMEA, optional), Adafruit Unified Sensor (dep of DHT), ArduinoJson, ArxContainer, ArxSmartPtr, DHT sensor library, I2C_EEPROM, IoAbstraction (dep of TaskManager), LiquidCrystalIO (dep of TaskManager), OneWire, RTClib, SimpleCollections (dep of TaskManager), SD, SolarCalculator, TaskManagerIO (disableable, dep of tcMenu), tcMenu (disableable), Time, and a WiFi-like library (optional): WiFi101 (MKR1000), WiFiNINA_Generic, WiFiEspAT (ext serial AT), or Ethernet.

Datasheet links include: [Generic LDR information](https://components101.com/resistors/ldr-datasheet), [Generic linear actuator information](https://arduinogetstarted.com/tutorials/arduino-actuator), [DHT12 Air Temperature and Humidity Sensor](https://github.com/NachtRaveVL/Simple-SolarTracker-Arduino/blob/main/extra/dht12.pdf), but many more are available online.

*If you value the work that we do, our small team always appreciates a subscription to our [Patreon](www.patreon.com/nachtrave).*

## About

We want to make solar trackers more accessible to DIY'ers by utilizing the widely-available low-cost IoT and IoT-like microcontrollers (MCUs) of today.

With the advances in miniaturization technology bringing us even more compact MCUs at even lower costs, it becomes a lot more possible to simply use one of these small devices to do what amounts to putting a few servos in the correct offsets as the day goes by. Solar tracking is a perfect application for these devices, especially as a data logger, process monitor, and more. Professional controller systems like this can cost hundreds to even thousands of dollars, but DIY systems can wind up being a fraction of that cost.

Helioduino is a MCU-based solution primarily written for Arduino and Arduino-like MCU devices. It allows one to throw together a bunch of hobbyist servos and relays, some solar panels, maybe some light dependent resistors (LDRs), and other widely available low-cost hardware to build a functional DIY solar tracking controller system. Be it made with PVC from the hardware store or 3D printed at home, Helioduino opens the door for more people to get involved in reducing their carbon footprint, becoming more knowledgeable about their power and where it comes from - and hey, hopefully learning some basic electronics/coding along the way.

## Controller Setup

### Requirements

Minimum MCU: 256-512kB Flash, 16-24kB SRAM, 8-16MHz  
Recommended: 1MB+ Flash, 32kB+ SRAM, 32MHz+

Will work: Nano 33 (any), MKR (any), Due/Zero, ESP32/8266, Teensy 3+, STM32 (properly sized), Pico, etc.

Won't work: Uno (any), Nano (classic & Every), Leonardo/Duemilanove, Micro, Pro, Esplora, Teensy 2-, STM8, etc.

Devices that _may_ work, but only with heavy tweaking/limited build: ATMega2560, Genuino 101, STM32 (improperly sized)

Note: Certain MCUs, such as those from STM, are sold in many different Flash/SRAM size configurations. Some configurations may not be supported, others may limit total system size (i.e. object count, library support, features, etc.). Bigger is always better until you get a better idea of your specific use case's size requirements.

### Installation

The easiest way to install this controller is to utilize the Arduino IDE library manager, or through a package manager such as PlatformIO. Otherwise, simply download this controller and extract its files into a `Simple-SolarTracker-Arduino` folder in your Arduino custom libraries folder, typically found in your `[My ]Documents\Arduino\libraries` folder (Windows), or `~/Documents/Arduino/libraries/` folder (Linux/OSX).

From there, you can make a local copy of one of the examples based on the kind of system setup you want to use. If you are unsure of which, we recommend using the (TODO) Example for older MCUs and the Full System Example for modern MCUs. Older storage constrained MCUs may need further modifications (and possibly external hardware) so is recommended only for advanced users.

### Header Defines

There are several defines inside of the controller's main header file that allow for more fine-tuned control of the controller. You may edit and uncomment these lines directly, or supply them via custom build flags. While editing the main header file isn't ideal, it is often easiest. Note that editing the controller's main header file directly will affect all projects compiled on your system using those modified controller files.

Alternatively, you may also refer to <https://forum.arduino.cc/index.php?topic=602603.0> on how to define custom build flags manually via modifying the platform[.local].txt file. Note that editing such directly will affect all other projects compiled on your system using those modified platform framework files, but at least you keep those changes to the same place.

From Helioduino.h:
```Arduino
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
//#define HELIO_DISABLE_BUILTIN_DATA              // Disables library data existing in Flash, instead relying solely on external storage.

// Uncomment or -D this define to enable debug output (treats Serial output as attached to serial monitor).
//#define HELIO_ENABLE_DEBUG_OUTPUT

// Uncomment or -D this define to enable verbose debug output (note: adds considerable size to compiled sketch).
//#define HELIO_ENABLE_VERBOSE_DEBUG

// Uncomment or -D this define to enable debug assertions (note: adds significant size to compiled sketch).
//#define HELIO_ENABLE_DEBUG_ASSERTIONS
```

### Controller Initialization

There are several initialization mode settings exposed through this controller that are used for more fine-tuned control.

#### Class Instantiation

The controller's class object must first be instantiated, commonly at the top of the sketch where pin setups are defined (or exposed through some other mechanism), which makes a call to the controller's class constructor. The constructor allows one to set the module's various devices and how they are connected, with defaults providing no device specified.

From Helioduino.h, in class Helioduino:
```Arduino
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
               DeviceSetup lcdSetup = DeviceSetup());               // LCD device setup (i2c only)
```

#### Controller Initialization

Additionally, a call is expected to be provided to the controller class object's `init[From…](…)` method, commonly called inside of the sketch's `setup()` function. This allows one to set the controller's system type (TODO), units of measurement (Metric, Imperial, or Scientific), control input mode, and display output mode. The default mode of the controller, if left unspecified, is a position calculating system set to Metric units, without any input control or output display.

From Helioduino.h, in class Helioduino:
```Arduino
    // Initializes default empty system. Typically called near top of setup().
    // See individual enums for more info.
    void init(Helio_SystemMode systemMode = Helio_SystemMode_PositionCalculating,       // What mode of panel orientation is performed
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
```

The controller can also be initialized from a saved configuration, such as from an EEPROM or SD Card, or other JSON or Binary stream. A saved configuration of the system can be made via the controller class object's `saveTo…(…)` methods, or called automatically on timer by setting an Autosave mode/interval.

From Helioduino.h, in class Helioduino:
```Arduino
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
```

### Event Logging & Data Publishing

The controller can, after initialization, be set to produce logs and data files that can be further used by other applications. Log entries are timestamped and can keep track of when offsets are performed, when voltage spikes/drops, etc., while data files can be read into plotting applications or exported to a database for further processing. The passed file prefix is typically the subfolder that such files should reside under and is appended with the year, month, and date (in YYMMDD format).

Note: You can also get the same logging output sent to the Serial device by defining `HELIO_ENABLE_DEBUG_OUTPUT`, described above in Header Defines.

Note: Files on FAT32-based SD cards are limited to 8 character file/folder names and a 3 character extension.

From Helioduino.h, in class Helioduino:
```Arduino
    // Enables data logging to the SD card. Log file names will append YYMMDD.txt to the specified prefix. Returns success flag.
    inline bool enableSysLoggingToSDCard(String logFilePrefix = "logs/hy");

    // Enables data publishing to the SD card. Log file names will append YYMMDD.csv to the specified prefix. Returns success flag.
    inline bool enableDataPublishingToSDCard(String dataFilePrefix = "data/hy");
```

## Hookup Callouts

* The recommended Vcc power supply and logic level is 5v, with most newer MCUs restricted to 3.3v.
  * There are many devices that are 3.3v only and not 5v tolerant. Check your IC's datasheet for details.
* Devices that do not have the same logic level voltage as the MCU will need a level converter (or similar), bi-directional particularly on any fast data lines, in order to operate together (unless capable of doing so without such).
  * Alternatively, using a 10kΩ resistor can often times be enough to 'convert' 5v to 3.3v, but the correct way is to use a 1kΩ resistor and a 2kΩ resistor (or any size with a 1:2 ratio) in a [simple voltage divider circuit](https://randomnerdtutorials.com/how-to-level-shift-5v-to-3-3v/).

### Serial UART

Serial UART uses individual communication lines for each device, with the receive `RX` pin of one being the transmit `TX` pin of the other - thus having to "flip wires" when connecting. However, devices can always be active and never have to share their access. UART runs at low to mid kHz speeds and is useful for simple device control, albeit somewhat clumsy at times.

* When wiring up modules that use Serial UART, make sure to flip `RX`/`TX` lines.
  * 5v devices interacting with 3.3v devices that are not 5v tolerant (such as [serial ESP WiFi modules](http://www.instructables.com/id/Cheap-Arduino-WiFi-Shield-With-ESP8266/)) will require a bi-directional logic level converter/shifter to utilize.
    * Alternatively, hack a single 10kΩ resistor ([but preferably two of any 1:2 ratio](https://randomnerdtutorials.com/how-to-level-shift-5v-to-3-3v/)) between the 5v module's TX pin and 3.3v module's RX pin.

Serial UART Devices Supported: AT WiFi modules, NMEA GPS modules

### SPI Bus

SPI devices can be chained together on the same shared data lines, which are typically labeled `COPI` (or `MOSI`), `CIPO` (or `MISO`), and `SCK`, often with an additional `CS` (or `SS`). Each SPI device requires its own individual cable-select `CS` wire as only one SPI device may be active at any given time - accomplished by pulling its `CS` line of that device low (aka active-low). SPI runs at MHz speeds and is useful for large data block transfers.

* The `CS` pin may be connected to any digital output pin, but it's common to use the `CS` (or `SS`) pin for the first device. Additional devices are not restricted to what pin they can or should use, but given it's not a data pin not using a choice interrupt-capable pin allows those to be used for interrupt driven mechanisms.
* Many low-cost SPI-based SD card modules on market only read SDHC sized SD cards (2GB to 32GB) formatted in FAT32 (filenames limited to 8 characters plus 3 character file extension).
  * Some SD cards simply will not play nicely with these modules and you may have to try another SD card manufacturer. We recommend 32GB SD cards due to overall lowest cost (5~10 $USD/SD card - smaller SD cards actually becoming _more_ expensive).

SPI Devices Supported: SD card modules, NMEA GPS modules

### I2C Bus

I2C (aka I²C, IIC, TwoWire, TWI) devices can be chained together on the same shared data lines (no flipping of wires), which are typically labeled `SCL` and `SDA`. Only different kinds of I2C devices can be used on the same data line together using factory default settings, otherwise manual addressing must be done. I2C runs at mid to high kHz speeds and is useful for advanced device control.

* When more than one I2C device of the same kind is to be used on the same data line, each device must be set to use a different address. This is accomplished via the A0-A2 (sometimes A0-A5) pins/pads on the physical device that must be set either open or closed (typically via a de-solderable resistor, or by shorting a pin/pad). Check your specific breakout's datasheet for details.
* Note that not all the I2C libraries used support multi-addressable I2C devices at this time (read as: may only use one). Currently, this restriction applies to: RTC devices.

I2C Devices Supported: DS*/PCF* RTC modules, AT24C* EEPROM modules, NMEA GPS modules, 16x2/20x4 LCD modules (TODO)

### OneWire Bus

OneWire devices can be chained together on the same shared data lines (no flipping of wires). Devices can be of the same or different types, require minimal setup (and no soldering), and most can even operate in "parasite" power mode where they use the power from the data line (and an internal capacitor) to function (thus saving a `Vcc` line, only requiring `Data` and `GND`). OneWire runs only in the low kb/s speeds and is useful for digital sensors.

* Typically, sensors are limited to 20 devices along a maximum 100m of wire.
* When more than one OneWire device is on the same data data line, each device registers itself an enumeration index (0 - N) along with its own 64-bit unique identifier (UUID, with last byte being CRC). The device can then be referenced via this UUID by the system in the future indefinitely, or enumeration index so long as the device doesn't change its line position.

OneWire Devices Supported: DHT* Temp modules

### Analog IO

* All analog sensors will need to have the same operational voltage range. Many analog sensors are set to use 0v to 5v by default, but some can go -5v to +5v, some even up to 5.5v.
* The `AREF` pin, by default, is the same voltage as the MCU. Analog sensors must not exceed this voltage limit.
  * 5v analog sensor signals **must** be [level converted](https://randomnerdtutorials.com/how-to-level-shift-5v-to-3-3v/) in order to connect to 3.3v MCUs.
* The SAM/SAMD family of MCUs (e.g. Due, Zero, MKR, etc.) as well as the RasPi Pico and others support different bit resolutions for analog/PWM pins, but may also impose other limits. See the datasheet of your MCU for details.

### WiFi

* Devices with built-in WiFi can enable such through header defines while other devices can utilize an external [serial ESP WiFi module](http://www.instructables.com/id/Cheap-Arduino-WiFi-Shield-With-ESP8266/).

## Memory Callouts

* The total number of objects and different kinds of objects (panels, servos, LDRs, relays, etc.) that the controller can support at once depends on how much free Flash storage and RAM your MCU has available. Helioduino objects range in RAM memory size from 150 to 500 bytes or more depending on settings and object type, with the base Flash memory usage ranging from 100kB to 300kB+ depending on settings.
  * For our target microcontroller range, on the low end we have older devices with 256kB of Flash and at least 8kB of RAM, while on the upper end we have more modern devices with 2+MB of Flash and 256+kB of RAM. Devices with < 32kB of RAM may struggle with system builds and may be limited to specific system setups (such as no WiFi, no data publishing, only minimal UI, etc.), while other newer devices with more capacity build with everything enabled.
* For AVR, SAM/SAMD, and other architectures that do not have C++ STL (standard container) support, there are a series of *`_MAXSIZE` defines at the top of `HelioDefines.h` that can be modified to adjust how much memory space is allocated for the various static array structures the controller uses.
* To save on the cost of code size for constrained devices, focus on not enabling that which you won't need, which has the benefit of being able to utilize code stripping to remove sections of code that don't get used.
  * There are also header defines that can strip out certain libraries and functionality, such as ones that disable the UI, multi-tasking subsystems, etc.

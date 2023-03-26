# Helioduino
Helioduino: Simple Solar Tracker Automation Controller.

**Simple-SolarTracker-Arduino v0.6.7.0**

Simple automation controller for solar tracking systems.  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, Jan 3rd, 2023.

**UNDER ACTIVE DEVELOPMENT -- WORK IN PROGRESS**

This controller allows one to set up a system of panels, servos, LDRs, relays, and other objects useful in controlling both single and dual axis sun tracking solar panel systems, and provides data monitoring & collection abilities while operating panel axis servos and/or linear actuators across the day as the sun moves to maintain optimal panel alignment. Works with a large variety of widely-available aquarium/hobbyist equipment, including popular GPS, RTC, EEPROM, SD card, WiFi, and other modules compatible with Arduino. Can be setup to calculate sun position accurately as possible or to auto-balance two opposing photoresistors per panel axis. With the right setup Helioduino can automatically do things like: drive large panels with multiple in-step linear actuators, engaging axis brakes to prevent large panels from moving/taking strain off motors that can then disengage, spray/wipe panels on routine to keep panels clean, deploy panels at sunrise and retract at sunset or when it's too windy out, remind when to realign panels, or even provide panel heating during cold temperatures or when ice is detected.

Our Keep-It-Simple controller system:

* Can be used entirely offline with RTC module and optional GPS module (or known static location) for accurate sun angle measurements and time keeping, or used online through enabled on-board WiFi/Ethernet or external ESP-AT WiFi module.
  * Uses [SolarCalculator](https://github.com/jpb10/SolarCalculator), inspired by the NOAA Solar Calculator, for fine offline calculations of the sun's solar position (including sunrise, sunset, & transit times), accurate until 2100.
* Configured system setup can be exported to EEPROM, SD card, or WiFiStorage external storage device.
  * Can be saved in pretty-print JSON for human-readability (allowing easy text editing), or in raw Binary for compactness & speed.
  * Auto-save, backup-auto-save (for auto-recovery functionality), and low storage-space cleanup (TODO) functionality.
  * Import string decode functions are pre-optimized with minimum spanning trie for ultra-fast text parsing & reduced loading times.
* Supports interval-based sensor data publishing and system event logging to MQTT IoT broker (for further IoT-integrated processing) or to external storage in .csv/.txt format (/w date in filename, segmented daily).
  * Can be extended to work with other JSON-based Web APIs or Client-like derivatives (for DB storage or server-endpoint support).
  * Can add a piezo buzzer for audible system warning/failure alerting (TODO), or a LCD/OLED/TFT display for current readings & recent logging messages (TODO).
* Enabled GUI works with a large variety of common Arduino-compatible LCD/OLED/TFT displays, touchscreens, matrix keypads, analog joysticks, rotary encoders, and momentary buttons (support by [tcMenu](https://github.com/davetcc/tcMenuLib)).
  * Contains at-a-glance system overview screen and interactive menu system for system configuration, sensor calibration, and more (TODO).
  * Critical system configuration menus can be pin-coded to prevent setup tampering, thus still allowing informational-screen/read-only access.
  * Includes remote GUI menu access through enabled WiFi, Ethernet, Bluetooth, Serial, and/or Simhub connection via tcMenu's excellent [embedCONTROL](https://github.com/davetcc/tcMenu/releases) desktop application, available for Linux/OSX/Windows.
  * GUI I/O pins can be setup as fully interrupt driven (5-25ms latency), partially interrupt driven (only keys & buttons polled), or polling based (75-100ms+ latency), and can be automatically selected depending on pins used.
  * System examples can be compiled in:
    * Disabled UI mode, which removes all GUI code entirely, freeing a large amount of Flash size for constrained (<=256kB Flash) devices.
    * Minimal UI mode, which saves on compiled sketch size through optimized code stripping at the cost of having to modify/re-upload a new sketch to change most system settings (or to change core system structure).
    * Full UI mode, which uses large amounts of Flash space available on modern MCUs to provide everything all at once, with only major system (or static linked component) changes requiring a sketch modify/re-upload.
* Actuator, Sensor, and I/O pins can be multiplexed or expanded through 8/16-bit i2c expanders for pin-limited controllers.
* Library data can be built into onboard Flash or exported onto external storage to additionally save on compiled sketch size.

Made primarily for Arduino microcontrollers / build environments, but should work with PlatformIO, Espressif, Teensy, STM32, Pico, and others - although one might experience turbulence until the bug reports get ironed out.

Dependencies include: Adafruit BusIO (dep of RTClib/tcMenu), Adafruit GPS Library (ext NMEA-AT, optional), Adafruit Unified Sensor (dep of DHT), ArduinoJson, ArxContainer (AVR/SAM STL), ArxSmartPtr (SharedPtr), DHT sensor library, I2C_EEPROM, IoAbstraction (dep of TaskManagerIO), MQTT, OneWire (platform|like), RTClib, SimpleCollections (dep of TaskManagerIO), SD (platform|like), SolarCalculator, TaskManagerIO (disableable, dep of tcMenu), tcMenu (disableable), Time, and a WiFi-like networking library (optional): WiFi101 (MKR1000 only), WiFiNINA_Generic, WiFiEspAT (ext ESP-AT), or Ethernet (platform|like).

Additional GUI (tcMenu) dependencies: Adafruit FT6206 Library (disableable), Adafruit GFX Library, Adafruit ILI9341, Adafruit ST7735 and ST7789 Library, Adafruit TouchScreen, LiquidCrystalIO, tcUnicodeHelper, TFT_eSPI, U8g2, and XPT2046_Touchscreen (optional). (Note: There may be additional sub-dependencies not listed here).

Datasheet links include: [Generic LDR information](https://components101.com/resistors/ldr-datasheet), [Generic linear actuator information](https://arduinogetstarted.com/tutorials/arduino-actuator), [DHT12 Air Temperature and Humidity Sensor](https://github.com/NachtRaveVL/Simple-SolarTracker-Arduino/blob/main/extra/dht12.pdf), but many more are available online.

*If you value the work that we do, our small team always appreciates a subscription to our [Patreon](www.patreon.com/nachtrave).*

## About

We want to make solar trackers more accessible to DIY'ers by utilizing the widely-available low-cost IoT and IoT-like microcontrollers (MCUs) of today.

With the advances in miniaturization technology bringing us even more compact MCUs at even lower costs, it becomes a lot more possible to simply use one of these small devices to do what amounts to putting a few servos in the correct offsets as the day goes by. Solar tracking is a perfect application for these devices, especially as a data logger, process monitor, and more. Professional controller systems like this can cost hundreds to even thousands of dollars, but DIY systems can wind up being a fraction of that cost.

Helioduino is a MCU-based solution primarily written for Arduino and Arduino-like MCU devices. It allows one to throw together a bunch of hobbyist servos and relays, some solar panels, maybe some light dependent resistors (LDRs), and other widely available low-cost hardware to build a functional DIY solar tracking controller system. Be it made with PVC from the hardware store or 3D printed at home, Helioduino opens the door for more people to get involved in reducing their carbon footprint, becoming more knowledgeable about their power and where it comes from - and hey, hopefully learning some basic electronics/coding along the way.

## Controller Setup

### Requirements

Minimum MCU: 256-512kB Flash, 16-24kB SRAM, 16MHz  
Recommended: 512kB-1MB+ Flash, 24-32kB+ SRAM, 32-48MHz+

* Definitely ___will___ work: GIGA, Portenta (any), ESP32/8266, Teensy 3.5+, STM32 (>256kB), Pico/RP2040 (any)

* _Can_ work /w ext. data/limited UI/small setup: Uno R4, Nano 33 (any), MKR (any), Due/Zero, Teensy 3.2, STM32 (256kB)

* _May_ work, but only with heavy tweaking/limited build: ATMega2560, Genuino 101

* Definitely ___will not___ work: Uno (classic to R3), Nano (classic & Every), Leonardo/Duemilanove, Micro, Pro, Esplora, Teensy 2/LC, STM8 (|32<256kB), ATtiny (any)

Note: Pin-limited MCUs may be restricted in how many sensors/actuators/etc. can be connected at once, and in such case where more pins are needed an i2c-based 8/16-bit expander using a PCF857X or MCP23017 might be recommended. The controller also supports standard multiplexing with a CD74HC or similar.

Note: Certain MCUs, such as those from STM, are sold in many different Flash/SRAM size configurations. Some configurations may not be supported, others may limit total system size (i.e. object count, library support, features, etc.). Bigger is always better until you get a better idea of your specific use case's size requirements.

### Installation

The easiest way to install this controller is to utilize the Arduino IDE library manager, or through a package manager such as PlatformIO. Otherwise, simply download this controller and extract its files into a `Simple-SolarTracker-Arduino` folder in your Arduino custom libraries folder, typically found in your `[My ]Documents\Arduino\libraries` folder (Windows), or `~/Documents/Arduino/libraries/` folder (Linux/OSX).

From there, you can make a local copy of one of the example sketches based on the kind of system setup you want to use. If you are unsure of which, we recommend the Dual-Axis Tracking Example, as it is our standard implementation built for most common system setups and only requires changing setup defines at the top of the file.

Storage constrained MCUs (< 512kB Flash, particularly <= 256kB) may need further setup file/max-sizes tweaking, and possibly external storage hardware (such as EEPROM or SD Card - see the Data Writer example for more details). Modern MCUs with lots of Flash storage can instead simply build the Full System Example (TODO: Still a WIP - use DA Tracking Example for right now).

### Setup

#### Header Defines

There are several defines inside of the controller's main `Helioduino[UI].h` header file that allow for more fine-tuned control of the controller. You may edit and uncomment these lines directly, or supply them via custom build flags. While editing the main header file isn't ideal, it is often easiest. Note that editing the controller's main header file directly will affect all projects compiled on your system using those modified controller files.

Alternatively, you may also refer to <https://forum.arduino.cc/index.php?topic=602603.0> on how to define custom build flags manually via modifying the platform[.local].txt file, or with the Arduino CLI (preferred way going forward).

For the older platform.local.txt file override approach, create platform.local.txt alongside platform.txt located in %applocaldata%\Arduino15\packages\{platform}\hardware\{arch}\{version}\ (replacing %applocaldata%\Arduino15 with ~/Library/Arduino15 for OSX, and ~/.arduino15 for Linux), with the contents: `compiler.cpp.extra_flags=-Dname` (replacing `name` with full name of below define). Note that it will affect all builds for that platform until again changed/removed. Some build systems may require directly editing platform.txt and adding onto the end of its CPP build recipe, e.g. Teensy & `recipe.cpp.o.pattern=<bunch-of-stuff> -Dname`.

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
//#define HELIO_DISABLE_BUILTIN_DATA              // Disables library data existing in Flash, see DataWriter example for exporting details

// Uncomment or -D this define to enable debug output (treats Serial output as attached to serial monitor).
//#define HELIO_ENABLE_DEBUG_OUTPUT

// Uncomment or -D this define to enable verbose debug output (note: adds considerable size to compiled sketch).
//#define HELIO_ENABLE_VERBOSE_DEBUG

// Uncomment or -D this define to enable debug assertions (note: adds significant size to compiled sketch).
//#define HELIO_ENABLE_DEBUG_ASSERTIONS
```

From shared/HelioduinoUI.h:
```Arduino
// Uncomment or -D this define to enable usage of the XPT2046_Touchscreen library, in place of the Adafruit FT6206 library.
//#define HELIO_UI_ENABLE_XPT2046TS               // https://github.com/PaulStoffregen/XPT2046_Touchscreen

// Uncomment or -D this define to enable usage of the StChromaArt LDTC framebuffer capable canvas in place of default U8g2Drawable canvas (STM32/mbed only, note: requires advanced setup)
//#define HELIO_UI_ENABLE_STCHROMA_LDTC

// Uncomment or -D this define to enable usage of the StChromaArt BSP touch screen interrogator in place of the default AdaLibTouchInterrogator (STM32/mbed only, note: requires advanced setup, see tcMenu_Extra_BspUserSettings.h)
//#define HELIO_UI_ENABLE_BSP_TOUCH

// Uncomment or -D this define to enable usage of the debug menu 
//#define HELIO_UI_ENABLE_DEBUG_MENU
```

#### External Libraries

Certain setups may require additional, and in some cases specialized, library dependency setup in order to function. This is mainly seen around certain display and input options. Ones to highlight include:

* **U8g2** (monochrome OLED displays): The CustomOLED display output option uses `HELIO_UI_CUSTOM_OLED_I2C` and/or `HELIO_UI_CUSTOM_OLED_SPI` for allocating a custom U8g2 device. These defines should resolve to an appropriate U8g2 based device string, such as `U8G2_SSD1309_128X64_NONAME0_F_HW_I2C`, defined en-masse inside of the U8g2 library header file (note: the `_F_` part of the name implies frame buffer support for non-flickering animations - recommended). This custom option has static linkage against a single custom i2c/SPI device at a time and will require sketch modify/re-upload upon needing any changes.

* **TFT_eSPI** (advanced color TFT display library): The TFT display output option (in lieu of the default AdafruitGFX driven TFT options) uses the configuration specified via its `TFT_eSPI\User_Setup.h` library setup file. This library always has static linkage and will require sketch modify/re-upload upon needing any changes. Use of this library is only recommended for advanced users.

* **BSP_LCD** & **BSP Touch** (STM32746G-Discovery on STM32/mbed only): This particular setup can utilize a ChromaArt-based drawable (in place of U8g2Drawable) with STM32 LDTC frame buffer, and requires advanced user setup via the included `shared\tcMenu_Extra_BspUserSettings.h` library setup file. This library always has static linkage and will require sketch modify/re-upload upon needing any changes. Use of this library is only recommended for advanced users.

* **ST7789::CustomTFT**, **TFT_eSPI**: These options utilize the `TFT_GFX_WIDTH` and `TFT_GFX_HEIGHT` defines for the screen width/height (defaulting to TFT_eSPI's `TFT_WIDTH` and `TFT_HEIGHT` values if defined, else assuming standard 240x320), and should be either edited directly or defined through custom build defines. These values are statically linked and will require sketch modify/re-upload upon needing any changes.

### Initialization

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
               DeviceSetup displaySetup = DeviceSetup());           // Display device setup (i2c/spi)
```

#### Controller Initialization

Additionally, a call is expected to be provided to the controller class object's `init[From…](…)` method, commonly called inside of the sketch's `setup()` function. This allows one to set the controller's system type (Tracking or Balancing), units of measurement (Metric, Imperial, or Scientific), control input mode, and display output mode. The default mode of the controller, if left unspecified, is a position calculating system set to Metric units, without any input control or output display.

From Helioduino.h, in class Helioduino:
```Arduino
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
    inline bool enableSysLoggingToSDCard(String logFilePrefix = "logs/he");

    // Enables data publishing to the SD card. Log file names will append YYMMDD.csv to the specified prefix. Returns success flag.
    inline bool enableDataPublishingToSDCard(String dataFilePrefix = "data/he");
```

## Hookup Callouts

* The recommended Vcc power supply and logic level is 5v, with most newer MCUs restricted to 3.3v.
  * There are many devices that are 3.3v only and not 5v tolerant. Check your IC's datasheet for details.
* 5v device output pins that interface with any 3.3v device input pins that are not 5v tolerant (such as a 5v AVR interfacing with a 3.3v-only [serial ESP-AT WiFi module](http://www.instructables.com/id/Cheap-Arduino-WiFi-Shield-With-ESP8266/), or a 3.3v MCU interfacing with a 5v analog sensor), will require a bi-directional logic level converter/shifter to use, especially for any high-speed digital data transfer lines.
  * Alternatively, using a 10kΩ resistor can often times be enough to 'convert' 5v to 3.3v, but the correct way is to utilize a 1kΩ resistor and a 2kΩ resistor (or any size with a 1:2 ratio) in a [simple voltage divider circuit](https://randomnerdtutorials.com/how-to-level-shift-5v-to-3-3v/).

### Serial UART

Serial UART uses individual communication lines for each device, with the receive `RX` pin of one being the transmit `TX` pin of the other - thus having to "flip wires" when connecting. However, devices can always be active and never have to share their access. UART runs at low to mid kHz speeds and is useful for simple device control, albeit somewhat clumsy at times.

* When wiring up modules that use Serial UART, make sure to flip `RX`/`TX` pins.
* Always ensure that the data output pins and data input pins have compatible voltages.

Serial UART Devices Supported: Bluetooth-AT modules, ESP-AT WiFi modules, NMEA-AT GPS modules

### SPI Bus

SPI devices can be chained together on the same shared data lines, which are typically labeled `COPI` (or `MOSI`), `CIPO` (or `MISO`), and `SCK`, often with an additional `CS` (or `SS`). Each SPI device requires its own individual cable-select `CS` wire as only one SPI device may be active at any given time - accomplished by pulling its `CS` line of that device low (aka active-low). SPI runs at MHz speeds and is useful for large data block transfers.

* The `CS` pin may be connected to any digital output pin, but it's common to use the `CS` (or `SS`) pin for the first device. Additional devices are not restricted to what pin they can or should use, but given it's not a data pin not using a choice interrupt-capable pin allows those to be used for interrupt driven mechanisms.
* Many low-cost SPI-based SD card modules on market only read SDHC sized SD cards (2GB to 32GB) formatted in FAT32 (filenames limited to 8 characters plus 3 character file extension).
  * Some SD cards simply will not play nicely with these modules and you may have to try another SD card manufacturer. We recommend 32GB SD cards due to overall lowest cost (smaller SD cards actually becoming _more_ expensive).
* Many various graphical displays may have an additional `DC` (or `RS`) pin, which is required to be connected to any open digital pin in addition to its `CS` pin.
  * There is often an additional `Reset` (or `RST`) pin that needs either wired to an open digital pin for MCU control, otherwise typically will need hard-tied to a HIGH signal (such as that from `Vcc`) in order for the display to function/turn-on.
  * There is also often an additional `LED` (or `BL`) pin that controls the backlight that can be either optionally wired to an open digital or analog pin for MCU control, otherwise can be hard-tied typically to a HIGH signal (such as that from `Vcc`) in order to stay always-on, or simply left disconnected for device default.
* Always ensure that the data output pins and data input pins have compatible voltages.

SPI Devices Supported: SD card modules, NMEA GPS modules, 128x128+ LCD/OLED/TFT graphical displays, XPT2046 touchscreens

### I2C Bus

I2C (aka I²C, IIC, TwoWire, TWI) devices can be chained together on the same shared data lines (no flipping of wires), which are typically labeled `SCL` and `SDA`. Only different kinds of I2C devices can be used on the same data line together using factory default settings, otherwise manual addressing must be performed. I2C runs at mid to high kHz speeds and is useful for advanced device control.

* When more than one I2C device of the same kind is to be used on the same data line, each device must be set to use a different address. This is accomplished via the A0-A2 (sometimes A0-A5) pins/pads on the physical device that must be set either open or closed (typically via a de-solderable resistor, or by shorting a pin/pad). Check your specific breakout's datasheet for details.
* Note that not all the I2C libraries used support multi-addressable I2C devices at this time (read as: may only use one). Currently, this restriction applies to: RTC devices.
* Always ensure that the data output pins and data input pins have compatible voltages.

I2C Devices Supported: DS*/PCF* RTC modules, AT24C* EEPROM modules, NMEA GPS modules, 16x2/20x4 LCD displays, 128x32/128x64 OLED displays, FT6206 touchscreens, 8/16-bit pin expanders

### OneWire Bus

OneWire devices can be chained together on the same shared data lines (no flipping of wires). Devices can be of the same or different types, require minimal setup (and often no soldering), and most can even operate in "parasite" power mode where they use the power from the data line (and an internal capacitor) to function (thus saving a `Vcc` line, only requiring `Data` and return `GND`). OneWire runs only in the low kb/s speeds and is useful for light-weight digital sensors.

* Typically, sensors are limited to 20 devices along a maximum 100m of wire.
* When more than one OneWire device is on the same device line, each device registers itself an enumeration index (0 - N) along with its own 64-bit unique identifier (UUID, with last byte being CRC). The device can then be referenced via this UUID by the system in the future indefinitely, or enumeration index so long as the device doesn't change its line position.
* Always ensure that the data output pins and data input pins have compatible voltages.

OneWire Devices Supported: DHT* 1W air temp/humidity sensors

### Analog IO

* All analog sensors will need to have the same operational voltage range as the controller supports. Many analog sensors are set to use 0v to 5v by default, but some can go -5v to +5v, some even up to 5.5v.
  * Note: Altering default factory calibration settings may require addition tools for setting up a new calibration, such as special calibration fluids/procedures/etc. Refer to the datasheet of your device for details.
* The `AREF` (or `IOREF`) pin, which controls the upper-bound of this range, by default if left not-connected (NC) is the same voltage as the MCU. Analog sensors must not exceed this voltage limit.
  * 5v analog sensor output signals connecting to 3.3v MCUs that are not 5v tolerant **must** either be: [level converted](https://randomnerdtutorials.com/how-to-level-shift-5v-to-3-3v/) in order to connect, or configured to output 0v to `AREF` (or `IOREF`) in voltage calibration output range (if able to calibrate - see note above).
  * Warning: Too high of applied voltage to any pin incapable of receiving such high a voltage risks permanent damage to that device. _Always_ ensure that the applied voltage level coming out a device is supported when going back into another. Some breakouts/IC's have 5v tolerance built-in, some do not. Refer to the datasheet of your MCU/device for details.
  * Note: Typically a 3.3v output signal will _not_ need level converted up to 5v for a 5v digital input to operate (read as: 3.3v is plenty enough to trigger HIGH on 5v device inputs).
* The SAM/SAMD family of MCUs (e.g. Due, Zero, MKR, Nano 33, etc.) as well as many more modern MCUs support different bit resolutions for analog/PWM pins (tied to overridable `DAC_RESOLUTION` & `ADC_RESOLUTION` defines), with some (e.g. Pico, ESP32, etc.) supporting any pin being digital or analog w/o restriction. Refer to the datasheet of your MCU for details.

### Sensors

* Many different kinds of hobbyist sensors label their analog output `AO` (or `Ao`) - however, always check your specific sensor's datasheet.
  * Again, make sure all analog sensors are calibrated to output the same 0v - `AREF` (or `IOREF`) volts in range.
* Sensor pins used for event triggering when measurements go above/below a pre-set tolerance - many of which are deceptively labeled `DO` (or `Do`), despite having nothing to do with being `D`ata lines of any kind - can be safely ignored, as the software implementation of such mechanism is more than sufficient.
  * Often these connections are used to drive other hardware-only based solutions that aren't a part of Helioduino's use case, but can still be connected up using a BinarySensor that triggers upon specific conditions, possibly using an ISR-capable pin if desired.

### Networking & Wireless

* Networking of any kind is 100% optional, with all base functionality being able to be performed on a fully remote basis utilizing a single RTC and optional GPS (or known static location).
  * Networking does however make things much simpler and is highly recommended.
* Devices with built-in WiFi or Ethernet can enable such through header/build defines while other devices can utilize an external [serial ESP WiFi module](http://www.instructables.com/id/Cheap-Arduino-WiFi-Shield-With-ESP8266/) on any open Serial line.
  * Warning: While WiFi password is encrypted into system settings data, it should not be considered secure.
* Serial Bluetooth-AT modules can be used on any open Serial port to provide remote device control (only).
* MQTT requires remotely accessible broker daemon in order to publish sensor data (setup separately).
* UDP time server requires remotely accessible time & date API service in order to sync time (TODO).
  * RTC not required / used in reserve when UDP service enabled.
* Note: Geo-location APIs require external 3rd party monthly subscription fees, thus isn't included as a feature.

## Memory Callouts

* The total number of objects and different kinds of objects (panels, servos, LDRs, relays, etc.) that the controller can support at once depends on how much free Flash storage and SRAM your MCU has available. Helioduino C++0x11 objects range in memory usage size from 150 to 750 bytes or more depending on settings and object type, with the compiled Flash binary ranging in size from 200kB to 750kB+ depending on platform and settings.
  * For our supported microcontroller range, on the low end we have devices with 256kB of Flash and at least 16kB of SRAM, while on the upper end we have more modern devices with 1MB+ of Flash and 32kB+ of SRAM. Devices with < 24kB of SRAM may struggle with system builds and may be limited to minimal system setups (such as no WiFi, no data publishing, no built-in library data, only minimal-to-no GUI, etc.), while other newer devices with more capacity build with everything enabled.
* For AVR, SAM, and other build architectures that do not have C++0x11 STL (standard container library) support, there are a series of *`_MAXSIZE` defines nearer to the top of `Helio[UI]Defines.h` that can be modified to adjust how much memory space is allocated for the various static array structures the controller instead uses.
* To save on the cost of code size for constrained devices, focus on not enabling that which you won't need, which has the benefit of being able to utilize code stripping to remove sections of code that don't get used.
  * There are also header defines that can strip out certain libraries and functionality, such as ones that disable the GUI, multi-tasking subsystems, etc.
* To further save on code size cost, see the Data Writer Example on how to externalize library data onto an SD Card or EEPROM.
  * Note: Upgrading between versions or changing custom/program data may require you to re-build and re-deploy to such external device.

## Example Usage

Below are several examples of controller usage.

### Simple Light Dependent Resistor (LDR) System Example

LDR setups are great for beginners, and has the advantage of being able to be built out of commonly available materials. A positional servo is used to tilt the panel, which is mounted horizontally facing the general direction of the sun.

The Simple LDR Example sketch shows how a simple Helioduino system can be setup using the most minimal of work. In this sketch only that which you actually use is built into the final compiled binary, making it an ideal lean choice for those who don't need anything fancy. This sketch has no UI or input control, but with a simple buzzer and some additional sensors the system can control another axis for a gimballed system.

```Arduino
#include <Helioduino.h>

#define SETUP_PIEZO_BUZZER_PIN          -1              // Piezo buzzer pin, else -1
#define SETUP_AXIS_SERVO_PIN            A0              // Axis servo write pin (analog)
#define SETUP_LDR_LOWER_PIN             A1              // Lower LDR read pin (analog)
#define SETUP_LDR_UPPER_PIN             A2              // Upper LDR read pin (analog)

#define SETUP_SERVO_MIN_DEG             -90             // Minimum degrees of axial servo
#define SETUP_SERVO_MAX_DEG             90              // Maximum degrees of axial servo

#define SETUP_PANEL_TYPE                Horizontal      // Panel type (Horizontal, Vertical, Gimballed, Equatorial)
#define SETUP_PANEL_HOME                {0.0f,0.0f}     // Panel home position (azimuth,elevation or RA,declination)
#define SETUP_PANEL_OFFSET              {0.0f,0.0f}     // Panel offset position (azi,ele or RA,dec)

Helioduino helioController(SETUP_PIEZO_BUZZER_PIN);     // Controller using default setup aside from buzzer pin, if defined

float _SETUP_PANEL_HOME[] = SETUP_PANEL_HOME;
float _SETUP_PANEL_OFFSET[] = SETUP_PANEL_OFFSET;

void setup() {
    // Setup base interfaces
    #ifdef HELIO_ENABLE_DEBUG_OUTPUT
        Serial.begin(115200);           // Begin USB Serial interface
        while (!Serial) { ; }           // Wait for USB Serial to connect
    #endif

    // Initializes controller with LDR environment (saves some time/space), no logging, eeprom, SD, or anything else.
    helioController.init(Helio_SystemMode_Balancing);

    // Adds a simple horizontal LDR balanced solar panel, and sets up any specified offsets.
    auto panel = helioController.addLDRBalancingPanel(JOIN(Helio_PanelType,SETUP_PANEL_TYPE));
    panel->setHomePosition(_SETUP_PANEL_HOME);
    panel->setAxisOffset(_SETUP_PANEL_OFFSET);

    // Adds a simple positional servo at SETUP_AXIS_SERVO_PIN, installed to control the vertical elevation of the panel.
    auto axisServo = helioController.addPositionalServo(SETUP_AXIS_SERVO_PIN, SETUP_SERVO_MIN_DEG, SETUP_SERVO_MAX_DEG);
    axisServo->setParentPanel(panel, Helio_PanelAxis_Elevation);

    // Adds a light intensity sensor at SETUP_LDR_LOWER_PIN, installed on the lower side of the panel.
    auto ldrLower = helioController.addLightIntensitySensor(SETUP_LDR_LOWER_PIN);
    panel->setLDRSensor(ldrLower, Helio_PanelLDR_VerticalMin); // will provide downwards control

    // Adds a light intensity sensor at SETUP_LDR_UPPER_PIN, installed on the upper side of the panel.
    auto ldrUpper = helioController.addLightIntensitySensor(SETUP_LDR_UPPER_PIN);
    panel->setLDRSensor(ldrUpper, Helio_PanelLDR_VerticalMax); // will provide upwards control

    // Launches controller into main operation.
    helioController.launch();
}

void loop()
{
    // Helioduino will manage most updates for us.
    helioController.update();
}

```

### Main System Examples

There are two main system examples to choose from, Dual-Axis (DA) Tracking and Full System, each with its own requirements and capabilities. The DA Tracking Example is the recommended starting point for most system builders, and is perfect for those with only basic programming knowledge. It can be easily extended to include other functionality if desired, simply by copying and pasting the example code.

The DA Tracking Example sketch has the benefit of being able to compile in a minimal UI mode that will strip out what isn't used, making it ideal for storage constrained devices (e.g. those with < 512kB Flash), but will not provide full UI functionality since it will be missing the code for all the other objects the system build code strips out, thus requiring re-compiling/re-uploading on system setup changes. The UI, in this mode, only provides edit capabilities, not create/delete, with more customization options locked out.

By contrast, the Full System Example sketch will build an empty system with all object and system features enabled. It is recommended for modern MCUs that have lots of storage space to use such as ESP32, RasPi Pico, etc. It works similarly to the DA Tracking Example, except is meant for systems where a GUI (possibly only remotely controlled) will be primarily used to create objects with. It involves the least amount of coding, but comes at the highest cost since it has to contain everything that possibly could be used, even if it ultimately isn't. The UI, in this mode, provides edit, create, and delete capabilities, with more options available for customization.

Included below is the default system setup defines of the DA Tracking example (of which a smaller similar version is used for the Full System example) to illustrate a variety of the controller features. This is not an exhaustive list of course, as there are many more things the controller is capable of, as documented in its main header file include, GitHub Project Wiki, and elsewhere.

```Arduino
// Pins & Class Instances
#define SETUP_PIEZO_BUZZER_PIN          -1              // Piezo buzzer pin, else -1
#define SETUP_EEPROM_DEVICE_TYPE        None            // EEPROM device type/size (AT24LC01, AT24LC02, AT24LC04, AT24LC08, AT24LC16, AT24LC32, AT24LC64, AT24LC128, AT24LC256, AT24LC512, None)
#define SETUP_EEPROM_I2C_ADDR           0b000           // EEPROM i2c address (A0-A2, bitwise or'ed with base address 0x50)
#define SETUP_RTC_DEVICE_TYPE           None            // RTC device type (DS1307, DS3231, PCF8523, PCF8563, None)
#define SETUP_SD_CARD_SPI               SPI             // SD card SPI class instance
#define SETUP_SD_CARD_SPI_CS            -1              // SD card CS pin, else -1
#define SETUP_SD_CARD_SPI_SPEED         F_SPD           // SD card SPI speed, in Hz (ignored on Teensy)
#define SETUP_DISP_LCD_I2C_ADDR         0b111           // LCD i2c address (A0-A2, bitwise or'ed with base address 0x20)
#define SETUP_DISP_OLED_I2C_ADDR        0b000           // OLED i2c address (A0-A2, bitwise or'ed with base address 0x78)
#define SETUP_DISP_SPI                  SPI             // Display SPI class instance
#define SETUP_DISP_SPI_CS               -1              // Display SPI CS pin, else -1
#define SETUP_DISP_SPI_SPEED            F_SPD           // Display SPI speed, in Hz
#define SETUP_CTRL_INPUT_PINS           {hpin_none}     // Control input pins, else {-1}
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
#define SETUP_ETHERNET_MAC              { (uint8_t)0xDE, (uint8_t)0xAD, (uint8_t)0xBE, (uint8_t)0xEF, (uint8_t)0xFE, (uint8_t)0xED } // Ethernet MAC address
#define SETUP_ETHERNET_SPI              SPI1            // Ethernet SPI class instance
#define SETUP_ETHERNET_SPI_CS           SS1             // Ethernet CS pin

// GPS Settings                                         (note: define HELIO_ENABLE_GPS to enable GPS)
#define SETUP_GPS_TYPE                  None            // Type of GPS (UART, I2C, SPI, None)
#define SETUP_GPS_SERIAL                Serial1         // GPS serial class instance, if using serial
#define SETUP_GPS_I2C_ADDR              0b000           // GPS i2c address (A0-A2, bitwise or'ed with base address 0x10), if using i2c
#define SETUP_GPS_SPI                   SPI             // GPS SPI class instance, if using spi
#define SETUP_GPS_SPI_CS                SS              // GPS CS pin, if using spi

// System Settings
#define SETUP_SYSTEM_MODE               Tracking        // System run mode (Tracking, Balancing)
#define SETUP_MEASURE_MODE              Default         // System measurement mode (Default, Imperial, Metric, Scientific)
#define SETUP_DISPLAY_OUT_MODE          Disabled        // System display output mode (Disabled, LCD16x2_EN, LCD16x2_RS, LCD20x4_EN, LCD20x4_RS, SSD1305, SSD1305_x32Ada, SSD1305_x64Ada, SSD1306, SH1106, CustomOLED, SSD1607, IL3820, IL3820_V2, ST7735, ST7789, ILI9341, TFT)
#define SETUP_CONTROL_IN_MODE           Disabled        // System control input mode (Disabled, RotaryEncoderOk, RotaryEncoderOkLR, UpDownButtonsOk, UpDownButtonsOkLR, UpDownESP32TouchOk, UpDownESP32TouchOkLR, AnalogJoystickOk, Matrix2x2UpDownButtonsOkL, Matrix3x4Keyboard_OptRotEncOk, Matrix3x4Keyboard_OptRotEncOkLR, Matrix4x4Keyboard_OptRotEncOk, Matrix4x4Keyboard_OptRotEncOkLR, ResistiveTouch, TouchScreen, TFTTouch, RemoteControl)
#define SETUP_SYS_UI_MODE               Minimal         // System user interface mode (Disabled, Minimal, Full)
#define SETUP_SYS_NAME                  "Helioduino"    // System name
#define SETUP_SYS_TIMEZONE              +0              // System timezone offset, in hours (int or float)
#define SETUP_SYS_LOGLEVEL              All             // System log level filter (All, Warnings, Errors, None)
#define SETUP_SYS_STATIC_LAT            DBL_UNDEF       // System static latitude (if not using GPS/UI, else DBL_UNDEF), in degrees
#define SETUP_SYS_STATIC_LONG           DBL_UNDEF       // System static longitude (if not using GPS/UI, else DBL_UNDEF), in minutes
#define SETUP_SYS_STATIC_ALT            DBL_UNDEF       // System static altitude (if not using GPS/UI, else DBL_UNDEF), in meters above sea level (msl)

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
#define SETUP_MQTT_BROKER_IPADDR        { (uint8_t)192, (uint8_t)168, (uint8_t)1, (uint8_t)2 } // IP address that MQTT broker exists at
#define SETUP_MQTT_BROKER_PORT          1883            // Port number that MQTT broker exists at

// External Data Settings
#define SETUP_EXTDATA_SD_ENABLE         false           // If data should be read from an external SD card (searched first for panels lib data)
#define SETUP_EXTDATA_SD_LIB_PREFIX     "lib/"          // Library data folder/data file prefix (appended with {type}##.dat)
#define SETUP_EXTDATA_EEPROM_ENABLE     false           // If data should be read from an external EEPROM (searched first for strings data)

// External EEPROM Settings
#define SETUP_EEPROM_SYSDATA_ADDR       0x2222          // System data memory offset for EEPROM saves (from Data Writer output)
#define SETUP_EEPROM_STRINGS_ADDR       0x0000          // Start address for strings data (from Data Writer output)
#define SETUP_EEPROM_UIDSTRS_ADDR       0x1111          // Start address for UI strings data (from Data Writer output, GUI not disabled)

// UI Settings
#define SETUP_UI_LOGIC_LEVEL            ACT_LOW         // I/O signaling logic active level (ACT_LOW, ACT_HIGH)
#define SETUP_UI_ALLOW_INTERRUPTS       true            // Allow interrupt driven I/O if able, else force polling
#define SETUP_UI_USE_TCUNICODE_FONTS    false           // Use tcUnicode fonts instead of AdafruitGFX fonts, if using graphical display
#define SETUP_UI_IS_DFROBOTSHIELD       false           // Using DFRobotShield as preset (SETUP_CTRL_INPUT_PINS may be left {-1})

// UI Display Output Settings
#define SETUP_UI_GFX_ROTATION           R0              // Display rotation (R0, R1, R2, R3, HorzMirror, VertMirror), if using graphical display or touchscreen
#define SETUP_UI_GFX_DC_PIN             -1              // Display interface DC/RS pin, if using SPI display
#define SETUP_UI_GFX_RESET_PIN          -1              // Optional display interface reset/RST pin, if using SPI display, else -1 (Note: Unused reset pin typically needs tied to HIGH for display to function)
#define SETUP_UI_GFX_ST7735_TAG         Undefined       // ST7735 tag color (B, Green, Green18, Red, Red18, Black, Black18, Green144, Mini, Mini_Plugin, Hallo_Wing), if using ST7735 display
#define SETUP_UI_GFX_ST7789_RES         Undefined       // ST7789 screen resolution (128x128, 135x240, 170x320, 172x320, 240x240, 240x280, 240x320, CustomTFT), if using ST7789 display
#define SETUP_UI_GFX_BACKLIGHT_PIN      -1              // Optional display interface backlight/LED/BL pin, if using SPI display (Note: Unused backlight pin can optionally be tied typically to HIGH for always-on)
#define SETUP_UI_GFX_BACKLIGHT_MODE     Normal          // Display backlight mode (Normal, Inverted, PWM), if using LCD or display /w backlight pin
#define SETUP_UI_GFX_BACKLIGHT_ESP_CHN  1               // Backlight PWM channel, if on ESP/using PWM backlight
#define SETUP_UI_GFX_BACKLIGHT_ESP_FRQ  1000            // Backlight PWM frequency, if on ESP/using PWM backlight

// UI Control Input Settings
#define SETUP_UI_ENC_ROTARY_SPEED       HalfCycle       // Rotary encoder cycling speed (FullCycle, HalfCycle, QuarterCycle)
#define SETUP_UI_KEY_REPEAT_SPEED       20              // Key repeat speed, in ticks
#define SETUP_UI_KEY_REPEAT_DELAY       850             // Key repeat delay, in milliseconds
#define SETUP_UI_KEY_REPEAT_INTERVAL    350             // Key repeat interval, in milliseconds
#define SETUP_UI_JS_ACCELERATION        3.0f            // Joystick acceleration (decrease divisor), if using analog joystick
#define SETUP_UI_TOUCHSCREEN_ORIENT     Same            // Touchscreen orientation tuning (Same, None, InvertX, InvertY, InvertXY, SwapXY, InvertX_SwapXY, InvertY_SwapXY, InvertXY_SwapXY), if using touchscreen
#define SETUP_UI_ESP32TOUCH_SWITCH      800             // ESP32 Touch key switch threshold, if on ESP32/using ESP32Touch
#define SETUP_UI_ESP32TOUCH_HVOLTS      V_2V7           // ESP32 Touch key high reference voltage (Keep, V_2V4, V_2V5, V_2V6, V_2V7, Max), if on ESP32/using ESP32Touch
#define SETUP_UI_ESP32TOUCH_LVOLTS      V_0V5           // ESP32 Touch key low reference voltage (Keep, V_0V5, V_0V6, V_0V7, V_0V8, Max), if on ESP32/using ESP32Touch
#define SETUP_UI_ESP32TOUCH_HVATTEN     V_1V            // ESP32 Touch key high ref voltage attenuation (Keep, V_1V5, V_1V, V_0V5, V_0V, Max), if on ESP32/using ESP32Touch

// UI Remote Control Settings
#define SETUP_UI_REMOTE1_TYPE           Disabled        // Type of first remote control (Disabled, Serial, Simhub, WiFi, Ethernet)
#define SETUP_UI_REMOTE1_UART           Serial1         // Serial setup for first remote control, if Serial/Simhub
#define SETUP_UI_REMOTE2_TYPE           Disabled        // Type of second remote control (Disabled, Serial, Simhub, WiFi, Ethernet)
#define SETUP_UI_REMOTE2_UART           Serial1         // Serial setup for second remote control, if Serial/Simhub
#define SETUP_UI_RC_NETWORKING_PORT     3333            // Remote controller networking port, if WiFi/Ethernet

// Device Setup
#define SETUP_AC_USAGE_SENSOR_PIN       -1              // AC power usage meter sensor pin (analog), else -1
#define SETUP_DC_USAGE_SENSOR_PIN       -1              // DC power usage meter sensor pin (analog), else -1
#define SETUP_DC_GEN_SENSOR_PIN         -1              // DC power generation meter sensor pin (analog), else -1
#define SETUP_DHT_AIR_TEMP_HUMID_PIN    -1              // DHT* air temp sensor data pin (digital), else -1
#define SETUP_DHT_SENSOR_TYPE           None            // DHT sensor type enum (DHT11, DHT12, DHT21, DHT22, AM2301, None)
#define SETUP_ICE_INDICATOR_PIN         -1              // Ice indicator pin (digital), else -1
#define SETUP_ICE_INDICATOR_TYPE        ACT_HIGH        // Ice indicator type/active level (ACT_HIGH, ACT_LOW)
#define SETUP_MINMAX_INDICATOR_TYPE     ACT_HIGH        // Linear actuator min/max endstop indicator type/active level (ACT_HIGH, ACT_LOW)
#define SETUP_LINACT1_AXIS1_PINA        -1              // Ele/dec axis linear actuator #1 pin A (digital), else -1
#define SETUP_LINACT1_AXIS1_PINB        -1              // Ele/dec axis linear actuator #1 pin B (digital), else -1
#define SETUP_LINACT1_POS_SENSOR_PIN    -1              // Ele/dec axis linear actuator #1 position sensor pin (analog), else -1
#define SETUP_LINACT1_MIN_ENDSTOP_PIN   -1              // Ele/dec axis linear actuator #1 min endstop pin (digital), else -1
#define SETUP_LINACT1_MAX_ENDSTOP_PIN   -1              // Ele/dec axis linear actuator #1 max endstop pin (digital), else -1
#define SETUP_LINACT2_AXIS1_PINA        -1              // Ele/dec axis linear actuator #2 pin A (digital), else -1
#define SETUP_LINACT2_AXIS1_PINB        -1              // Ele/dec axis linear actuator #2 pin B (digital), else -1
#define SETUP_LINACT2_POS_SENSOR_PIN    -1              // Ele/dec axis linear actuator #2 position sensor pin (analog), else -1
#define SETUP_LINACT2_MIN_ENDSTOP_PIN   -1              // Ele/dec axis linear actuator #2 min endstop pin (digital), else -1
#define SETUP_LINACT2_MAX_ENDSTOP_PIN   -1              // Ele/dec axis linear actuator #2 max endstop pin (digital), else -1
#define SETUP_PANEL_BRAKE_PIN           -1              // Panel brake relay pin (digital), else -1
#define SETUP_PANEL_HEATER_PIN          -1              // Panel heater relay pin (digital), else -1
#define SETUP_PANEL_SPRAYER_PIN         -1              // Panel sprayer/wiper relay pin (digital), else -1
#define SETUP_PANEL_TILT_AXIS1_PIN      -1              // Ele/dec axis panel tilt angle sensor pin (analog), else -1
#define SETUP_POS_SERVO_AXIS0_PIN       -1              // Azi/RA axis positional servo pin (analog), else -1
#define SETUP_POS_SERVO_AXIS1_PIN       -1              // Ele/dec axis positional servo pin (analog), else -1
#define SETUP_WIND_SPEED_SENSOR_PIN     -1              // Wind speed sensor pin (analog), else -1

// Device Multiplexing Setup
#define SETUP_MUXER_CHANNEL_BITS        -1              // Multiplexer channel bits (3 = 8-bit, 4 = 16-bit), else -1
#define SETUP_MUXER_ADDRESS_PINS        {hpin_none}     // Multiplexer addressing bus/channel pins, else {-1}
#define SETUP_MUXER_ENABLE_PIN          -1              // Multiplexer chip enable pin (optional), else -1
#define SETUP_MUXER_ENABLE_TYPE         ACT_LOW         // Multiplexer chip enable pin type/active level (ACT_HIGH, ACT_LOW)

// Device Pin Expanders Setup                           // (Note: Will redefine any used pins from channel setup below to a virtual pin = 100+chnl#)
#define SETUP_EXPANDER1_CHANNEL_BITS    -1              // Pin expander 1 channel bits (3 = 8-bit, 4 = 16-bit), else -1
#define SETUP_EXPANDER2_CHANNEL_BITS    -1              // Pin expander 2 channel bits (3 = 8-bit, 4 = 16-bit), else -1
#define SETUP_EXPANDER1_IOREF_I2C_ADDR  0x27            // Pin expander 1 full I2C device address (including device base offset)
#define SETUP_EXPANDER2_IOREF_I2C_ADDR  0x28            // Pin expander 2 full I2C device address (including device base offset)
#define SETUP_EXPANDER1_IOREF_ISR_PIN   -1              // Pin expander 1 interrupt pin, else -1
#define SETUP_EXPANDER2_IOREF_ISR_PIN   -1              // Pin expander 2 interrupt pin, else -1
#define SETUP_EXPANDER_IOREF_I2C_WIRE   Wire            // Pin expanders I2C wire class instance
// IORef allocation command using ioFrom* functions in IoAbstraction for pin expander 1
#define SETUP_EXPANDER1_IOREF_ALLOC()   ioFrom8574((uint8_t)SETUP_EXPANDER1_IOREF_I2C_ADDR, (pinid_t)SETUP_EXPANDER1_IOREF_ISR_PIN, &SETUP_EXPANDER_IOREF_I2C_WIRE)
// IORef allocation command using ioFrom* functions in IoAbstraction for pin expander 2
#define SETUP_EXPANDER2_IOREF_ALLOC()   ioFrom8574((uint8_t)SETUP_EXPANDER2_IOREF_I2C_ADDR, (pinid_t)SETUP_EXPANDER2_IOREF_ISR_PIN, &SETUP_EXPANDER_IOREF_I2C_WIRE)

// Pin Muxer/Expander Channel Setup                     // (Note: Only multiplexing or expanding may be done at the same time)
#define SETUP_AC_USAGE_SENSOR_PINCHNL   hpinchnl_none   // AC power usage meter sensor pin muxer/expander channel #, else -127/none
#define SETUP_DC_USAGE_SENSOR_PINCHNL   hpinchnl_none   // DC power usage meter sensor pin muxer/expander channel #, else -127/none
#define SETUP_DC_GEN_SENSOR_PINCHNL     hpinchnl_none   // DC power generation meter sensor pin muxer/expander channel #, else -127/none
#define SETUP_ICE_INDICATOR_PINCHNL     hpinchnl_none   // Ice indicator pin muxer/expander channel #, else -127/none
#define SETUP_LINACT1_AXIS1_PINCHNLA     hpinchnl_none  // Ele/dec axis linear actuator #1 A pin muxer/expander channel #, else -127/none
#define SETUP_LINACT1_AXIS1_PINCHNLB     hpinchnl_none  // Ele/dec axis linear actuator #1 B pin muxer/expander channel #, else -127/none
#define SETUP_LINACT1_POS_SENSOR_PINCHNL hpinchnl_none  // Ele/dec axis linear actuator #1 position sensor pin muxer/expander channel #, else -127/none
#define SETUP_LINACT1_MIN_ENDSTOP_PINCHNL hpinchnl_none // Ele/dec axis linear actuator #1 min endstop pin muxer/expander channel #, else -127/none
#define SETUP_LINACT1_MAX_ENDSTOP_PINCHNL hpinchnl_none // Ele/dec axis linear actuator #1 max endstop pin muxer/expander channel #, else -127/none
#define SETUP_LINACT2_AXIS1_PINCHNLA     hpinchnl_none  // Ele/dec axis linear actuator #2 A pin muxer/expander channel #, else -127/none
#define SETUP_LINACT2_AXIS1_PINCHNLB     hpinchnl_none  // Ele/dec axis linear actuator #2 B pin muxer/expander channel #, else -127/none
#define SETUP_LINACT2_POS_SENSOR_PINCHNL hpinchnl_none  // Ele/dec axis linear actuator #2 position sensor pin muxer/expander channel #, else -127/none
#define SETUP_LINACT2_MIN_ENDSTOP_PINCHNL hpinchnl_none // Ele/dec axis linear actuator #2 min endstop pin muxer/expander channel #, else -127/none
#define SETUP_LINACT2_MAX_ENDSTOP_PINCHNL hpinchnl_none // Ele/dec axis linear actuator #2 max endstop pin muxer/expander channel #, else -127/none
#define SETUP_PANEL_BRAKE_PINCHNL       hpinchnl_none   // Panel brake relay pin muxer/expander channel #, else -127/none
#define SETUP_PANEL_HEATER_PINCHNL      hpinchnl_none   // Panel heater relay pin muxer/expander channel #, else -127/none
#define SETUP_PANEL_SPRAYER_PINCHNL     hpinchnl_none   // Panel sprayer/wiper relay pin muxer/expander channel #, else -127/none
#define SETUP_PANEL_TILT_AXIS1_PINCHNL  hpinchnl_none   // Ele/dec axis panel tilt angle sensor pin muxer/expander channel #, else -127/none
#define SETUP_POS_SERVO_AXIS0_PINCHNL   hpinchnl_none   // Azi/RA axis positional servo pin muxer/expander channel #, else -127/none
#define SETUP_POS_SERVO_AXIS1_PINCHNL   hpinchnl_none   // Ele/dec axis positional servo pin muxer/expander channel #, else -127/none
#define SETUP_WIND_SPEED_SENSOR_PINCHNL hpinchnl_none   // Wind speed sensor pin muxer/expander channel #, else -127/none

// System Setup
#define SETUP_AC_POWER_RAIL_TYPE        AC110V          // Rail power type used for actuator AC rail (AC110V, AC220V)
#define SETUP_DC_POWER_RAIL_TYPE        DC12V           // Rail power type used for actuator DC rail (DC3V3, DC5V, DC12V, DC24V, DC48V)
#define SETUP_AC_SUPPLY_POWER           0               // Maximum AC supply power wattage, else 0 if not known (-> use simple rails)
#define SETUP_DC_SUPPLY_POWER           0               // Maximum DC supply power wattage, else 0 if not known (-> use simple rails)
#define SETUP_PANEL_TYPE                Gimballed       // Panel/mount type (Horizontal, Vertical, Gimballed, Equatorial)
#define SETUP_PANEL_HOME                {0.0f,0.0f}     // Panel home axis position (azi,ele or RA,dec)
#define SETUP_PANEL_OFFSET              {0.0f,0.0f}     // Panel axis alignment offset (azi,ele or RA,dec)
#define SETUP_PANEL_HEATER_TEMP         0               // Temperature at which panel heaters are engaged (if using temp sensor), in Celsius
#define SETUP_PANEL_STORMING_SPEED      500             // Wind speed at which storming mode is engaged (if using wind sensor), in m/min
#define SETUP_LINACT_AXIS1_STROKE       1               // Stroke length of linear actuators on axis 1, in meters
#define SETUP_LINACT_TRAVEL_SPEED       1.0/0.5         // The base continuous linear actuator travel speed, in m/min.
#define SETUP_PANEL_TILT_AXIS1_SCALE    0,0 , 1,90      // Ele/dec axis panel tilt angle sensor scaling parameters used for angle calibration (passed to setFromTwoPoints), from raw into degrees
#define SETUP_POS_SERVO_MINMAX_ANGLE    -90, 90         // Axial positional servo min,max angle range used for angle calibration (passed to setFromServo), in degrees
#define SETUP_WIND_SPEED_SENSOR_SCALE   0.08,0 , 0.4,0.54 // Wind speed sensor scaling parameters used for speed calibration (passed to setFromTwoPoints), from raw into m/min
```

### Data Writer Example

The Data Writer Example can be used on the same system setup to offload all exportable data, such as string data, onto a connected external SD card or EEPROM storage device to help storage constrained MCUs compile system builds.

This Example doesn't actually run the Helioduino controller in full, but a code stripped version of it that easily compiles with all extra data built-in. The compiled binary in this Example will include all exportable data stored as compact JSON string text, and typically easily fits in under 256kB compilation size.

This data can further be exported into smaller binary chunks with native endianness for EEPROM storage, or into expanded pretty print JSON text files for SD card storage. You can also further customize default and/or extend library data that will be included in data export.

If you are planning to utilize an external EEPROM storage device and have made any custom data modifications, you will have to update your system's sketch to include the proper offsets that the output of the Data Writer sketch produces. These defines can be copied over and overwrite the existing ones in the main system examples.

In serial monitor (near end):
```
…
2023-02-07T04:24:27 [INFO] Writing String: #365 "W"
2023-02-07T04:24:27 [INFO] ... to byte offset: 11854 (0x2e4e)
2023-02-07T04:24:27 [INFO] Wrote: 2 bytes
2023-02-07T04:24:27 [INFO] Successfully wrote: 4908 bytes
2023-02-07T04:24:27 [INFO] Total EEPROM usage: 11856 bytes
2023-02-07T04:24:27 [INFO] EEPROM capacity used: 36.18% of 32768 bytes
2023-02-07T04:24:27 [INFO] Use the following EEPROM setup defines in your sketch:
#define SETUP_EEPROM_SYSDATA_ADDR       0x2222
#define SETUP_EEPROM_STRINGS_ADDR       0x0000
#define SETUP_EEPROM_UIDSTRS_ADDR       0x1111
2023-02-07T04:24:27 [INFO] Done!
```

Note: Again, you can get logging output sent to the Serial device by defining `HELIO_ENABLE_DEBUG_OUTPUT`, described above in Header Defines.

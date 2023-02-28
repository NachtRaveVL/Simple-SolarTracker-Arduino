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

From there, you can make a local copy of one of the examples based on the kind of system setup you want to use. If you are unsure of which, we recommend using the Dual Axis Tracking Example for older MCUs and the Full System Example for modern MCUs. Older storage constrained MCUs may need further modifications (and possibly external hardware) so is recommended only for advanced users.

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

There are two main system examples to choose from, Dual-Axis Tracking and Full System, each with its own requirements and capabilities. The Dual-Axis Tracking example is the recommended starting point for most system builds, and is perfect for those with only intermediate programming knowledge. It can also be easily extended to include other functionality if desired, simply by copying and pasting the example code.

This system code has the benefit of being able to compile out what you don't use, making it ideal for storage constrained devices, but will not provide full UI functionality since it will be missing the code for all the other objects the system build code strips out.

The Full System Example sketch will build an empty system with all object and system features enabled. It is recommended for modern MCUs that have lots of storage space to use such as ESP32, RasPi Pico, etc. It works similarly to the Dual-Axis Tracking Example, except is meant for systems where a GUI will be primarily used to create objects with (or done similarly in code to initialize, as is done in the Dual-Axis Tracking example). It involves the least amount of coding and setup, but comes at the highest cost.

Included below is the default system setup defines of the Dual-Axis Tracking example (of which a smaller similar version is used for the Full System example) to illustrate a variety of the controller features. This is not an exhaustive list of course, as there are many more things the controller is capable of, as documented in its main header file include, GitHub Project Wiki, and elsewhere.

```Arduino
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
#define SETUP_SYSTEM_MODE               Tracking // System run mode (Tracking, Balancing)
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
#define SETUP_EEPROM_SYSDATA_ADDR       0x2e50
#define SETUP_EEPROM_STRINGS_ADDR       0x0000
2023-02-07T04:24:27 [INFO] Done!
```

Note: Again, you can get logging output sent to the Serial device by defining `HELIO_ENABLE_DEBUG_OUTPUT`, described above in Header Defines.

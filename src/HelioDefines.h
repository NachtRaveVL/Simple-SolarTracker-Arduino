/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Defines
*/

#ifndef HelioDefines_H
#define HelioDefines_H

#ifndef FLT_EPSILON
#define FLT_EPSILON                     0.00001f            // Single-precision floating point error tolerance
#endif
#ifndef FLT_UNDEF
#define FLT_UNDEF                       __FLT_MAX__         // A floating point value to stand in for "undefined"
#endif
#ifndef DBL_EPSILON
#define DBL_EPSILON                     0.0000000000001     // Double-precision floating point error tolerance
#endif
#ifndef ENABLED
#define ENABLED                         0x1                 // Enabled define (convenience)
#endif
#ifndef DISABLED
#define DISABLED                        0x0                 // Disabled define (convenience)
#endif
#define ACTIVE_HIGH                     false               // Active high (convenience)
#define ACTIVE_ABOVE                    false               // Active above (convenience)
#define PULL_DOWN                       false               // Pull down (convenience)
#define ACTIVE_LOW                      true                // Active low (convenience)
#define ACTIVE_BELOW                    true                // Active below (convenience)
#define PULL_UP                         true                // Pull up (convenience)
#define RAW                             false               // Raw mode (convenience)
#define JSON                            true                // JSON mode (convenience)
#ifndef JOIN                                                // Define joiner
#define JOIN_(X,Y) X##_##Y
#define JOIN(X,Y) JOIN_(X,Y)
#endif
#ifndef RANDOM_MAX                                          // Missing random max
#ifdef RAND_MAX
#define RANDOM_MAX RAND_MAX
#else
#define RANDOM_MAX INTPTR_MAX
#endif
#endif
#if (defined(ESP32) || defined(ESP8266)) && !defined(ESP_PLATFORM) // Missing ESP_PLATFORM
#define ESP_PLATFORM
#endif
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_STM32)    // Missing min/max
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#if !defined(ADC_RESOLUTION) && defined(IOA_ANALOGIN_RES)   // Missing ADC resolution
#define ADC_RESOLUTION IOA_ANALOGIN_RES
#endif
#if !defined(ADC_RESOLUTION)
#define ADC_RESOLUTION 10
#endif
#if !defined(DAC_RESOLUTION) && defined(IOA_ANALOGOUT_RES)  // Missing DAC resolution
#define DAC_RESOLUTION IOA_ANALOGOUT_RES
#endif
#if !defined(DAC_RESOLUTION)
#define DAC_RESOLUTION 8
#endif
#ifndef F_SPD                                               // F_CPU/F_BUS alias for default SPI device speeds
#if defined(F_CPU)
#define F_SPD F_CPU
#elif defined(F_BUS)                                        // Teensy/etc support
#define F_SPD F_BUS
#else                                                       // Fast/good enough
#define F_SPD 50000000U
#endif
#endif

typedef typeof(millis()) millis_t;                          // Time millis type
typedef int8_t hposi_t;                                     // Position indexing type alias
typedef uint32_t hkey_t;                                    // Key type alias, for hashing
typedef uint16_t hframe_t;                                  // Polling frame type, for sync
typedef typeof(INPUT) ard_pinmode_t;                        // Arduino pin mode type alias
typedef typeof(LOW) ard_pinstatus_t;                        // Arduino pin status type alias

// The following slot sizes apply to all architectures
#define HELIO_NAME_MAXSIZE              32                  // Naming character maximum size (system name, crop name, etc.)
#define HELIO_POS_MAXSIZE               32                  // Position indicies maximum size (max # of objs of same type)
#define HELIO_URL_MAXSIZE               64                  // URL string maximum size (max url length)
#define HELIO_JSON_DOC_SYSSIZE          256                 // JSON document chunk data bytes for reading in main system data (serialization buffer size)
#define HELIO_JSON_DOC_DEFSIZE          192                 // Default JSON document chunk data bytes (serialization buffer size)
#define HELIO_STRING_BUFFER_SIZE        32                  // Size in bytes of string serialization buffers
#define HELIO_WIFISTREAM_BUFFER_SIZE    128                 // Size in bytes of WiFi serialization buffers
// The following max sizes only matter for architectures that do not have STL support
#define HELIO_ACTUATOR_SIGNAL_SLOTS     4                   // Maximum number of slots for actuator's activation signal
#define HELIO_SENSOR_SIGNAL_SLOTS       6                   // Maximum number of slots for sensor's measurement signal
#define HELIO_TRIGGER_SIGNAL_SLOTS      4                   // Maximum number of slots for trigger's state signal
#define HELIO_DRIVER_SIGNAL_SLOTS       2                   // Maximum number of slots for driver's state signal
#define HELIO_LOG_SIGNAL_SLOTS          2                   // Maximum number of slots for system log signal
#define HELIO_PUBLISH_SIGNAL_SLOTS      2                   // Maximum number of slots for data publish signal
#define HELIO_PANEL_SIGNAL_SLOTS        2                   // Maximum number of slots for various signals
#define HELIO_CAPACITY_SIGNAL_SLOTS     8                   // Maximum number of slots for rail capacity signal
#define HELIO_SYS_OBJECTS_MAXSIZE       16                  // Maximum array size for system objects (max # of objects in system)
#define HELIO_CAL_CALIBSTORE_MAXSIZE    8                   // Maximum array size for calibration store objects (max # of different custom calibrations)
#define HELIO_OBJ_LINKS_MAXSIZE         8                   // Maximum array size for object linkage list, per obj (max # of linked objects)
#define HELIO_DRV_ACTUATORS_MAXSIZE     8                   // Maximum array size for driver actuators list (max # of actuators used)
#define HELIO_SCH_TRACKING_MAXSIZE      4                   // Maximum array size for scheduler tracking process list (max # of panels)
#define HELIO_SCH_REQACTUATORS_MAXSIZE  4                   // Maximum array size for scheduler required actuators list (max # of actuators active per process stage)
#define HELIO_SYS_ONEWIRE_MAXSIZE       2                   // Maximum array size for pin OneWire list (max # of OneWire comm pins)
#define HELIO_SYS_PINLOCKS_MAXSIZE      2                   // Maximum array size for pin locks list (max # of locks)
#define HELIO_SYS_PINMUXERS_MAXSIZE     2                   // Maximum array size for pin muxers list (max # of muxers)

#define HELIO_CONTROL_LOOP_INTERVAL     100                 // Run interval of main control loop, in milliseconds
#define HELIO_DATA_LOOP_INTERVAL        2000                // Default run interval of data loop, in milliseconds (customizable later)
#define HELIO_MISC_LOOP_INTERVAL        250                 // Run interval of misc loop, in milliseconds

#define HELIO_ACT_TRAVELCALC_UPDATEMS   250                 // Minimum time millis needing to pass before a motor reports/writes changed position (reduces error accumulation)
#define HELIO_ACT_TRAVELCALC_MINSPEED   0.05f               // What percentage of continuous speed an instantaneous speed sensor must achieve before it is used in travel/distance calculations (reduces near-zero error jitters)

#define HELIO_OBJ_LINKSFILTER_DEFSIZE   8                   // Default array size for object linkage filtering

#define HELIO_POS_SEARCH_FROMBEG        -1                  // Search from beginning to end, 0 up to MAXSIZE-1
#define HELIO_POS_SEARCH_FROMEND        HELIO_POS_MAXSIZE   // Search from end to beginning, MAXSIZE-1 down to 0
#define HELIO_POS_EXPORT_BEGFROM        1                   // Whenever exported/user-facing position indexing starts at 1 or 0 (aka display offset)

#define HELIO_RANGE_TEMP_HALF           5.0f                // How far to go, in either direction, to form a range when Temp is expressed as a single number, in C (note: this also controls auto-balancer ranges)

#define HELIO_RAILS_LINKS_BASESIZE      4                   // Base array size for rail's linkage list

#define HELIO_SCH_BALANCE_MINTIME       30                  // Minimum time, in seconds, that all balancers must register as balanced for until driving is marked as completed

#define HELIO_SENSOR_ANALOGREAD_SAMPLES 5                   // Number of samples to take for any analogRead call inside of a sensor's takeMeasurement call, or 0 to disable sampling (note: bitRes.maxValue * # of samples must fit inside a uint32_t)
#define HELIO_SENSOR_ANALOGREAD_DELAY   0                   // Delay time between samples, or 0 to disable delay

#define HELIO_SYS_AUTOSAVE_INTERVAL     120                 // Default autosave interval, in minutes
#define HELIO_SYS_I2CEEPROM_BASEADDR    0x50                // Base address of I2C EEPROM (bitwise or'ed with passed address)
#define HELIO_SYS_ATWIFI_SERIALBAUD     115200              // Data baud rate for serial AT WiFi, in bps (older modules may need 9600)
#define HELIO_SYS_ATWIFI_SERIALMODE     SERIAL_8N1          // Data transfer mode for serial AT WiFi (see SERIAL_* defines)
#define HELIO_SYS_NMEAGPS_SERIALBAUD    9600                // Data baud rate for serial NMEA GPS, in bps (older modules may need 4800)
#define HELIO_SYS_URLHTTP_PORT          80                  // Which default port to access when accessing HTTP resources
#define HELIO_SYS_LEAVE_FILES_OPEN      !defined(__AVR__)   // If high access files should be left open to improve performance (true), or closed after use to reduce memory consumption (false)
#define HELIO_SYS_FREERAM_LOWBYTES      1024                // How many bytes of free memory left spawns a handle low mem call to all objects
#define HELIO_SYS_FREESPACE_INTERVAL    240                 // How many minutes should pass before checking attached file systems have enough disk space (performs cleanup if not)
#define HELIO_SYS_FREESPACE_LOWSPACE    256                 // How many kilobytes of disk space remaining will force cleanup of oldest log/data files first
#define HELIO_SYS_FREESPACE_DAYSBACK    180                 // How many days back log/data files are allowed to be stored up to (any beyond this are deleted during cleanup)
#define HELIO_SYS_DELAYFINE_SPINMILLIS  20                  // How many milliseconds away from stop time fine delays can use yield() up to before using a blocking spin-lock (used for fine timing)
#define HELIO_SYS_DEBUGOUT_FLUSH_YIELD  DISABLED            // If debug output statements should flush and yield afterwards to force send through to serial monitor (mainly used for debugging)
#define HELIO_SYS_MEM_LOGGING_ENABLE    DISABLED            // If system will periodically log memory remaining messages (mainly used for debugging)
#define HELIO_SYS_DRY_RUN_ENABLE        DISABLED            // Disables pins from actually enabling in order to simply simulate (mainly used for debugging)

#if defined(__APPLE__) || defined(__APPLE) || defined(__unix__) || defined(__unix)
#define HELIO_BLDPATH_SEPARATOR         '/'                 // Path separator for nix-based build machines
#else
#define HELIO_BLDPATH_SEPARATOR         '\\'                // Path separator for win-based build machines
#endif
#define HELIO_FSPATH_SEPARATOR          '/'                 // Path separator for filesystem paths (SD card/WiFiStorage)
#define HELIO_URLPATH_SEPARATOR         '/'                 // Path separator for URL paths

#if HELIO_SYS_LEAVE_FILES_OPEN                              // How subsequent getters should be called when file left open
#define HELIO_LOFS_BEGIN false
#else
#define HELIO_LOFS_BEGIN true
#endif


// EEPROM Device Type Enumeration
enum Helio_EEPROMType : signed short {
    Helio_EEPROMType_24LC01 = I2C_DEVICESIZE_24LC01 >> 7,   // 24LC01 (1K bits, 128 bytes), 7-bit address space
    Helio_EEPROMType_24LC02 = I2C_DEVICESIZE_24LC02 >> 7,   // 24LC02 (2K bits, 256 bytes), 8-bit address space
    Helio_EEPROMType_24LC04 = I2C_DEVICESIZE_24LC04 >> 7,   // 24LC04 (4K bits, 512 bytes), 9-bit address space
    Helio_EEPROMType_24LC08 = I2C_DEVICESIZE_24LC08 >> 7,   // 24LC08 (8K bits, 1024 bytes), 10-bit address space
    Helio_EEPROMType_24LC16 = I2C_DEVICESIZE_24LC16 >> 7,   // 24LC16 (16K bits, 2048 bytes), 11-bit address space
    Helio_EEPROMType_24LC32 = I2C_DEVICESIZE_24LC32 >> 7,   // 24LC32 (32K bits, 4096 bytes), 12-bit address space
    Helio_EEPROMType_24LC64 = I2C_DEVICESIZE_24LC64 >> 7,   // 24LC64 (64K bits, 8192 bytes), 13-bit address space
    Helio_EEPROMType_24LC128 = I2C_DEVICESIZE_24LC128 >> 7, // 24LC128 (128K bits, 16384 bytes), 14-bit address space
    Helio_EEPROMType_24LC256 = I2C_DEVICESIZE_24LC256 >> 7, // 24LC256 (256K bits, 32768 bytes), 15-bit address space
    Helio_EEPROMType_24LC512 = I2C_DEVICESIZE_24LC512 >> 7, // 24LC512 (512K bits, 65536 bytes), 16-bit address space
    Helio_EEPROMType_None = -1,                             // No EEPROM

    Helio_EEPROMType_1K_Bits = Helio_EEPROMType_24LC01,     // 1K bits (alias of 24LC01)
    Helio_EEPROMType_2K_Bits = Helio_EEPROMType_24LC02,     // 2K bits (alias of 24LC02)
    Helio_EEPROMType_4K_Bits = Helio_EEPROMType_24LC04,     // 4K bits (alias of 24LC04)
    Helio_EEPROMType_8K_Bits = Helio_EEPROMType_24LC08,     // 8K bits (alias of 24LC08)
    Helio_EEPROMType_16K_Bits = Helio_EEPROMType_24LC16,    // 16K bits (alias of 24LC16)
    Helio_EEPROMType_32K_Bits = Helio_EEPROMType_24LC32,    // 32K bits (alias of 24LC32)
    Helio_EEPROMType_64K_Bits = Helio_EEPROMType_24LC64,    // 64K bits (alias of 24LC64)
    Helio_EEPROMType_128K_Bits = Helio_EEPROMType_24LC128,  // 128K bits (alias of 24LC128)
    Helio_EEPROMType_256K_Bits = Helio_EEPROMType_24LC256,  // 256K bits (alias of 24LC256)
    Helio_EEPROMType_512K_Bits = Helio_EEPROMType_24LC512,  // 512K bits (alias of 24LC512)

    Helio_EEPROMType_128_Bytes = Helio_EEPROMType_24LC01,   // 128 bytes (alias of 24LC01)
    Helio_EEPROMType_256_Bytes = Helio_EEPROMType_24LC02,   // 256 bytes (alias of 24LC02)
    Helio_EEPROMType_512_Bytes = Helio_EEPROMType_24LC04,   // 512 bytes (alias of 24LC04)
    Helio_EEPROMType_1024_Bytes = Helio_EEPROMType_24LC08,  // 1024 bytes (alias of 24LC08)
    Helio_EEPROMType_2048_Bytes = Helio_EEPROMType_24LC16,  // 2048 bytes (alias of 24LC16)
    Helio_EEPROMType_4096_Bytes = Helio_EEPROMType_24LC32,  // 4096 bytes (alias of 24LC32)
    Helio_EEPROMType_8192_Bytes = Helio_EEPROMType_24LC64,  // 8192 bytes (alias of 24LC64)
    Helio_EEPROMType_16384_Bytes = Helio_EEPROMType_24LC128,// 16384 bytes (alias of 24LC128)
    Helio_EEPROMType_32768_Bytes = Helio_EEPROMType_24LC256,// 32768 bytes (alias of 24LC256)
    Helio_EEPROMType_65536_Bytes = Helio_EEPROMType_24LC512 // 65536 bytes (alias of 24LC512)
};

// RTC Device Type Enumeration
enum Helio_RTCType : signed char {
    Helio_RTCType_DS1307 = 13,                              // DS1307 (no battFail)
    Helio_RTCType_DS3231 = 32,                              // DS3231
    Helio_RTCType_PCF8523 = 85,                             // PCF8523
    Helio_RTCType_PCF8563 = 86,                             // PCF8563
    Helio_RTCType_None = -1                                 // No RTC
};

// DHT Device Type Enumeration
enum Helio_DHTType : signed char {
    Helio_DHTType_DHT11 = DHT11,                            // DHT11
    Helio_DHTType_DHT12 = DHT12,                            // DHT12
    Helio_DHTType_DHT21 = DHT21,                            // DHT21
    Helio_DHTType_DHT22 = DHT22,                            // DHT22
    Helio_DHTType_None = -1,                                // No DHT

    Helio_DHTType_AM2301 = AM2301,                          // AM2301 (alias of DHT21)
};


// System Run Mode
// Specifies the general solar setup, how orientation is determined, etc.
enum Helio_SystemMode : signed char {
    Helio_SystemMode_PositionCalc,                          // System will aim panels towards the sun's precise position in the sky as properly calculated (requires location/time, light/power sensing not required).
    Helio_SystemMode_SensorDependent,                       // System will aim panels towards a position based on driving/maximizing light/power sensor data (location/time not required, requires light/power sensing).

    Helio_SystemMode_Count,                                 // Placeholder
    Helio_SystemMode_Undefined = -1                         // Placeholder
};

// Measurement Units Mode
// Specifies the standard of measurement style that units will use.
enum Helio_MeasurementMode : signed char {
    Helio_MeasurementMode_Imperial,                         // Imperial measurement mode (default setting, °F Ft Gal Lbs M-D-Y Val.X etc)
    Helio_MeasurementMode_Metric,                           // Metric measurement mode (°C M L Kg Y-M-D Val.X etc)
    Helio_MeasurementMode_Scientific,                       // Scientific measurement mode (°K M L Kg Y-M-D Val.XX etc)

    Helio_MeasurementMode_Count,                            // Placeholder
    Helio_MeasurementMode_Undefined = -1,                   // Placeholder
    Helio_MeasurementMode_Default = Helio_MeasurementMode_Metric // Default system measurement mode
};

// LCD/Display Output Mode
// Specifies what kind of visual output device is to be used.
// Currently, all ouput devices must ultimately be supported by tcMenu.
enum Helio_DisplayOutputMode : signed char {
    Helio_DisplayOutputMode_Disabled,                       // No display output
    Helio_DisplayOutputMode_20x4LCD,                        // 20x4 i2c LCD (with layout: EN, RW, RS, BL, Data)
    Helio_DisplayOutputMode_20x4LCD_Swapped,                // 20x4 i2c LCD (with EN<->RS swapped, layout: RS, RW, EN, BL, Data)
    Helio_DisplayOutputMode_16x2LCD,                        // 16x2 i2c LCD (with layout: EN, RW, RS, BL, Data)
    Helio_DisplayOutputMode_16x2LCD_Swapped,                // 16x2 i2c LCD (with EN<->RS swapped, layout: RS, RW, EN, BL, Data)

    Helio_DisplayOutputMode_Count,                          // Placeholder
    Helio_DisplayOutputMode_Undefined = -1                  // Placeholder
};

// Control Input Mode
// Specifies what kind of control input mode is to be used.
// Currently, all input devices must ultimately be supported by tcMenu.
enum Helio_ControlInputMode : signed char {
    Helio_ControlInputMode_Disabled,                        // No control input
    Helio_ControlInputMode_2x2Matrix,                       // 2x2 directional keyboard matrix button array, ribbon: {L1,L2,R1,R2} (L1 = pin 1)
    Helio_ControlInputMode_4xButton,                        // 4x standard momentary buttons, ribbon: {U,D,L,R} (U = pin 1)
    Helio_ControlInputMode_6xButton,                        // 6x standard momentary buttons, ribbon: {TODO} (X = pin 1)
    Helio_ControlInputMode_RotaryEncoder,                   // Rotary encoder, ribbon: {A,B,OK,L,R} (A = pin 1)

    Helio_ControlInputMode_Count,                           // Placeholder
    Helio_ControlInputMode_Undefined = -1                   // Placeholder
};

// Actuator Type
// Control actuator type. Specifies the various controllable equipment and their usage.
enum Helio_ActuatorType : signed char {
    Helio_ActuatorType_PanelCover,                          // Panel cover actuator
    Helio_ActuatorType_PanelHeater,                         // Panel heater actuator
    Helio_ActuatorType_PanelCleaner,                        // Panel cleaner actuator
    Helio_ActuatorType_LinearActuator,                      // Panel axis linear actuator
    Helio_ActuatorType_RotaryServo,                         // Panel axis rotary servo

    Helio_ActuatorType_Count,                               // Placeholder
    Helio_ActuatorType_Undefined = -1                       // Placeholder
};

// Sensor Type
// Sensor device type. Specifies the various sensors and the kinds of things they measure.
enum Helio_SensorType : signed char {
    Helio_SensorType_LightIntensity,                        // Light dependent resistor (LDR, analog)
    Helio_SensorType_PowerUsage,                            // Power usage meter (analog)
    Helio_SensorType_PowerProduction,                       // Power production meter (analog)
    Helio_SensorType_TemperatureHumidity,                   // Temperature and humidity sensor (digital)
    Helio_SensorType_IceDetector,                           // Ice detector (binary/analog)
    Helio_SensorType_WindSpeed,                             // Wind speed sensor (binary/analog)
    Helio_SensorType_Endstop,                               // Track axis endstop (binary)
    Helio_SensorType_StrokePosition,                        // Actuator stroke position potentiometer (analog)

    Helio_SensorType_Count,                                 // Placeholder
    Helio_SensorType_Undefined = -1                         // Placeholder
};

// Panel Type
// Common panel types. Specifies the various solar panel axis/mount configurations.
enum Helio_PanelType : signed char {
    Helio_PanelType_Horizontal,                             // Single axis horizontal / tilting panel (elevation)
    Helio_PanelType_Vertical,                               // Single axis vertical / facing panel (azimuth)
    Helio_PanelType_Gimballed,                              // Dual axis gimballed panel (azimuth + elevation)
    Helio_PanelType_Equatorial,                             // Dual axis equatorial mounted panel (right-ascension + declination, external polar alignment required)

    Helio_PanelType_Count,                                  // Placeholder
    Helio_PanelType_Undefined = -1                          // Placeholder
};

// Power Rail
// Common power rails. Specifies an isolated operational power rail unit.
enum Helio_RailType : signed char {
    Helio_RailType_AC110V,                                  // 110~120V AC-based power rail, for heaters, etc.
    Helio_RailType_AC220V,                                  // 110~120V AC-based power rail, for heaters, etc.
    Helio_RailType_DC5V,                                    // 5v DC-based power rail, for sensors, etc.
    Helio_RailType_DC12V,                                   // 12v DC-based power rail, for actuators, etc.

    Helio_RailType_Count,                                   // Placeholder
    Helio_RailType_Undefined = -1                           // Placeholder
};

// Pin Mode
// Pin mode setting. Specifies what kind of pin and how it's used.
enum Helio_PinMode : signed char {
    Helio_PinMode_Digital_Input_PullUp,                     // Digital input pin with pull-up resistor enabled input (default pairing for active-low trigger)
    Helio_PinMode_Digital_Input_PullDown,                   // Digital input pin with pull-down resistor enabled input (or pull-up disabled if not avail, default pairing for active-high trigger)
    Helio_PinMode_Digital_Input_Floating,                   // Digital input pin with floating/disabled input (pull-up/pull-down disabled, used during mux channel select)
    Helio_PinMode_Digital_Output_OpenDrain,                 // Digital output pin with open-drain NPN-based sink (default pairing for active-low trigger)
    Helio_PinMode_Digital_Output_PushPull,                  // Digital output pin with push-pull NPN+PNP-based src+sink (default pairing for active-high trigger)
    Helio_PinMode_Analog_Input,                             // Analog input pin
    Helio_PinMode_Analog_Output,                            // Analog output pin

    Helio_PinMode_Count,                                    // Placeholder
    Helio_PinMode_Undefined = -1                            // Placeholder
};

// Trigger Status
// Common trigger statuses. Specifies enablement and tripped state.
enum Helio_TriggerState : signed char {
    Helio_TriggerState_Disabled,                            // Triggers disabled (not hooked up)
    Helio_TriggerState_NotTriggered,                        // Not triggered
    Helio_TriggerState_Triggered,                           // Triggered

    Helio_TriggerState_Count,                               // Placeholder
    Helio_TriggerState_Undefined = -1                       // Placeholder
};

// Driving State
// Common driving states. Specifies balance or which direction of imbalance.
enum Helio_DrivingState : signed char {
    Helio_DrivingState_GoHigher,                            // Need to go higher / too low
    Helio_DrivingState_OnTarget,                            // On target
    Helio_DrivingState_GoLower,                             // Need to go lower / too high

    Helio_DrivingState_Count,                               // Placeholder
    Helio_DrivingState_Undefined = -1                       // Placeholder
};

// Panel State
// Common panel states. Specifies panel alignment and .
enum Helio_PanelState : signed char {
    Helio_PanelState_AlignedToSun,                          // Aligned to sun
    Helio_PanelState_TravelingToSun,                        // Traveling to sun
    Helio_PanelState_TravelingToHome,                       // Traveling to home position
    Helio_PanelState_AlignedToHome,                         // Aligned to home position/stowed

    Helio_PanelState_Count,                                 // Placeholder
    Helio_PanelState_Undefined = -1                         // Placeholder
};

// Enable Mode
// Actuator intensity/enablement calculation mode. Specifies how multiple activations get used together.
enum Helio_EnableMode : signed char {
    Helio_EnableMode_Highest,                               // Parallel activation using highest drive intensity
    Helio_EnableMode_Lowest,                                // Parallel activation using lowest drive intensity
    Helio_EnableMode_Average,                               // Parallel activation using averaged drive intensities
    Helio_EnableMode_Multiply,                              // Parallel activation using multiplied drive intensities

    Helio_EnableMode_InOrder,                               // Serial activation using in-order/fifo-queue drive intensities
    Helio_EnableMode_RevOrder,                              // Serial activation using reverse-order/lifo-stack drive intensities
    Helio_EnableMode_DesOrder,                              // Serial activation using highest-to-lowest/descending-order drive intensities
    Helio_EnableMode_AscOrder,                              // Serial activation using lowest-to-highest/ascending-order drive intensities

    Helio_EnableMode_Count,                                 // Placeholder
    Helio_EnableMode_Undefined = -1,                        // Placeholder
    Helio_EnableMode_Serial = Helio_EnableMode_InOrder      // Serial (alias for in-order)
};

// Direction Mode
// Actuator intensity application mode. Specifies activation directionality and enablement.
enum Helio_DirectionMode : signed char {
    Helio_DirectionMode_Forward,                            // Standard/forward direction mode
    Helio_DirectionMode_Reverse,                            // Opposite/reverse direction mode
    Helio_DirectionMode_Stop,                               // Stationary/braking direction mode

    Helio_DirectionMode_Count,                              // Placeholder
    Helio_DirectionMode_Undefined = -1                      // Placeholder
};

// Units Category
// Unit of measurement category. Specifies the kind of unit.
enum Helio_UnitsCategory : signed char {
    Helio_UnitsCategory_Temperature,                        // Temperature based unit
    Helio_UnitsCategory_Humidity,                           // Humidity based unit
    Helio_UnitsCategory_HeatIndex,                          // Heat index based unit
    Helio_UnitsCategory_Distance,                           // Distance/position based unit
    Helio_UnitsCategory_Speed,                              // Speed based unit
    Helio_UnitsCategory_Power,                              // Power based unit

    Helio_UnitsCategory_Count,                              // Placeholder
    Helio_UnitsCategory_Undefined = -1                      // Placeholder
};

// Units Type
// Unit of measurement type. Specifies the unit type associated with a measured value.
enum Helio_UnitsType : signed char {
    Helio_UnitsType_Raw_0_1,                                // Raw value [0.0,1.0] mode
    Helio_UnitsType_Percentile_0_100,                       // Percentile [0.0,100.0] mode
    Helio_UnitsType_Angle_0_360,                            // Angle [0.0,360.0) mode
    Helio_UnitsType_Temperature_Celsius,                    // Celsius temperature mode
    Helio_UnitsType_Temperature_Fahrenheit,                 // Fahrenheit temperature mode
    Helio_UnitsType_Temperature_Kelvin,                     // Kelvin temperature mode
    Helio_UnitsType_Distance_Meters,                        // Meters distance mode
    Helio_UnitsType_Distance_Feet,                          // Feet distance mode
    Helio_UnitsType_Speed_MetersPerMin,                     // Meters per minute speed mode
    Helio_UnitsType_Speed_FeetPerMin,                       // Feet per minute speed mode
    Helio_UnitsType_Power_Wattage,                          // Wattage power mode
    Helio_UnitsType_Power_Amperage,                         // Amperage current power mode

    Helio_UnitsType_Count,                                  // Placeholder
    Helio_UnitsType_Power_JoulesPerSecond = Helio_UnitsType_Power_Wattage, // Joules per second power mode alias
    Helio_UnitsType_Undefined = -1                          // Placeholder
};

// Common forward decls
class Helioduino;
class HelioScheduler;
class HelioLogger;
class HelioPublisher;
struct HelioIdentity;
struct HelioData;
struct HelioSubData;
struct HelioObjectData;
struct HelioPin;
struct HelioDigitalPin;
struct HelioAnalogPin;
struct HelioActivationHandle;
struct HelioMeasurement;
struct HelioSingleMeasurement;
class HelioObject;
class HelioSubObject;
class HelioDLinkObject;
class HelioAttachment;
class HelioActuatorAttachment;
class HelioSensorAttachment;
class HelioTriggerAttachment;
class HelioDriverAttachment;
class HelioTrigger;
class HelioDriver;
class HelioActuator;
class HelioSensor;
class HelioPanel;
class HelioRail;

#endif // /ifndef HelioDefines_H

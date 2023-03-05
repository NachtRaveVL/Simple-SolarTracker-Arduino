/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Defines
*/

#ifndef HelioDefines_H
#define HelioDefines_H

#ifndef JOIN                                                // Define joiner
#define JOIN_(X,Y) X##_##Y
#define JOIN(X,Y) JOIN_(X,Y)
#endif

#define ACTIVE_HIGH                     false               // Active high (convenience)
#define ACTIVE_ABOVE                    false               // Active above (convenience)
#define ACTIVE_LOW                      true                // Active low (convenience)
#define ACTIVE_BELOW                    true                // Active below (convenience)
#define PULL_DOWN                       false               // Pull down (convenience)
#define PULL_UP                         true                // Pull up (convenience)
#define RAW                             false               // Raw mode (convenience)
#define JSON                            true                // JSON mode (convenience)
#ifndef ENABLED
#define ENABLED                         0x1                 // Enabled define (convenience)
#endif
#ifndef DISABLED
#define DISABLED                        0x0                 // Disabled define (convenience)
#endif
#ifndef FLT_EPSILON
#define FLT_EPSILON                     0.00001f            // Single-precision floating point error tolerance
#endif
#ifndef DBL_EPSILON
#define DBL_EPSILON                     0.0000000000001     // Double-precision floating point error tolerance
#endif
#ifndef FLT_UNDEF
#define FLT_UNDEF                       __FLT_MAX__         // Single-precision floating point value to stand in for "undefined"
#endif
#ifndef DBL_UNDEF
#define DBL_UNDEF                       __DBL_MAX__         // Double-precision floating point value to stand in for "undefined"
#endif
#ifndef MIN_PER_DAY
#define MIN_PER_DAY                     1440                // Minutes per day
#endif
#ifndef TWO_PI                                              // Two pi
#define TWO_PI                          6.283185307179586476925286766559
#endif

// Platform standardizations
#if (defined(ESP32) || defined(ESP8266)) && !defined(ESP_PLATFORM) // ESP_PLATFORM for any esp
#define ESP_PLATFORM
#endif
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_STM32)    // Missing min/max
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef RANDOM_MAX                                          // Missing RANDOM_MAX
#if defined(RAND_MAX)
#define RANDOM_MAX                      RAND_MAX
#else
#define RANDOM_MAX                      __INT_MAX__
#endif
#endif

#ifndef ADC_RESOLUTION                                      // Resolving ADC resolution, or define manually by build define (see BOARD for example)
#if defined(IOA_ANALOGIN_RES)                               // From IOAbstraction
#define ADC_RESOLUTION IOA_ANALOGIN_RES
#else
#define ADC_RESOLUTION 10                                   // Default per AVR
#endif
#endif
#ifndef DAC_RESOLUTION                                      // Resolving DAC resolution, or define manually by build define (see BOARD for example)
#if defined(IOA_ANALOGOUT_RES)                              // From IOAbstraction
#define DAC_RESOLUTION IOA_ANALOGOUT_RES
#else
#define DAC_RESOLUTION 8                                    // Default per AVR
#endif
#endif
#ifndef F_SPD                                               // Resolving F_SPD=F_CPU|F_BUS alias (for default SPI device speeds), or define manually by build define (see BOARD for example)
#if defined(F_CPU)
#define F_SPD F_CPU
#elif defined(F_BUS)                                        // Teensy/etc support
#define F_SPD F_BUS
#else                                                       // Fast/good enough (32MHz)
#define F_SPD 32000000U
#endif
#endif
#ifndef V_MCU                                               // Resolving MCU voltage, or define manually by build define (see BOARD for example)
#if (defined(__AVR__) || defined(__STM8__)) && !defined(ARDUINO_AVR_FIO) && !(defined(ARDUINO_AVR_PRO) && F_CPU == 8000000L)
#define V_MCU                           5.0                 // 5v MCU (assumed/tolerant)
#else
#define V_MCU                           3.3                 // 3v3 MCU (assumed)
#endif
#endif
#ifndef BOARD                                               // Resolving board name alias, or define manually by build define via creating platform.local.txt in %applocaldata%\Arduino15\packages\{platform}\hardware\{arch}\{version}\ containing (/w quotes): compiler.cpp.extra_flags="-DBOARD={build.board}"
#if defined(TEENSYDUINO)                                    // For Teensy, define manually by build define via editing platform.txt directly in %applocaldata%\Arduino15\packages\teensy\hardware\avr\{version}\ and adding (/w space & quotes):  "-DBOARD={build.board}" to end of recipe.cpp.o.pattern=
#define BOARD "TEENSY"
#elif defined(ARDUINO_BOARD)
#define BOARD ARDUINO_BOARD
#elif defined(BOARD_NAME)
#define BOARD BOARD_NAME
#elif defined(USB_PRODUCT)
#define BOARD USB_PRODUCT
#else
#define BOARD "OTHER"
#endif
#endif

typedef typeof(millis())                millis_t;           // Millisecond time type
typedef int8_t                          hposi_t;            // Position indexing type alias
typedef uint32_t                        hkey_t;             // Key type alias, for hashing
typedef int8_t                          hid_t;              // Id type alias, for RTTI
typedef uint16_t                        hframe_t;           // Polling frame type, for sync
#define millis_none                     ((millis_t)0)       // No millis defined/invalid placeholder
#define hposi_none                      ((hposi_t)-1)       // No position defined/invalid placeholder
#define hkey_none                       ((hkey_t)-1)        // No key defined/invalid placeholder
#define hid_none                        ((hid_t)-1)         // No id defined/invalid placeholder
#define hframe_none                     ((hframe_t)0)       // No frame defined/invalid placeholder
typedef typeof(INPUT)                   ard_pinmode_t;      // Arduino pin mode type alias
typedef typeof(LOW)                     ard_pinstatus_t;    // Arduino pin status type alias

// The following sizes apply to all architectures
#define HELIO_PREFIX_MAXSIZE            16                  // Prefix names maximum size (for logging/publishing)
#define HELIO_NAME_MAXSIZE              24                  // Naming character maximum size (system name, panel name, etc.)
#define HELIO_POS_MAXSIZE               32                  // Position indicies maximum size (max # of objs of same type)
#define HELIO_URL_MAXSIZE               64                  // URL string maximum size (max url length)
#define HELIO_JSON_DOC_SYSSIZE          256                 // JSON document chunk data bytes for reading in main system data (serialization buffer size)
#define HELIO_JSON_DOC_DEFSIZE          192                 // Default JSON document chunk data bytes (serialization buffer size)
#define HELIO_STRING_BUFFER_SIZE        32                  // Size in bytes of string serialization buffers
#define HELIO_WIFISTREAM_BUFFER_SIZE    128                 // Size in bytes of WiFi serialization buffers
// The following sizes only matter for architectures that do not have STL support
#define HELIO_DEFAULT_MAXSIZE           8                   // Default maximum array/map size
#define HELIO_ACTUATOR_SIGNAL_SLOTS     4                   // Maximum number of slots for actuator's activation signal
#define HELIO_SENSOR_SIGNAL_SLOTS       6                   // Maximum number of slots for sensor's measurement signal
#define HELIO_TRIGGER_SIGNAL_SLOTS      4                   // Maximum number of slots for trigger's state signal
#define HELIO_DRIVER_SIGNAL_SLOTS       2                   // Maximum number of slots for driver's state signal
#define HELIO_LOG_SIGNAL_SLOTS          2                   // Maximum number of slots for system log signal
#define HELIO_PUBLISH_SIGNAL_SLOTS      2                   // Maximum number of slots for data publish signal
#define HELIO_PANEL_SIGNAL_SLOTS        2                   // Maximum number of slots for various signals
#define HELIO_RAIL_SIGNAL_SLOTS         8                   // Maximum number of slots for rail capacity signal
#define HELIO_SYS_OBJECTS_MAXSIZE       16                  // Maximum array size for system objects (max # of objects in system)
#define HELIO_CAL_CALIBS_MAXSIZE        8                   // Maximum array size for calibration store objects (max # of different custom calibrations)
#define HELIO_OBJ_LINKS_MAXSIZE         8                   // Maximum array size for object linkage list, per obj (max # of linked objects)
#define HELIO_DRV_ACTUATORS_MAXSIZE     8                   // Maximum array size for driver actuators list (max # of actuators used)
#define HELIO_SCH_PROCS_MAXSIZE         4                   // Maximum array size for scheduler tracking process list (max # of panels)
#define HELIO_SCH_REQACTS_MAXSIZE       4                   // Maximum array size for scheduler required actuators list (max # of actuators active per process stage)
#define HELIO_SYS_ONEWIRES_MAXSIZE      2                   // Maximum array size for pin OneWire list (max # of OneWire comm pins)
#define HELIO_SYS_PINLOCKS_MAXSIZE      2                   // Maximum array size for pin locks list (max # of locks)
#define HELIO_SYS_PINMUXERS_MAXSIZE     2                   // Maximum array size for pin muxers list (max # of muxers)

#define HELIO_CONTROL_LOOP_INTERVAL     100                 // Run interval of main control loop, in milliseconds
#define HELIO_DATA_LOOP_INTERVAL        2000                // Default run interval of data loop, in milliseconds (customizable later)
#define HELIO_MISC_LOOP_INTERVAL        250                 // Run interval of misc loop, in milliseconds

#define HELIO_ACT_TRAVELCALC_UPDATEMS   250                 // Minimum time millis needing to pass before a motor reports/writes changed position (reduces error accumulation)
#define HELIO_ACT_TRAVELCALC_MINSPEED   0.05f               // What percentage of continuous speed an instantaneous speed sensor must achieve before it is used in travel/distance calculations (reduces near-zero error jitters)

#define HELIO_DRV_FINETRAVEL_RATEMULT   0.5f                // Fine travel movement rate multiplier used on activations when actuator is within fine alignment distance with target.

#define HELIO_NIGHT_START_HR            20                  // Hour of the day night starts (for resting panels, used if not able to calculate from lat/long/date)
#define HELIO_NIGHT_FINISH_HR           6                   // Hour of the day night finishes (for resting panels, used if not able to calculate from lat/long/date)

#define HELIO_PANEL_LINKS_BASESIZE      4                   // Base array size for panel's linkage list 
#define HELIO_PANEL_ALIGN_DEGTOL        2.5f                // Default degrees error tolerance for panel alignment queries
#define HELIO_PANEL_ALIGN_LDRTOL        0.05f               // Default LDR intensity balancing tolerance for panel alignment queries
#define HELIO_PANEL_ALIGN_LDRMIN        0.20f               // Default LDR intensity minimum needed for panel alignment queries

#define HELIO_POS_SEARCH_FROMBEG        -1                  // Search from beginning to end, 0 up to MAXSIZE-1
#define HELIO_POS_SEARCH_FROMEND        HELIO_POS_MAXSIZE   // Search from end to beginning, MAXSIZE-1 down to 0
#define HELIO_POS_EXPORT_BEGFROM        1                   // Whenever exported/user-facing position indexing starts at 1 or 0 (aka display offset)

#define HELIO_RANGE_TEMP_HALF           5.0f                // How far to go, in either direction, to form a range when Temp is expressed as a single number, in C (note: this also controls auto-balancer ranges)

#define HELIO_RAILS_LINKS_BASESIZE      4                   // Base array size for rail's linkage list
#define HELIO_RAILS_FRACTION_SATURATED  0.8f                // What fraction of maximum power is allowed to be used in canActivate() checks (aka maximum saturation point), used in addition to regulated rail's limitTrigger

#define HELIO_SCH_BALANCE_MINTIME       30                  // Minimum time, in seconds, that all balancers must register as balanced for until driving is marked as completed

#define HELIO_SENSOR_ANALOGREAD_SAMPLES 5                   // Number of samples to take for any analogRead call inside of a sensor's takeMeasurement call, or 0 to disable sampling (note: bitRes.maxValue * # of samples must fit inside a uint32_t)
#define HELIO_SENSOR_ANALOGREAD_DELAY   0                   // Delay time between samples, or 0 to disable delay, in milliseconds

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
#define HELIO_SYS_SUNRISESET_CALCITERS  3                   // # of iterations that sunrise/sunset calculations should run (higher # = more accurate but also more costly)
#define HELIO_SYS_LATLONG_DISTSQRDTOL   0.25                // Squared difference in lat/long coords that needs to occur for it to be considered significant enough for system update
#define HELIO_SYS_ALTITUDE_DISTTOL      0.5                 // Difference in altitude coords that needs to occur for it to be considered significant enough for system update
#define HELIO_SYS_DELAYFINE_SPINMILLIS  20                  // How many milliseconds away from stop time fine delays can use yield() up to before using a blocking spin-lock (used for fine timing)
#define HELIO_SYS_YIELD_AFTERMILLIS     20                  // How many milliseconds must pass by before system run loops call a yield() mid-loop, in order to allow finely-timed tasks a chance to run
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
    Helio_SystemMode_Tracking,                              // System will aim panels towards the sun's precise position in the sky as properly calculated (requires location/time, light/power sensing not required).
    Helio_SystemMode_Balancing,                             // System will aim panels towards a position based on driving/maximizing light/power sensor data (location/time not required, requires light/power sensing).

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
    Helio_MeasurementMode_Default = Helio_MeasurementMode_Metric // Default system measurement mode (alias, feel free to change)
};

// LCD/Display Output Mode
// Specifies what kind of visual output device is to be used.
// Currently, all ouput devices must ultimately be supported by tcMenu.
enum Helio_DisplayOutputMode : signed char {
    Helio_DisplayOutputMode_Disabled,                       // No display output
    Helio_DisplayOutputMode_16x2LCD,                        // 16x2 text LCD (with pins: {EN,RW,RS,BL,Data})
    Helio_DisplayOutputMode_16x2LCD_Swapped,                // 16x2 text LCD (with EN<->RS swapped, pins: {RS,RW,EN,BL,Data})
    Helio_DisplayOutputMode_20x4LCD,                        // 20x4 text LCD (with pins: {EN,RW,RS,BL,Data})
    Helio_DisplayOutputMode_20x4LCD_Swapped,                // 20x4 text LCD (with EN<->RS swapped, pins: {RS,RW,EN,BL,Data})
    Helio_DisplayOutputMode_SSD1305,                        // SSD1305 128x32 graphical LCD, requires U8G2
    Helio_DisplayOutputMode_SSD1305_x32Ada,                 // SSD1305 128x32 graphical LCD, using Adafruit + U8G2
    Helio_DisplayOutputMode_SSD1305_x64Ada,                 // SSD1305 128x64 graphical LCD, using Adafruit + U8G2
    Helio_DisplayOutputMode_SSD1306,                        // SSD1306 128x64 graphical LCD, using U8G2
    Helio_DisplayOutputMode_SH1106,                         // SH1106 128x64 graphical LCD, using U8G2
    Helio_DisplayOutputMode_SSD1607,                        // SSD1607 200x200 graphical LCD, using U8G2
    Helio_DisplayOutputMode_IL3820,                         // IL3820 296x128 graphical LCD, using U8G2
    Helio_DisplayOutputMode_IL3820_V2,                      // IL3820 V2 296x128 graphical LCD, using U8G2
    Helio_DisplayOutputMode_ST7735,                         // ST7735 320x240 graphical LCD, using AdafruitGFX
    Helio_DisplayOutputMode_ST7735_TFT,                     // ST7735 320x240 graphical LCD, using TFT_eSPI
    Helio_DisplayOutputMode_ST7789,                         // ST7789 320x240 graphical LCD, using AdafruitGFX
    Helio_DisplayOutputMode_ST7789_TFT,                     // ST7789 320x240 graphical LCD, using TFT_eSPI
    Helio_DisplayOutputMode_ILI9341,                        // ILI9341 320x240 graphical LCD, using AdafruitGFX
    Helio_DisplayOutputMode_ILI9341_TFT,                    // ILI9341 320x240 graphical LCD, using TFT_eSPI
    Helio_DisplayOutputMode_PCD8544,                        // PCD8544 320x240 graphical LCD, using AdafruitGFX
    Helio_DisplayOutputMode_PCD8544_TFT,                    // PCD8544 320x240 graphical LCD, using TFT_eSPI
    Helio_DisplayOutputMode_Nokia5110,                      // Nokia5110 320x240 graphical LCD, using AdafruitGFX
    Helio_DisplayOutputMode_Nokia5110_TFT,                  // Nokia5110 320x240 graphical LCD, using TFT_eSPI

    Helio_DisplayOutputMode_Count,                          // Placeholder
    Helio_DisplayOutputMode_Undefined = -1                  // Placeholder
};

// Control Input Mode
// Specifies what kind of control input mode is to be used.
// Currently, all input devices must ultimately be supported by tcMenu.
enum Helio_ControlInputMode : signed char {
    Helio_ControlInputMode_Disabled,                        // No control input
    Helio_ControlInputMode_RotaryEncoder,                   // Rotary encoder, pins: {A,B} (A = pin 1)
    Helio_ControlInputMode_RotaryEncoder_Ok,                // Rotary encoder /w ok button, pins: {A,B,OK} (A = pin 1)
    Helio_ControlInputMode_RotaryEncoder_OkLR,              // Rotary encoder /w ok and l/r buttons, pins: {A,B,OK,L,R} (A = pin 1)
    Helio_ControlInputMode_2x2Matrix,                       // 2x2 directional keyboard matrix button array, pins: {L1,L2,R1,R2} (L1 = pin 1)
    Helio_ControlInputMode_2x2Matrix_Ok,                    // 2x2 directional keyboard matrix button array /w ok button, pins: {L1,L2,R1,R2,OK} (L1 = pin 1)
    Helio_ControlInputMode_Joystick,                        // Analog joystick, pins: {X,Y}
    Helio_ControlInputMode_Joystick_Ok,                     // Analog joystick /w ok button, pins: {X,Y,OK}
    Helio_ControlInputMode_3x4Matrix,                       // 3x4 keyboard matrix (graphical), /w optional rotary encoder
    Helio_ControlInputMode_3x4Matrix_Ok,                    // 3x4 keyboard matrix (graphical), /w optional rotary encoder /w ok button
    Helio_ControlInputMode_3x4Matrix_OkLR,                  // 3x4 keyboard matrix (graphical), /w optional rotary encoder /w ok and l/r buttons
    Helio_ControlInputMode_ResistiveTouch,                  // Resistive touchscreen, pins: {X+,X-,Y+,Y-}
    Helio_ControlInputMode_TouchScreen,                     // Full touchscreen (FT6206/XPT2046), pins: {}

    Helio_ControlInputMode_Count,                           // Placeholder
    Helio_ControlInputMode_Undefined = -1                   // Placeholder
};

// Actuator Type
// Control actuator type. Specifies the various controllable equipment and their usage.
enum Helio_ActuatorType : signed char {
    Helio_ActuatorType_ContinuousServo,                     // Continuous servo (motor)
    Helio_ActuatorType_LinearActuator,                      // Linear actuator (motor)
    Helio_ActuatorType_PanelBrake,                          // Panel brake/stops (binary)
    Helio_ActuatorType_PanelCover,                          // Panel cover (binary/analog/motor)
    Helio_ActuatorType_PanelHeater,                         // Panel heater (binary/analog)
    Helio_ActuatorType_PanelSprayer,                        // Panel sprayer/cleaner (binary/analog)
    Helio_ActuatorType_PositionalServo,                     // Positional servo (analog)

    Helio_ActuatorType_Count,                               // Placeholder
    Helio_ActuatorType_Undefined = -1                       // Placeholder
};

// Sensor Type
// Sensor device type. Specifies the various sensors and the kinds of things they measure.
enum Helio_SensorType : signed char {
    Helio_SensorType_IceDetector,                           // Ice detector/meter (binary/analog)
    Helio_SensorType_LightIntensity,                        // Light dependent resistor (analog)
    Helio_SensorType_PowerProduction,                       // Power production level meter (analog)
    Helio_SensorType_PowerUsage,                            // Power usage level meter (analog)
    Helio_SensorType_TemperatureHumidity,                   // Temperature sensor (analog), optionally with humidity (digital)
    Helio_SensorType_TiltAngle,                             // Tilt angle sensor (analog)
    Helio_SensorType_TravelPosition,                        // Travel position sensor (binary/analog)
    Helio_SensorType_WindSpeed,                             // Wind speed sensor (binary/analog)

    Helio_SensorType_Count,                                 // Placeholder
    Helio_SensorType_Undefined = -1                         // Placeholder
};

// Panel Type
// Common panel types. Specifies the various solar panel axis/mount configurations.
enum Helio_PanelType : signed char {
    Helio_PanelType_Horizontal,                             // Single axis horizontal / tilting panel (elevation-driven)
    Helio_PanelType_Vertical,                               // Single axis vertical / facing panel (azimuth-driven)
    Helio_PanelType_Gimballed,                              // Dual axis gimballed panel (azimuth + elevation)
    Helio_PanelType_Equatorial,                             // Dual axis equatorial mounted panel (right-ascension + declination, external polar alignment required)

    Helio_PanelType_Count,                                  // Placeholder
    Helio_PanelType_Undefined = -1                          // Placeholder
};

// Power Rail
// Common power rails. Specifies an isolated operational power rail unit.
enum Helio_RailType : signed char {
    Helio_RailType_AC110V,                                  // ~110V AC-based power rail
    Helio_RailType_AC220V,                                  // ~220V AC-based power rail
    Helio_RailType_DC3V3,                                   // 3.3v DC-based power rail
    Helio_RailType_DC5V,                                    // 5v DC-based power rail
    Helio_RailType_DC12V,                                   // 12v DC-based power rail
    Helio_RailType_DC24V,                                   // 24v DC-based power rail
    Helio_RailType_DC48V,                                   // 48v DC-based power rail

    Helio_RailType_Count,                                   // Placeholder
    Helio_RailType_Undefined = -1,                          // Placeholder
    Helio_RailType_DefaultAC = Helio_RailType_AC110V        // Default AC rating for AC-based power rail (alias, feel free to change)
};

// Pin Mode
// Pin mode setting. Specifies what kind of pin and how it's used.
enum Helio_PinMode : signed char {
    Helio_PinMode_Digital_Input_Floating,                   // Digital input pin as floating/no-pull (pull-up/pull-down disabled, used during mux channel select, type alias for INPUT/GPIO_PuPd_NOPULL)
    Helio_PinMode_Digital_Input_PullUp,                     // Digital input pin with pull-up resistor (default pairing for active-low trigger, type alias for INPUT_PULLUP/GPIO_PuPd_UP)
    Helio_PinMode_Digital_Input_PullDown,                   // Digital input pin with pull-down resistor (or pull-up disabled if not avail, default pairing for active-high trigger, type alias for INPUT_PULLDOWN/GPIO_PuPd_DOWN)
    Helio_PinMode_Digital_Output_OpenDrain,                 // Digital output pin with open-drain NPN-based sink (default pairing for active-low trigger, type alias for OUTPUT/GPIO_OType_OD)
    Helio_PinMode_Digital_Output_PushPull,                  // Digital output pin with push-pull NPN+PNP-based sink+src (default pairing for active-high trigger, type alias for GPIO_OType_PP)
    Helio_PinMode_Analog_Input,                             // Analog input pin (type alias for INPUT)
    Helio_PinMode_Analog_Output,                            // Analog output pin (type alias for OUTPUT)

    Helio_PinMode_Count,                                    // Placeholder
    Helio_PinMode_Undefined = -1,                           // Placeholder
    Helio_PinMode_Digital_Input = Helio_PinMode_Digital_Input_Floating, // Default digital input (alias for Floating, type for INPUT)
    Helio_PinMode_Digital_Output = Helio_PinMode_Digital_Output_OpenDrain // Default digital output (alias for OpenDrain, type for OUTPUT)
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
    Helio_EnableMode_DescOrder,                             // Serial activation using highest-to-lowest/descending-order drive intensities
    Helio_EnableMode_AscOrder,                              // Serial activation using lowest-to-highest/ascending-order drive intensities

    Helio_EnableMode_Count,                                 // Placeholder
    Helio_EnableMode_Undefined = -1,                        // Placeholder
    Helio_EnableMode_Serial = Helio_EnableMode_InOrder      // Serial activation (alias for InOrder)
};

// Direction Mode
// Actuator intensity application mode. Specifies activation directionality and enablement.
enum Helio_DirectionMode : signed char {
    Helio_DirectionMode_Forward,                            // Standard/forward direction mode
    Helio_DirectionMode_Reverse,                            // Opposite/reverse direction mode
    Helio_DirectionMode_Stop,                               // Stationary/braking direction mode (intensity undef)

    Helio_DirectionMode_Count,                              // Placeholder
    Helio_DirectionMode_Undefined = -1                      // Placeholder
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
// Common driving states. Specifies parking ability and speed of travel.
enum Helio_DrivingState : signed char {
    Helio_DrivingState_OffTarget,                           // Far off target / use coarse/fast travel correction
    Helio_DrivingState_NearbyTarget,                        // Nearby target / use fine/slow travel correction
    Helio_DrivingState_AlignedTarget,                       // Aligned to target  / no movement, brake if able

    Helio_DrivingState_Count,                               // Placeholder
    Helio_DrivingState_Undefined = -1                       // Placeholder
};

// Panel State
// Common panel states. Specifies panel operational status.
enum Helio_PanelState : signed char {
    Helio_PanelState_AlignedToSun,                          // Aligned to sun
    Helio_PanelState_TravelingToSun,                        // Traveling to sun
    Helio_PanelState_TravelingToHome,                       // Traveling to home position
    Helio_PanelState_AlignedToHome,                         // Aligned to home position

    Helio_PanelState_Count,                                 // Placeholder
    Helio_PanelState_Undefined = -1                         // Placeholder
};

// Panel Axis
// Common panel axis aliases. Aliases axis names to axis indicies.
enum Helio_PanelAxis : signed char {
    Helio_PanelAxis_Azimuth = 0,                            // Azimuth axis (horizontal-coords, left/right control)
    Helio_PanelAxis_Elevation = 1,                          // Elevation axis (horizontal-coords, up/down control)
    Helio_PanelAxis_RightAscension = 0,                     // Right ascension axis (equatorial-coords, left/right control)
    Helio_PanelAxis_Declination = 1,                        // Declination axis (equatorial-coords, up/down control)

    Helio_PanelAxis_FacingHorizontal = 0,                   // Horizontal facing axis (azi or RA, left/right control)
    Helio_PanelAxis_FacingVertical = 1,                     // Vertical facing axis (ele or dec, up/down control)
    Helio_PanelAxis_AlignedHorizontal = 1,                  // Horizontally aligned axis (ele or dec, up/down control)
    Helio_PanelAxis_AlignedVertical = 0,                    // Vertically aligned axis (azi or RA, left/right control)
};

// Panel LDR
// Common LDR index aliases. Aliases LDR names to LDR indicies.
enum Helio_PanelLDR : signed char {
    Helio_PanelLDR_HorizontalMin = 0,                       // Horizontally mounted minimum LDR (left control)
    Helio_PanelLDR_HorizontalMax = 1,                       // Horizontally mounted maximum LDR (right control)
    Helio_PanelLDR_VerticalMin = 2,                         // Vertically mounted minimum LDR (down control)
    Helio_PanelLDR_VerticalMax = 3                          // Vertically mounted maximum LDR (up control)
};

// Units Category
// Unit of measurement category. Specifies the kind of unit.
enum Helio_UnitsCategory : signed char {
    Helio_UnitsCategory_Angle,                              // Angle based unit
    Helio_UnitsCategory_Distance,                           // Distance/position based unit
    Helio_UnitsCategory_Percentile,                         // Percentile based unit
    Helio_UnitsCategory_Power,                              // Power based unit
    Helio_UnitsCategory_Speed,                              // Speed/travel based unit
    Helio_UnitsCategory_Temperature,                        // Temperature based unit

    Helio_UnitsCategory_Count,                              // Placeholder
    Helio_UnitsCategory_Undefined = -1                      // Placeholder
};

// Units Type
// Unit of measurement type. Specifies the unit type associated with a measured value.
// Note: Rate units may only be in per minute, use PER_X_TO_PER_Y defines to convert.
enum Helio_UnitsType : signed char {
    Helio_UnitsType_Raw_1,                                  // Normalized raw value mode [0,1=aRef]
    Helio_UnitsType_Percentile_100,                         // Percentile mode [0,100]
    Helio_UnitsType_Angle_Degrees_360,                      // Degrees angle mode [0,%360)
    Helio_UnitsType_Angle_Radians_2pi,                      // Radians angle mode [0,%2pi)
    Helio_UnitsType_Angle_Minutes_24hr,                     // Minutes angle mode [0,%24hr)
    Helio_UnitsType_Distance_Feet,                          // Feet distance mode
    Helio_UnitsType_Distance_Meters,                        // Meters distance mode
    Helio_UnitsType_Power_Amperage,                         // Amperage current power mode
    Helio_UnitsType_Power_Wattage,                          // Wattage power mode
    Helio_UnitsType_Speed_FeetPerMin,                       // Feet per minute speed mode
    Helio_UnitsType_Speed_MetersPerMin,                     // Meters per minute speed mode
    Helio_UnitsType_Temperature_Celsius,                    // Celsius temperature mode
    Helio_UnitsType_Temperature_Fahrenheit,                 // Fahrenheit temperature mode
    Helio_UnitsType_Temperature_Kelvin,                     // Kelvin temperature mode

    Helio_UnitsType_Count,                                  // Placeholder
    Helio_UnitsType_Power_JoulesPerSecond = Helio_UnitsType_Power_Wattage, // Joules per second power mode alias
    Helio_UnitsType_Undefined = -1                          // Placeholder
};

#define PER_SEC_TO_PER_MIN(t)       ((t) * (SECS_PER_MIN))  // Per seconds to per minutes
#define PER_SEC_TO_PER_HR(t)        ((t) * (SECS_PER_HOUR)) // Per seconds to per hour
#define PER_MIN_TO_PER_SEC(t)       ((t) / (SECS_PER_MIN))  // Per minutes to per seconds
#define PER_MIN_TO_PER_HR(t)        ((t) * (SECS_PER_MIN))  // Per minutes to per hour
#define PER_HR_TO_PER_SEC(t)        ((t) / (SECS_PER_HOUR)) // Per hour to per seconds
#define PER_HR_TO_PER_MIN(t)        ((t) / (SECS_PER_MIN))  // Per hour to per minutes

// Common forward decls
class Helioduino;
class HelioScheduler;
class HelioLogger;
class HelioPublisher;
struct HelioIdentity;
struct HelioData;
struct HelioSubData;
struct HelioCalibrationData;
struct HelioObjectData;
struct HelioSystemData;
struct HelioPin;
struct HelioDigitalPin;
struct HelioAnalogPin;
struct HelioActivation;
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

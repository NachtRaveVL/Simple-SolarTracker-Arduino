// Simple-SolarTracking-Arduino Dual-Axis (DA) Tracking Example
//
// The DA Tracking Example sketch is our recommended standard implementation. It can
// be easily extended to include other functionality as desired. Follow the setup
// defines below, filling in for your own particular system setup.
//
// Under minimal UI mode, if any UI I/O is enabled, the UI will allow you to modify,
// but not create/destroy, objects created below. This restriction also applies to
// various other settings, which will be locked under the UI.
//
// To again modify these locked values, a new sketch will have to be re-built and
// re-uploaded. This process allows code-stripping to reduce build sizes to levels
// that may work with <512kB Flash on constrained devices. Otherwise, one may need to
// consider data export to SD Card or EEPROM - see the DataWriter example for details.

#ifdef USE_SW_SERIAL
#include "SoftwareSerial.h"
SoftwareSerial SWSerial(RX, TX);                        // Replace with Rx/Tx pins of your choice
#define Serial1 SWSerial
#endif

#include <Helioduino.h>

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
#define SETUP_CTRL_INPUT_PINS           {hpin_none}     // Control input pins array, else {-1} (should be same sized array as control input mode enum specifies)
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
#define SETUP_UI_TOUCHSCREEN_SPI        SPI             // SPI class for XPT2046 touchscreen, if using XTP2046
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

#if defined(HELIO_USE_WIFI)
WiFiClient netClient;
#elif defined(HELIO_USE_ETHERNET)
EthernetClient netClient;
#endif
#ifdef HELIO_USE_MQTT
MQTTClient mqttClient;
#endif

#if defined(HELIO_USE_GUI) && (NOT_SETUP_AS(SETUP_CONTROL_IN_MODE, Disabled) || NOT_SETUP_AS(SETUP_DISPLAY_OUT_MODE, Disabled)) && NOT_SETUP_AS(SETUP_SYS_UI_MODE, Disabled)
#if IS_SETUP_AS(SETUP_SYS_UI_MODE, Minimal)
#include "min/HelioduinoUI.h"
#elif IS_SETUP_AS(SETUP_SYS_UI_MODE, Full)
#include "full/HelioduinoUI.h"
#endif
#if IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ST7735) || IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ST7789) || IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ILI9341)
#if !SETUP_UI_USE_TCUNICODE_FONTS
#include "shared/tcMenu_Font_AdafruitGFXArial14.h"
#define SETUP_UI_USE_MENU_FONT          Arial14
#define SETUP_UI_USE_DETAIL_FONT        Arial14
#else
#include "shared/tcMenu_Font_tcUnicodeArial14.h"
#define SETUP_UI_USE_MENU_FONT          Arial14
#define SETUP_UI_USE_DETAIL_FONT        Arial14
#endif
#endif
#endif

#if SETUP_EXPANDER1_CHANNEL_BITS > 0 || SETUP_EXPANDER2_CHANNEL_BITS > 0
#include <IoAbstractionWire.h>
#endif

// Pre-init checks
#if (NOT_SETUP_AS(SETUP_SAVES_WIFISTORAGE_MODE, Disabled) || SETUP_DATA_WIFISTORAGE_ENABLE || SETUP_LOG_WIFISTORAGE_ENABLE) && !defined(HELIO_USE_WIFI_STORAGE)
#warning The HELIO_ENABLE_WIFI flag is expected to be defined as well as WiFiNINA_Generic.h included in order to run this sketch with WiFiStorage features enabled
#endif
#if (NOT_SETUP_AS(SETUP_SAVES_SD_CARD_MODE, Disabled) || SETUP_DATA_SD_ENABLE || SETUP_LOG_SD_ENABLE || SETUP_EXTDATA_SD_ENABLE) && SETUP_SD_CARD_SPI_CS == -1
#warning The SETUP_SD_CARD_SPI_CS define is expected to be set to a valid pin in order to run this sketch with SD card features enabled
#endif
#if (NOT_SETUP_AS(SETUP_SAVES_EEPROM_MODE, Disabled) || SETUP_EXTDATA_EEPROM_ENABLE) && IS_SETUP_AS(SETUP_EEPROM_DEVICE_TYPE, None)
#warning The SETUP_EEPROM_DEVICE_TYPE define is expected to be set to a valid size in order to run this sketch with EEPROM features enabled
#endif

pintype_t SETUP_CTRL_INPUT_PINS_[] = SETUP_CTRL_INPUT_PINS;
Helioduino helioController((pintype_t)SETUP_PIEZO_BUZZER_PIN,
                           JOIN(Helio_EEPROMType,SETUP_EEPROM_DEVICE_TYPE),
                           I2CDeviceSetup((uint8_t)SETUP_EEPROM_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
                           JOIN(Helio_RTCType,SETUP_RTC_DEVICE_TYPE),
                           I2CDeviceSetup((uint8_t)0b000, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
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
#if defined(HELIO_USE_GPS) && (IS_SETUP_AS(SETUP_GPS_TYPE, UART) || IS_SETUP_AS(SETUP_GPS_TYPE, Serial))
                           UARTDeviceSetup(&SETUP_GPS_SERIAL, HELIO_SYS_NMEAGPS_SERIALBAUD),
#elif defined(HELIO_USE_GPS) && IS_SETUP_AS(SETUP_GPS_TYPE, I2C)
                           I2CDeviceSetup(SETUP_GPS_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
#elif defined(HELIO_USE_GPS) && IS_SETUP_AS(SETUP_GPS_TYPE, SPI)
                           SPIDeviceSetup(SETUP_GPS_SPI_CS, &SETUP_GPS_SPI),
#else
                           DeviceSetup(),
#endif
                           SETUP_CTRL_INPUT_PINS_,
#if SETUP_DISP_SPI_CS >= 0
                           SPIDeviceSetup(SETUP_DISP_SPI_CS, &SETUP_DISP_SPI, SETUP_DISP_SPI_SPEED)
#elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, LCD16x2_EN) || IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, LCD16x2_RS) ||\
      IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, LCD20x4_EN) || IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, LCD20x4_RS) ||\
      SETUP_UI_IS_DFROBOTSHIELD
                           I2CDeviceSetup((uint8_t)SETUP_DISP_LCD_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED)
#else
                           I2CDeviceSetup((uint8_t)SETUP_DISP_OLED_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED)
#endif
                           );

#if SETUP_PANEL_BRAKE_PIN >= 0 || SETUP_PANEL_HEATER_PIN >= 0 || SETUP_PANEL_SPRAYER_PINCHNL >= 0
#define SETUP_USE_AC_RAIL
#endif
#if (SETUP_LINACT1_AXIS1_PINA >= 0 && SETUP_LINACT1_AXIS1_PINB >= 0) || (SETUP_LINACT2_AXIS1_PINA >= 0 && SETUP_LINACT2_AXIS1_PINB >= 0) || SETUP_POS_SERVO_AXIS0_PIN >= 0 || SETUP_POS_SERVO_AXIS1_PIN >= 0
#define SETUP_USE_DC_RAIL
#endif
#define SETUP_USE_ONEWIRE_BITRES    12

inline void setupPinChannels()
{
    #if SETUP_MUXER_CHANNEL_BITS > 0
        pintype_t _SETUP_MUXER_ADDRESS_PINS[] = SETUP_MUXER_ADDRESS_PINS;
        HelioDigitalPin chipEnable(SETUP_MUXER_ENABLE_PIN, OUTPUT, SETUP_MUXER_ENABLE_TYPE);
        #if SETUP_AC_USAGE_SENSOR_PINCHNL >= 0 && SETUP_AC_USAGE_SENSOR_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_AC_USAGE_SENSOR_PIN)) { helioController.setPinMuxer(SETUP_AC_USAGE_SENSOR_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_AC_USAGE_SENSOR_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_DC_USAGE_SENSOR_PINCHNL >= 0 && SETUP_DC_USAGE_SENSOR_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_DC_USAGE_SENSOR_PIN)) { helioController.setPinMuxer(SETUP_DC_USAGE_SENSOR_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_DC_USAGE_SENSOR_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_DC_GEN_SENSOR_PINCHNL >= 0 && SETUP_DC_GEN_SENSOR_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_DC_GEN_SENSOR_PIN)) { helioController.setPinMuxer(SETUP_DC_GEN_SENSOR_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_DC_GEN_SENSOR_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_ICE_INDICATOR_PINCHNL >= 0 && SETUP_ICE_INDICATOR_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_ICE_INDICATOR_PIN)) { helioController.setPinMuxer(SETUP_ICE_INDICATOR_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_ICE_INDICATOR_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_LINACT1_AXIS1_PINCHNLA >= 0 && SETUP_LINACT1_AXIS1_PINCHNLB >= 0 && SETUP_LINACT1_AXIS1_PINA >= 0 && SETUP_LINACT1_AXIS1_PINB >= 0
            if (!helioController.getPinMuxer(SETUP_LINACT1_AXIS1_PINA)) { helioController.setPinMuxer(SETUP_LINACT1_AXIS1_PINA, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT1_AXIS1_PINA, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
            if (!helioController.getPinMuxer(SETUP_LINACT1_AXIS1_PINB)) { helioController.setPinMuxer(SETUP_LINACT1_AXIS1_PINB, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT1_AXIS1_PINB, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_LINACT1_POS_SENSOR_PINCHNL >= 0 && SETUP_LINACT1_POS_SENSOR_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_LINACT1_POS_SENSOR_PIN)) { helioController.setPinMuxer(SETUP_LINACT1_POS_SENSOR_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT1_POS_SENSOR_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_LINACT1_MIN_ENDSTOP_PINCHNL >= 0 && SETUP_LINACT1_MIN_ENDSTOP_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_LINACT1_MIN_ENDSTOP_PIN)) { helioController.setPinMuxer(SETUP_LINACT1_MIN_ENDSTOP_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT1_MIN_ENDSTOP_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_LINACT1_MAX_ENDSTOP_PINCHNL >= 0 && SETUP_LINACT1_MAX_ENDSTOP_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_LINACT1_MAX_ENDSTOP_PIN)) { helioController.setPinMuxer(SETUP_LINACT1_MAX_ENDSTOP_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT1_MAX_ENDSTOP_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_LINACT2_AXIS1_PINCHNLA >= 0 && SETUP_LINACT2_AXIS1_PINCHNLB >= 0 && SETUP_LINACT2_AXIS1_PINA >= 0 && SETUP_LINACT2_AXIS1_PINB >= 0
            if (!helioController.getPinMuxer(SETUP_LINACT2_AXIS1_PINA)) { helioController.setPinMuxer(SETUP_LINACT2_AXIS1_PINA, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT2_AXIS1_PINA, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
            if (!helioController.getPinMuxer(SETUP_LINACT2_AXIS1_PINB)) { helioController.setPinMuxer(SETUP_LINACT2_AXIS1_PINB, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT2_AXIS1_PINB, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_LINACT2_POS_SENSOR_PINCHNL >= 0 && SETUP_LINACT2_POS_SENSOR_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_LINACT2_POS_SENSOR_PIN)) { helioController.setPinMuxer(SETUP_LINACT2_POS_SENSOR_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT2_POS_SENSOR_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_LINACT2_MIN_ENDSTOP_PINCHNL >= 0 && SETUP_LINACT2_MIN_ENDSTOP_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_LINACT2_MIN_ENDSTOP_PIN)) { helioController.setPinMuxer(SETUP_LINACT2_MIN_ENDSTOP_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT2_MIN_ENDSTOP_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_LINACT2_MAX_ENDSTOP_PINCHNL >= 0 && SETUP_LINACT2_MAX_ENDSTOP_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_LINACT2_MAX_ENDSTOP_PIN)) { helioController.setPinMuxer(SETUP_LINACT2_MAX_ENDSTOP_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_LINACT2_MAX_ENDSTOP_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_PANEL_BRAKE_PINCHNL >= 0 && SETUP_PANEL_BRAKE_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_PANEL_BRAKE_PIN)) { helioController.setPinMuxer(SETUP_PANEL_BRAKE_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_PANEL_BRAKE_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_PANEL_HEATER_PINCHNL >= 0 && SETUP_PANEL_HEATER_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_PANEL_HEATER_PIN)) { helioController.setPinMuxer(SETUP_PANEL_HEATER_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_PANEL_HEATER_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_PANEL_SPRAYER_PINCHNL >= 0 && SETUP_PANEL_SPRAYER_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_PANEL_SPRAYER_PIN)) { helioController.setPinMuxer(SETUP_PANEL_SPRAYER_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_PANEL_SPRAYER_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_PANEL_TILT_AXIS1_PINCHNL >= 0 && SETUP_PANEL_TILT_AXIS1_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_PANEL_TILT_AXIS1_PIN)) { helioController.setPinMuxer(SETUP_PANEL_TILT_AXIS1_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_PANEL_TILT_AXIS1_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_POS_SERVO_AXIS0_PINCHNL >= 0 && SETUP_POS_SERVO_AXIS0_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_POS_SERVO_AXIS0_PIN)) { helioController.setPinMuxer(SETUP_POS_SERVO_AXIS0_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_POS_SERVO_AXIS0_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_POS_SERVO_AXIS1_PINCHNL >= 0 && SETUP_POS_SERVO_AXIS1_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_POS_SERVO_AXIS1_PIN)) { helioController.setPinMuxer(SETUP_POS_SERVO_AXIS1_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_POS_SERVO_AXIS1_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
        #if SETUP_WIND_SPEED_SENSOR_PINCHNL >= 0 && SETUP_WIND_SPEED_SENSOR_PIN >= 0
            if (!helioController.getPinMuxer(SETUP_WIND_SPEED_SENSOR_PIN)) { helioController.setPinMuxer(SETUP_WIND_SPEED_SENSOR_PIN, SharedPtr<HelioPinMuxer>(new HelioPinMuxer(SETUP_WIND_SPEED_SENSOR_PIN, _SETUP_MUXER_ADDRESS_PINS, SETUP_MUXER_CHANNEL_BITS, chipEnable))); }
        #endif
    #elif SETUP_EXPANDER1_CHANNEL_BITS > 0 || SETUP_EXPANDER2_CHANNEL_BITS > 0
        auto expanders[] = {
            #if SETUP_EXPANDER1_CHANNEL_BITS > 0
                SharedPtr<HelioPinExpander>(new HelioPinExpander(SETUP_EXPANDER1_CHANNEL_BITS, SETUP_EXPANDER1_IOREF_ALLOC())),
            #else
                SharedPtr<HelioPinExpander>(nullptr),
            #endif
            #if SETUP_EXPANDER2_CHANNEL_BITS > 0
                SharedPtr<HelioPinExpander>(new HelioPinExpander(SETUP_EXPANDER2_CHANNEL_BITS, SETUP_EXPANDER2_IOREF_ALLOC()))
            #else
                SharedPtr<HelioPinExpander>(nullptr)
            #endif
        };
        // To setup control input pins as part of an expander, use pin #'s 100+, which are treated as virtual pins representing an expander index ((#-100) /16) and offset ((#-100) %16).
        if (isValidPin(SETUP_CTRL_INPUT_PINS_[0]) && SETUP_CTRL_INPUT_PINS_[0] >= hpin_virtual && !helioController.getPinExpander(expanderForPinNumber(SETUP_CTRL_INPUT_PINS_[0]))) {
            helioController.setPinExpander(expanderForPinNumber(SETUP_CTRL_INPUT_PINS_[0]), expanders[expanderForPinNumber(SETUP_CTRL_INPUT_PINS_[0])]);
        }
        #if SETUP_AC_USAGE_SENSOR_PINCHNL >= 0 || SETUP_AC_USAGE_SENSOR_PIN >= 100
            if (!helioController.getPinExpander(SETUP_AC_USAGE_SENSOR_PINCHNL/16)) { helioController.setPinExpander(SETUP_AC_USAGE_SENSOR_PINCHNL/16, expanders[SETUP_AC_USAGE_SENSOR_PINCHNL/16]); }
            #undef SETUP_AC_USAGE_SENSOR_PIN
            #define SETUP_AC_USAGE_SENSOR_PIN (100 + SETUP_AC_USAGE_SENSOR_PINCHNL)
        #endif
        #if SETUP_DC_USAGE_SENSOR_PINCHNL >= 0 || SETUP_DC_USAGE_SENSOR_PIN >= 100
            if (!helioController.getPinExpander(SETUP_DC_USAGE_SENSOR_PINCHNL/16)) { helioController.setPinExpander(SETUP_DC_USAGE_SENSOR_PINCHNL/16, expanders[SETUP_DC_USAGE_SENSOR_PINCHNL/16]); }
            #undef SETUP_DC_USAGE_SENSOR_PIN
            #define SETUP_DC_USAGE_SENSOR_PIN (100 + SETUP_DC_USAGE_SENSOR_PINCHNL)
        #endif
        #if SETUP_DC_GEN_SENSOR_PINCHNL >= 0 || SETUP_DC_GEN_SENSOR_PIN >= 100
            if (!helioController.getPinExpander(SETUP_DC_GEN_SENSOR_PINCHNL/16)) { helioController.setPinExpander(SETUP_DC_GEN_SENSOR_PINCHNL/16, expanders[SETUP_DC_GEN_SENSOR_PINCHNL/16]); }
            #undef SETUP_DC_GEN_SENSOR_PIN
            #define SETUP_DC_GEN_SENSOR_PIN (100 + SETUP_DC_GEN_SENSOR_PINCHNL)
        #endif
        #if SETUP_ICE_INDICATOR_PINCHNL >= 0 || SETUP_ICE_INDICATOR_PIN >= 100
            if (!helioController.getPinExpander(SETUP_ICE_INDICATOR_PINCHNL/16)) { helioController.setPinExpander(SETUP_ICE_INDICATOR_PINCHNL/16, expanders[SETUP_ICE_INDICATOR_PINCHNL/16]); }
            #undef SETUP_ICE_INDICATOR_PIN
            #define SETUP_ICE_INDICATOR_PIN (100 + SETUP_ICE_INDICATOR_PINCHNL)
        #endif
        #if (SETUP_LINACT1_AXIS1_PINCHNLA >= 0 && SETUP_LINACT1_AXIS1_PINCHNLB >= 0) || (SETUP_LINACT1_AXIS1_PINA >= 100 && SETUP_LINACT1_AXIS1_PINB >= 100)
            if (!helioController.getPinExpander(SETUP_LINACT1_AXIS1_PINCHNLA/16)) { helioController.setPinExpander(SETUP_LINACT1_AXIS1_PINCHNLA/16, expanders[SETUP_LINACT1_AXIS1_PINCHNLA/16]); }
            if (!helioController.getPinExpander(SETUP_LINACT1_AXIS1_PINCHNLB/16)) { helioController.setPinExpander(SETUP_LINACT1_AXIS1_PINCHNLB/16, expanders[SETUP_LINACT1_AXIS1_PINCHNLB/16]); }
            #undef SETUP_LINACT1_AXIS1_PINA
            #define SETUP_LINACT1_AXIS1_PINA (100 + SETUP_LINACT1_AXIS1_PINCHNLA)
            #undef SETUP_LINACT1_AXIS1_PINB
            #define SETUP_LINACT1_AXIS1_PINB (100 + SETUP_LINACT1_AXIS1_PINCHNLB)
        #endif
        #if SETUP_LINACT1_POS_SENSOR_PINCHNL >= 0 || SETUP_LINACT1_POS_SENSOR_PIN >= 100
            if (!helioController.getPinExpander(SETUP_LINACT1_POS_SENSOR_PINCHNL/16)) { helioController.setPinExpander(SETUP_LINACT1_POS_SENSOR_PINCHNL/16, expanders[SETUP_LINACT1_POS_SENSOR_PINCHNL/16]); }
            #undef SETUP_LINACT1_POS_SENSOR_PIN
            #define SETUP_LINACT1_POS_SENSOR_PIN (100 + SETUP_LINACT1_POS_SENSOR_PINCHNL)
        #endif
        #if SETUP_LINACT1_MIN_ENDSTOP_PINCHNL >= 0 || SETUP_LINACT1_MIN_ENDSTOP_PIN >= 100
            if (!helioController.getPinExpander(SETUP_LINACT1_MIN_ENDSTOP_PINCHNL/16)) { helioController.setPinExpander(SETUP_LINACT1_MIN_ENDSTOP_PINCHNL/16, expanders[SETUP_LINACT1_MIN_ENDSTOP_PINCHNL/16]); }
            #undef SETUP_LINACT1_MIN_ENDSTOP_PIN
            #define SETUP_LINACT1_MIN_ENDSTOP_PIN (100 + SETUP_LINACT1_MIN_ENDSTOP_PINCHNL)
        #endif
        #if SETUP_LINACT1_MAX_ENDSTOP_PINCHNL >= 0 || SETUP_LINACT1_MAX_ENDSTOP_PIN >= 100
            if (!helioController.getPinExpander(SETUP_LINACT1_MAX_ENDSTOP_PINCHNL/16)) { helioController.setPinExpander(SETUP_LINACT1_MAX_ENDSTOP_PINCHNL/16, expanders[SETUP_LINACT1_MAX_ENDSTOP_PINCHNL/16]); }
            #undef SETUP_LINACT1_MAX_ENDSTOP_PIN
            #define SETUP_LINACT1_MAX_ENDSTOP_PIN (100 + SETUP_LINACT1_MAX_ENDSTOP_PINCHNL)
        #endif
        #if (SETUP_LINACT2_AXIS1_PINCHNLA >= 0 && SETUP_LINACT2_AXIS1_PINCHNLB >= 0) || (SETUP_LINACT2_AXIS1_PINA >= 100 || SETUP_LINACT2_AXIS1_PINB >= 100)
            if (!helioController.getPinExpander(SETUP_LINACT2_AXIS1_PINCHNLA/16)) { helioController.setPinExpander(SETUP_LINACT2_AXIS1_PINCHNLA/16, expanders[SETUP_LINACT2_AXIS1_PINCHNLA/16]); }
            if (!helioController.getPinExpander(SETUP_LINACT2_AXIS1_PINCHNLB/16)) { helioController.setPinExpander(SETUP_LINACT2_AXIS1_PINCHNLB/16, expanders[SETUP_LINACT2_AXIS1_PINCHNLB/16]); }
            #undef SETUP_LINACT2_AXIS1_PINA
            #define SETUP_LINACT2_AXIS1_PINA (100 + SETUP_LINACT2_AXIS1_PINCHNLA)
            #undef SETUP_LINACT2_AXIS1_PINB
            #define SETUP_LINACT2_AXIS1_PINB (100 + SETUP_LINACT2_AXIS1_PINCHNLB)
        #endif
        #if SETUP_LINACT2_POS_SENSOR_PINCHNL >= 0 || SETUP_LINACT2_POS_SENSOR_PIN >= 100
            if (!helioController.getPinExpander(SETUP_LINACT2_POS_SENSOR_PINCHNL/16)) { helioController.setPinExpander(SETUP_LINACT2_POS_SENSOR_PINCHNL/16, expanders[SETUP_LINACT2_POS_SENSOR_PINCHNL/16]); }
            #undef SETUP_LINACT2_POS_SENSOR_PIN
            #define SETUP_LINACT2_POS_SENSOR_PIN (100 + SETUP_LINACT2_POS_SENSOR_PINCHNL)
        #endif
        #if SETUP_LINACT2_MIN_ENDSTOP_PINCHNL >= 0 || SETUP_LINACT2_MIN_ENDSTOP_PIN >= 100
            if (!helioController.getPinExpander(SETUP_LINACT2_MIN_ENDSTOP_PINCHNL/16)) { helioController.setPinExpander(SETUP_LINACT2_MIN_ENDSTOP_PINCHNL/16, expanders[SETUP_LINACT2_MIN_ENDSTOP_PINCHNL/16]); }
            #undef SETUP_LINACT2_MIN_ENDSTOP_PIN
            #define SETUP_LINACT2_MIN_ENDSTOP_PIN (100 + SETUP_LINACT2_MIN_ENDSTOP_PINCHNL)
        #endif
        #if SETUP_LINACT2_MAX_ENDSTOP_PINCHNL >= 0 || SETUP_LINACT2_MAX_ENDSTOP_PIN >= 100
            if (!helioController.getPinExpander(SETUP_LINACT2_MAX_ENDSTOP_PINCHNL/16)) { helioController.setPinExpander(SETUP_LINACT2_MAX_ENDSTOP_PINCHNL/16, expanders[SETUP_LINACT2_MAX_ENDSTOP_PINCHNL/16]); }
            #undef SETUP_LINACT2_MAX_ENDSTOP_PIN
            #define SETUP_LINACT2_MAX_ENDSTOP_PIN (100 + SETUP_LINACT2_MAX_ENDSTOP_PINCHNL)
        #endif
        #if SETUP_PANEL_BRAKE_PINCHNL >= 0 || SETUP_PANEL_BRAKE_PIN >= 100
            if (!helioController.getPinExpander(SETUP_PANEL_BRAKE_PINCHNL/16)) { helioController.setPinExpander(SETUP_PANEL_BRAKE_PINCHNL/16, expanders[SETUP_PANEL_BRAKE_PINCHNL/16]); }
            #undef SETUP_PANEL_BRAKE_PIN
            #define SETUP_PANEL_BRAKE_PIN (100 + SETUP_PANEL_BRAKE_PINCHNL)
        #endif
        #if SETUP_PANEL_HEATER_PINCHNL >= 0 || SETUP_PANEL_HEATER_PIN >= 100
            if (!helioController.getPinExpander(SETUP_PANEL_HEATER_PINCHNL/16)) { helioController.setPinExpander(SETUP_PANEL_HEATER_PINCHNL/16, expanders[SETUP_PANEL_HEATER_PINCHNL/16]); }
            #undef SETUP_PANEL_HEATER_PIN
            #define SETUP_PANEL_HEATER_PIN (100 + SETUP_PANEL_HEATER_PINCHNL)
        #endif
        #if SETUP_PANEL_SPRAYER_PINCHNL >= 0 || SETUP_PANEL_SPRAYER_PIN >= 100
            if (!helioController.getPinExpander(SETUP_PANEL_SPRAYER_PINCHNL/16)) { helioController.setPinExpander(SETUP_PANEL_SPRAYER_PINCHNL/16, expanders[SETUP_PANEL_SPRAYER_PINCHNL/16]); }
            #undef SETUP_PANEL_SPRAYER_PIN
            #define SETUP_PANEL_SPRAYER_PIN (100 + SETUP_PANEL_SPRAYER_PINCHNL)
        #endif
        #if SETUP_PANEL_TILT_AXIS1_PINCHNL >= 0 || SETUP_PANEL_TILT_AXIS1_PIN >= 100
            if (!helioController.getPinExpander(SETUP_PANEL_TILT_AXIS1_PINCHNL/16)) { helioController.setPinExpander(SETUP_PANEL_TILT_AXIS1_PINCHNL/16, expanders[SETUP_PANEL_TILT_AXIS1_PINCHNL/16]); }
            #undef SETUP_PANEL_TILT_AXIS1_PIN
            #define SETUP_PANEL_TILT_AXIS1_PIN (100 + SETUP_PANEL_TILT_AXIS1_PINCHNL)
        #endif
        #if SETUP_POS_SERVO_AXIS0_PINCHNL >= 0 || SETUP_POS_SERVO_AXIS0_PIN >= 100
            if (!helioController.getPinExpander(SETUP_POS_SERVO_AXIS0_PINCHNL/16)) { helioController.setPinExpander(SETUP_POS_SERVO_AXIS0_PINCHNL/16, expanders[SETUP_POS_SERVO_AXIS0_PINCHNL/16]); }
            #undef SETUP_POS_SERVO_AXIS0_PIN
            #define SETUP_POS_SERVO_AXIS0_PIN (100 + SETUP_POS_SERVO_AXIS0_PINCHNL)
        #endif
        #if SETUP_POS_SERVO_AXIS1_PINCHNL >= 0 || SETUP_POS_SERVO_AXIS1_PIN >= 100
            if (!helioController.getPinExpander(SETUP_POS_SERVO_AXIS1_PINCHNL/16)) { helioController.setPinExpander(SETUP_POS_SERVO_AXIS1_PINCHNL/16, expanders[SETUP_POS_SERVO_AXIS1_PINCHNL/16]); }
            #undef SETUP_POS_SERVO_AXIS1_PIN
            #define SETUP_POS_SERVO_AXIS1_PIN (100 + SETUP_POS_SERVO_AXIS1_PINCHNL)
        #endif
        #if SETUP_WIND_SPEED_SENSOR_PINCHNL >= 0 || SETUP_WIND_SPEED_SENSOR_PIN >= 100
            if (!helioController.getPinExpander(SETUP_WIND_SPEED_SENSOR_PINCHNL/16)) { helioController.setPinExpander(SETUP_WIND_SPEED_SENSOR_PINCHNL/16, expanders[SETUP_WIND_SPEED_SENSOR_PINCHNL/16]); }
            #undef SETUP_WIND_SPEED_SENSOR_PIN
            #define SETUP_WIND_SPEED_SENSOR_PIN (100 + SETUP_WIND_SPEED_SENSOR_PINCHNL)
        #endif
    #endif
}

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
    #ifndef HELIO_USE_GPS
        helioController.setSystemLocation(SETUP_SYS_STATIC_LAT, SETUP_SYS_STATIC_LONG, SETUP_SYS_STATIC_ALT);
    #endif
    #if defined(HELIO_USE_WIFI_STORAGE) && IS_SETUP_AS(SETUP_SAVES_WIFISTORAGE_MODE, Primary)
        helioController.setAutosaveEnabled(Helio_Autosave_EnabledToWiFiStorageJson
    #elif SETUP_SD_CARD_SPI_CS >= 0 && IS_SETUP_AS(SETUP_SAVES_SD_CARD_MODE, Primary)
        helioController.setAutosaveEnabled(Helio_Autosave_EnabledToSDCardJson
    #elif NOT_SETUP_AS(SETUP_EEPROM_DEVICE_TYPE, None) && IS_SETUP_AS(SETUP_SAVES_EEPROM_MODE, Primary)
        helioController.setAutosaveEnabled(Helio_Autosave_EnabledToEEPROMRaw
    #else
        helioController.setAutosaveEnabled(Helio_Autosave_Disabled
    #endif
    #if defined(HELIO_USE_WIFI_STORAGE) && IS_SETUP_AS(SETUP_SAVES_WIFISTORAGE_MODE, Fallback)
        , Helio_Autosave_EnabledToWiFiStorageJson);
    #elif SETUP_SD_CARD_SPI_CS >= 0 && IS_SETUP_AS(SETUP_SAVES_SD_CARD_MODE, Fallback)
        , Helio_Autosave_EnabledToSDCardJson);
    #elif NOT_SETUP_AS(SETUP_EEPROM_DEVICE_TYPE, None) && IS_SETUP_AS(SETUP_SAVES_EEPROM_MODE, Fallback)
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
            #if IS_SETUP_AS(SETUP_MQTT_BROKER_CONNECT_BY, Hostname)
                mqttClient.begin(String(F(SETUP_MQTT_BROKER_HOSTNAME)).c_str(), SETUP_MQTT_BROKER_PORT, netClient);
            #elif IS_SETUP_AS(SETUP_MQTT_BROKER_CONNECT_BY, IPAddress)
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
            {   auto powerMeter = helioController.addPowerLevelMeter(SETUP_AC_USAGE_SENSOR_PIN, ADC_RESOLUTION, SETUP_AC_USAGE_SENSOR_PINCHNL);
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
            {   auto powerMeter = helioController.addPowerLevelMeter(SETUP_DC_USAGE_SENSOR_PIN, ADC_RESOLUTION, SETUP_DC_USAGE_SENSOR_PINCHNL);
                dcRelayPower->setPowerSensor(powerMeter);
            }
            #endif
        #else
            auto dcRelayPower = helioController.addSimplePowerRail(JOIN(Helio_RailType,SETUP_DC_POWER_RAIL_TYPE));
        #endif
    #endif
    auto trackingPanel = helioController.addSolarTrackingPanel(JOIN(Helio_PanelType,SETUP_PANEL_TYPE));
    {   float _SETUP_PANEL_HOME[] = SETUP_PANEL_HOME;
        float _SETUP_PANEL_OFFSET[] = SETUP_PANEL_OFFSET;
        trackingPanel->setHomePosition(_SETUP_PANEL_HOME);
        trackingPanel->setAxisOffset(_SETUP_PANEL_OFFSET);
    }

    // Panel Sensors
    #if SETUP_ICE_INDICATOR_PIN >= 0
    {   auto iceIndicator = helioController.addIceIndicator(SETUP_ICE_INDICATOR_PIN, SETUP_ICE_INDICATOR_TYPE, SETUP_ICE_INDICATOR_PINCHNL);
        iceIndicator->setParentPanel(trackingPanel);
        trackingPanel->setHeatingTrigger(new HelioMeasurementValueTrigger(iceIndicator, 0.5, ACT_ABOVE));
    }
    #endif
    #if SETUP_DC_GEN_SENSOR_PIN >= 0
    {   auto dcGeneration = helioController.addPowerProductionMeter(SETUP_DC_GEN_SENSOR_PIN, true, ADC_RESOLUTION, SETUP_DC_GEN_SENSOR_PINCHNL);
        dcGeneration->setParentPanel(trackingPanel);
        trackingPanel->setPowerProductionSensor(dcGeneration);
    }
    #endif
    #if SETUP_WIND_SPEED_SENSOR_PIN >= 0
    {   auto windSpeedSensor = helioController.addAnalogWindSpeedSensor(SETUP_WIND_SPEED_SENSOR_PIN, ADC_RESOLUTION);
        windSpeedSensor->setParentPanel(trackingPanel);
        HelioCalibrationData windSpeedCalib(windSpeedSensor->getId(), Helio_UnitsType_Speed_MetersPerMin);
        windSpeedCalib.setFromTwoPoints(SETUP_WIND_SPEED_SENSOR_SCALE);
        windSpeedSensor->setUserCalibrationData(&windSpeedCalib);
        trackingPanel->setWindSpeedSensor(windSpeedSensor);
        trackingPanel->setStormingTrigger(new HelioMeasurementValueTrigger(windSpeedSensor, SETUP_PANEL_STORMING_SPEED, ACT_ABOVE, 0, SETUP_PANEL_STORMING_SPEED * 0.1f, 30000));
    }
    #endif
    #if SETUP_DHT_AIR_TEMP_HUMID_PIN >= 0
    {   auto dhtTemperatureSensor = helioController.addDHTTempHumiditySensor(SETUP_DHT_AIR_TEMP_HUMID_PIN, JOIN(Helio_DHTType,SETUP_DHT_SENSOR_TYPE));
        trackingPanel->setTemperatureSensor(dhtTemperatureSensor);
        #if !(SETUP_ICE_INDICATOR_PIN >= 0)
            trackingPanel->setHeatingTrigger(new HelioMeasurementValueTrigger(dhtTemperatureSensor, SETUP_PANEL_HEATER_TEMP, ACT_BELOW));
            trackingPanel->getHeatingTrigger()->setMeasurementUnits(Helio_UnitsType_Temperature_Celsius);
        #endif
    }
    #endif

    // Linear Actuators
    #if SETUP_LINACT1_AXIS1_PINA >= 0 && SETUP_LINACT1_AXIS1_PINB >= 0
        auto linearAct1 = helioController.addLinearActuatorRelay(SETUP_LINACT1_AXIS1_PINA, SETUP_LINACT1_AXIS1_PINB, SETUP_LINACT_AXIS1_STROKE, 0, SETUP_LINACT1_AXIS1_PINCHNLA, SETUP_LINACT1_AXIS1_PINCHNLB);
        linearAct1->setParentPanel(trackingPanel, 1);
        linearAct1->setParentRail(dcRelayPower);
        linearAct1->setContinuousSpeed(HelioSingleMeasurement(SETUP_LINACT_TRAVEL_SPEED, Helio_UnitsType_Speed_MetersPerMin));
    #endif
    #if SETUP_LINACT1_POS_SENSOR_PIN >= 0
    {   auto positionSensor = helioController.addAnalogPositionSensor(SETUP_LINACT1_POS_SENSOR_PIN, ADC_RESOLUTION, SETUP_LINACT1_POS_SENSOR_PINCHNL);
        positionSensor->setParentPanel(trackingPanel, 1);
        #if SETUP_LINACT1_AXIS1_PINA >= 0 && SETUP_LINACT1_AXIS1_PINB >= 0
            linearAct1->setPositionSensor(positionSensor);
        #endif
    }
    #endif
    #if SETUP_LINACT1_MIN_ENDSTOP_PIN >= 0
    {   auto minimumIndicator = helioController.addEndstopIndicator(SETUP_LINACT1_MIN_ENDSTOP_PIN, SETUP_MINMAX_INDICATOR_TYPE, SETUP_LINACT1_MIN_ENDSTOP_PINCHNL);
        minimumIndicator->setParentPanel(trackingPanel, 1);
        #if SETUP_LINACT1_AXIS1_PINA >= 0 && SETUP_LINACT1_AXIS1_PINB >= 0
            linearAct1->setMinimumTrigger(new HelioMeasurementValueTrigger(minimumIndicator, 0.5, ACT_ABOVE));
        #endif
    }
    #endif
    #if SETUP_LINACT1_MAX_ENDSTOP_PIN >= 0
    {   auto maximumIndicator = helioController.addEndstopIndicator(SETUP_LINACT1_MAX_ENDSTOP_PIN, SETUP_MINMAX_INDICATOR_TYPE, SETUP_LINACT1_MAX_ENDSTOP_PINCHNL);
        maximumIndicator->setParentPanel(trackingPanel, 1);
        #if SETUP_LINACT1_AXIS1_PINA >= 0 && SETUP_LINACT1_AXIS1_PINB >= 0
            linearAct1->setMaximumTrigger(new HelioMeasurementValueTrigger(maximumIndicator, 0.5, ACT_ABOVE));
        #endif
    }
    #endif
    #if SETUP_LINACT2_AXIS1_PINA >= 0 && SETUP_LINACT2_AXIS1_PINB >= 0
        auto linearAct2 = helioController.addLinearActuatorRelay(SETUP_LINACT2_AXIS1_PINA, SETUP_LINACT2_AXIS1_PINB, SETUP_LINACT_AXIS1_STROKE, 0, SETUP_LINACT2_AXIS1_PINCHNLA, SETUP_LINACT2_AXIS1_PINCHNLB);
        linearAct2->setParentPanel(trackingPanel, 1);
        linearAct2->setParentRail(dcRelayPower);
        linearAct2->setContinuousSpeed(HelioSingleMeasurement(SETUP_LINACT_TRAVEL_SPEED, Helio_UnitsType_Speed_MetersPerMin));
    #endif
    #if SETUP_LINACT2_POS_SENSOR_PIN >= 0
    {   auto positionSensor = helioController.addAnalogPositionSensor(SETUP_LINACT2_POS_SENSOR_PIN, ADC_RESOLUTION, SETUP_LINACT2_POS_SENSOR_PINCHNL);
        positionSensor->setParentPanel(trackingPanel, 1);
        #if SETUP_LINACT2_AXIS1_PINA >= 0 && SETUP_LINACT2_AXIS1_PINB >= 0
            linearAct2->setPositionSensor(positionSensor);
        #endif
    }
    #endif
    #if SETUP_LINACT2_MIN_ENDSTOP_PIN >= 0
    {   auto minimumIndicator = helioController.addEndstopIndicator(SETUP_LINACT2_MIN_ENDSTOP_PIN, SETUP_MINMAX_INDICATOR_TYPE, SETUP_LINACT2_MIN_ENDSTOP_PINCHNL);
        minimumIndicator->setParentPanel(trackingPanel, 1);
        #if SETUP_LINACT2_AXIS1_PINA >= 0 && SETUP_LINACT2_AXIS1_PINB >= 0
            linearAct2->setMinimumTrigger(new HelioMeasurementValueTrigger(minimumIndicator, 0.5, ACT_ABOVE));
        #endif
    }
    #endif
    #if SETUP_LINACT2_MAX_ENDSTOP_PIN >= 0
    {   auto maximumIndicator = helioController.addEndstopIndicator(SETUP_LINACT2_MAX_ENDSTOP_PIN, SETUP_MINMAX_INDICATOR_TYPE, SETUP_LINACT2_MAX_ENDSTOP_PINCHNL);
        maximumIndicator->setParentPanel(trackingPanel, 1);
        #if SETUP_LINACT2_AXIS1_PINA >= 0 && SETUP_LINACT2_AXIS1_PINB >= 0
            linearAct2->setMaximumTrigger(new HelioMeasurementValueTrigger(maximumIndicator, 0.5, ACT_ABOVE));
        #endif
    }
    #endif

    // Panel Actuators
    #if SETUP_PANEL_BRAKE_PIN >= 0
    {
        auto panelBrake = helioController.addPanelBrakeRelay(SETUP_PANEL_BRAKE_PIN, SETUP_PANEL_BRAKE_PINCHNL);
        panelBrake->setParentPanel(trackingPanel, 1);
        panelBrake->setParentRail(acRelayPower);
    }
    #endif
    #if SETUP_PANEL_HEATER_PIN >= 0
    {
        auto panelHeater = helioController.addPanelHeaterRelay(SETUP_PANEL_HEATER_PIN, SETUP_PANEL_HEATER_PINCHNL);
        panelHeater->setParentPanel(trackingPanel);
        panelHeater->setParentRail(acRelayPower);
    }
    #endif
    #if SETUP_PANEL_SPRAYER_PIN >= 0
    {
        auto panelSprayer = helioController.addPanelSprayerRelay(SETUP_PANEL_SPRAYER_PIN, SETUP_PANEL_SPRAYER_PINCHNL);
        panelSprayer->setParentPanel(trackingPanel);
        panelSprayer->setParentRail(acRelayPower);
    }
    #endif
    #if SETUP_PANEL_TILT_AXIS1_PIN >= 0
    {
        auto panelTiltSensor = helioController.addAnalogTiltAngleSensor(SETUP_PANEL_TILT_AXIS1_PIN, ADC_RESOLUTION, SETUP_PANEL_TILT_AXIS1_PINCHNL);
        panelTiltSensor->setParentPanel(trackingPanel, 1);
        HelioCalibrationData panelTiltCalib(panelTiltSensor->getId(), Helio_UnitsType_Speed_MetersPerMin);
        panelTiltCalib.setFromTwoPoints(SETUP_PANEL_TILT_AXIS1_SCALE);
        panelTiltSensor->setUserCalibrationData(&panelTiltCalib);
        trackingPanel->setAxisAngleSensor(panelTiltSensor, 1);
    }
    #endif
    #if SETUP_POS_SERVO_AXIS0_PIN >= 0
    {
        auto panelAxis0Servo = helioController.addPositionalServo(SETUP_POS_SERVO_AXIS0_PIN, SETUP_POS_SERVO_MINMAX_ANGLE, DAC_RESOLUTION, SETUP_POS_SERVO_AXIS0_PINCHNL);
        panelAxis0Servo->setParentPanel(trackingPanel, 0);
        panelAxis0Servo->setParentRail(dcRelayPower);
    }
    #endif
    #if SETUP_POS_SERVO_AXIS1_PIN >= 0
    {
        auto panelAxis1Servo = helioController.addPositionalServo(SETUP_POS_SERVO_AXIS1_PIN, SETUP_POS_SERVO_MINMAX_ANGLE, DAC_RESOLUTION, SETUP_POS_SERVO_AXIS1_PINCHNL);
        panelAxis1Servo->setParentPanel(trackingPanel, 1);
        panelAxis1Servo->setParentRail(dcRelayPower);
    }
    #endif
}

inline void setupUI()
{
    #if defined(HELIO_USE_GUI) && (NOT_SETUP_AS(SETUP_CONTROL_IN_MODE, Disabled) || NOT_SETUP_AS(SETUP_DISPLAY_OUT_MODE, Disabled)) && NOT_SETUP_AS(SETUP_SYS_UI_MODE, Disabled)
        UIControlSetup uiCtrlSetup;
        UIDisplaySetup uiDispSetup;
        #if SETUP_UI_IS_DFROBOTSHIELD
            uiCtrlSetup = UIControlSetup::usingDFRobotShield();
            uiDispSetup = UIDisplaySetup::usingDFRobotShield();
        #else
            switch (helioController.getControlInputMode()) {
                case Helio_ControlInputMode_RotaryEncoderOk:
                case Helio_ControlInputMode_RotaryEncoderOkLR:
                    uiCtrlSetup = UIControlSetup(RotaryControlSetup(JOIN(Helio_EncoderSpeed,SETUP_UI_ENC_ROTARY_SPEED)));
                    break;
                case Helio_ControlInputMode_UpDownButtonsOk:
                case Helio_ControlInputMode_UpDownButtonsOkLR:
                    uiCtrlSetup = UIControlSetup(ButtonsControlSetup(SETUP_UI_KEY_REPEAT_SPEED));
                    break;
                case Helio_ControlInputMode_UpDownESP32TouchOk:
                case Helio_ControlInputMode_UpDownESP32TouchOkLR:
                    uiCtrlSetup = UIControlSetup(ESP32TouchControlSetup(SETUP_UI_KEY_REPEAT_SPEED, SETUP_UI_ESP32TOUCH_SWITCH, JOIN(Helio_ESP32Touch_HighRef,SETUP_UI_ESP32TOUCH_HVOLTS), JOIN(Helio_ESP32Touch_LowRef,SETUP_UI_ESP32TOUCH_LVOLTS), JOIN(Helio_ESP32Touch_HighRefAtten,SETUP_UI_ESP32TOUCH_HVATTEN)));
                    break;
                case Helio_ControlInputMode_AnalogJoystickOk:
                    uiCtrlSetup = UIControlSetup(JoystickControlSetup(SETUP_UI_KEY_REPEAT_DELAY, SETUP_UI_JS_ACCELERATION));
                    break;
                case Helio_ControlInputMode_Matrix2x2UpDownButtonsOkL:
                case Helio_ControlInputMode_Matrix3x4Keyboard_OptRotEncOk:
                case Helio_ControlInputMode_Matrix3x4Keyboard_OptRotEncOkLR:
                case Helio_ControlInputMode_Matrix4x4Keyboard_OptRotEncOk:
                case Helio_ControlInputMode_Matrix4x4Keyboard_OptRotEncOkLR:
                    uiCtrlSetup = UIControlSetup(MatrixControlSetup(SETUP_UI_KEY_REPEAT_DELAY, SETUP_UI_KEY_REPEAT_INTERVAL, JOIN(Helio_EncoderSpeed,SETUP_UI_ENC_ROTARY_SPEED)));
                    break;
                case Helio_ControlInputMode_ResistiveTouch:
                case Helio_ControlInputMode_TouchScreen:
                case Helio_ControlInputMode_TFTTouch:
                    #ifndef HELIO_UI_ENABLE_XPT2046TS
                        uiCtrlSetup = UIControlSetup(TouchscreenSetup(JOIN(Helio_TouchscreenOrientation,SETUP_UI_TOUCHSCREEN_ORIENT)));
                    #else
                        uiCtrlSetup = UIControlSetup(TouchscreenSetup(JOIN(Helio_TouchscreenOrientation,SETUP_UI_TOUCHSCREEN_ORIENT), &SETUP_UI_TOUCHSCREEN_SPI));
                    #endif
                    break;
                default: break;
            }
            switch (helioController.getDisplayOutputMode()) {
                case Helio_DisplayOutputMode_LCD16x2_EN:
                case Helio_DisplayOutputMode_LCD16x2_RS:
                case Helio_DisplayOutputMode_LCD20x4_EN:
                case Helio_DisplayOutputMode_LCD20x4_RS:
                    uiDispSetup = UIDisplaySetup(LCDDisplaySetup(JOIN(Helio_BacklightMode,SETUP_UI_GFX_BACKLIGHT_MODE)));
                    break;
                case Helio_DisplayOutputMode_SSD1305:
                case Helio_DisplayOutputMode_SSD1305_x32Ada:
                case Helio_DisplayOutputMode_SSD1305_x64Ada:
                case Helio_DisplayOutputMode_SSD1306:
                case Helio_DisplayOutputMode_SH1106:
                case Helio_DisplayOutputMode_CustomOLED:
                case Helio_DisplayOutputMode_SSD1607:
                case Helio_DisplayOutputMode_IL3820:
                case Helio_DisplayOutputMode_IL3820_V2:
                case Helio_DisplayOutputMode_ST7735:
                case Helio_DisplayOutputMode_ST7789:
                case Helio_DisplayOutputMode_ILI9341:
                    uiDispSetup = UIDisplaySetup(PixelDisplaySetup(JOIN(Helio_DisplayRotation,SETUP_UI_GFX_ROTATION), SETUP_UI_GFX_DC_PIN, SETUP_UI_GFX_RESET_PIN, SETUP_UI_GFX_BACKLIGHT_PIN, JOIN(Helio_BacklightMode,SETUP_UI_GFX_BACKLIGHT_MODE), DAC_RESOLUTION,
#ifdef ESP32
                                                                   SETUP_UI_GFX_BACKLIGHT_ESP_CHN,
#endif
#ifdef ESP_PLATFORM
                                                                   SETUP_UI_GFX_BACKLIGHT_ESP_FRQ,
#endif
#if IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ST7735)
                                                                   JOIN(Helio_ST7735Tag,SETUP_UI_GFX_ST7735_TAG)
#elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ST7789)
                                                                   JOIN(Helio_ST7789Res,SETUP_UI_GFX_ST7789_RES)
#else
                                                                   Helio_ST77XXKind_Undefined
#endif
                    ));
                    break;

                case Helio_DisplayOutputMode_TFT:
                    uiDispSetup = UIDisplaySetup(TFTDisplaySetup(JOIN(Helio_DisplayRotation,SETUP_UI_GFX_ROTATION), SETUP_UI_GFX_BACKLIGHT_PIN, JOIN(Helio_BacklightMode,SETUP_UI_GFX_BACKLIGHT_MODE), DAC_RESOLUTION,
#ifdef ESP32
                                                                 SETUP_UI_GFX_BACKLIGHT_ESP_CHN,
#endif
#ifdef ESP_PLATFORM
                                                                 SETUP_UI_GFX_BACKLIGHT_ESP_FRQ,
#endif
#if IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ST7735)
                                                                 JOIN(Helio_ST7735Tag,SETUP_UI_GFX_ST7735_TAG)
#else
                                                                 Helio_ST77XXKind_Undefined
#endif
                    ));
                    break;
                default: break;
            }
        #endif
        HelioduinoUI *ui = new HelioduinoUI(uiCtrlSetup, uiDispSetup, SETUP_UI_LOGIC_LEVEL, SETUP_UI_ALLOW_INTERRUPTS, SETUP_UI_USE_TCUNICODE_FONTS);
        HELIO_SOFT_ASSERT(ui, SFP(HStr_Err_AllocationFailure));

        if (ui) {
            #if IS_SETUP_AS(SETUP_SYS_UI_MODE, Minimal)
                #if IS_SETUP_AS(SETUP_CONTROL_IN_MODE, RotaryEncoderOk) || IS_SETUP_AS(SETUP_CONTROL_IN_MODE, RotaryEncoderOkLR) ||\
                    IS_SETUP_AS(SETUP_CONTROL_IN_MODE, UpDownButtons) || IS_SETUP_AS(SETUP_CONTROL_IN_MODE, UpDownButtonsLR) ||\
                    IS_SETUP_AS(SETUP_CONTROL_IN_MODE, AnalogJoystickOk) || IS_SETUP_AS(SETUP_CONTROL_IN_MODE, Matrix2x2UpDownButtonsOkL) ||\
                    IS_SETUP_AS(SETUP_CONTROL_IN_MODE, Matrix3x4Keyboard_OptRotEncOk) || IS_SETUP_AS(SETUP_CONTROL_IN_MODE, Matrix3x4Keyboard_OptRotEncOkLR) ||\
                    IS_SETUP_AS(SETUP_CONTROL_IN_MODE, Matrix4x4Keyboard_OptRotEncOk) || IS_SETUP_AS(SETUP_CONTROL_IN_MODE, Matrix4x4Keyboard_OptRotEncOkLR) ||\
                    SETUP_UI_IS_DFROBOTSHIELD
                    ui->allocateStandardControls();
                #elif IS_SETUP_AS(SETUP_CONTROL_IN_MODE, UpDownESP32Touch) || IS_SETUP_AS(SETUP_CONTROL_IN_MODE, UpDownESP32TouchLR)
                    ui->allocateESP32TouchControl();
                #endif
                #if IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, LCD16x2_EN) || IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, LCD16x2_RS) ||\
                    IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, LCD20x4_EN) || IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, LCD20x4_RS) ||\
                    SETUP_UI_IS_DFROBOTSHIELD
                    ui->allocateLCDDisplay();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, SSD1305)
                    ui->allocateSSD1305Display();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, SSD1305_x32Ada)
                    ui->allocateSSD1305x32AdaDisplay();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, SSD1305_x64Ada)
                    ui->allocateSSD1305x64AdaDisplay();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, SSD1306)
                    ui->allocateSSD1306Display();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, SH1106)
                    ui->allocateSH1106Display();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, CustomOLED)
                    ui->allocateSSD1607GDDisplay();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, SSD1607)
                    ui->allocateSSD1607WSDisplay();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, IL3820)
                    ui->allocateIL3820Display();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, IL3820_V2)
                    ui->allocateIL3820V2Display();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ST7735)
                    ui->allocateST7735Display();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ST7789)
                    ui->allocateST7789Display();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, ILI9341)
                    ui->allocateILI9341Display();
                #elif IS_SETUP_AS(SETUP_DISPLAY_OUT_MODE, TFT)
                    ui->allocateTFTDisplay();
                #endif
                #if IS_SETUP_AS(SETUP_CONTROL_IN_MODE, TouchScreen)
                    ui->allocateTouchscreenControl();
                #elif IS_SETUP_AS(SETUP_CONTROL_IN_MODE, ResistiveTouch)
                    ui->allocateResistiveTouchControl();
                #elif IS_SETUP_AS(SETUP_CONTROL_IN_MODE, TFTTouch)
                    ui->allocateTFTTouchControl();
                #endif

                #if IS_SETUP_AS(SETUP_UI_REMOTE1_TYPE, Serial) || IS_SETUP_AS(SETUP_UI_REMOTE1_TYPE, UART)
                    ui->addSerialRemote(UARTDeviceSetup(&SETUP_UI_REMOTE1_UART));
                #elif IS_SETUP_AS(SETUP_UI_REMOTE1_TYPE, Simhub)
                    ui->addSimhubRemote(UARTDeviceSetup(&SETUP_UI_REMOTE1_UART));
                #elif IS_SETUP_AS(SETUP_UI_REMOTE1_TYPE, WiFi)
                    ui->addWiFiRemote(SETUP_UI_RC_NETWORKING_PORT);
                #elif IS_SETUP_AS(SETUP_UI_REMOTE1_TYPE, Ethernet)
                    ui->addEthernetRemote(SETUP_UI_RC_NETWORKING_PORT);
                #endif
                #if IS_SETUP_AS(SETUP_UI_REMOTE2_TYPE, Serial) || IS_SETUP_AS(SETUP_UI_REMOTE2_TYPE, UART)
                    ui->addSerialRemote(UARTDeviceSetup(&SETUP_UI_REMOTE2_UART));
                #elif IS_SETUP_AS(SETUP_UI_REMOTE2_TYPE, Simhub)
                    ui->addSimhubRemote(UARTDeviceSetup(&SETUP_UI_REMOTE2_UART));
                #elif IS_SETUP_AS(SETUP_UI_REMOTE2_TYPE, WiFi)
                    ui->addWiFiRemote(SETUP_UI_RC_NETWORKING_PORT);
                #elif IS_SETUP_AS(SETUP_UI_REMOTE2_TYPE, Ethernet)
                    ui->addEthernetRemote(SETUP_UI_RC_NETWORKING_PORT);
                #endif
            #else // Full
                #if NOT_SETUP_AS(SETUP_UI_REMOTE1_TYPE, Disabled)
                    ui->addRemote(JOIN(Helio_RemoteControl,SETUP_UI_REMOTE1_TYPE), UARTDeviceSetup(&SETUP_UI_REMOTE1_UART), SETUP_UI_RC_NETWORKING_PORT);
                #endif
                #if NOT_SETUP_AS(SETUP_UI_REMOTE2_TYPE, Disabled)
                    ui->addRemote(JOIN(Helio_RemoteControl,SETUP_UI_REMOTE2_TYPE), UARTDeviceSetup(&SETUP_UI_REMOTE2_UART), SETUP_UI_RC_NETWORKING_PORT);
                #endif
            #endif
            #ifdef SETUP_UI_USE_CLOCK_FONT
                ui->setOverviewClockFont(&SETUP_UI_USE_CLOCK_FONT);
            #endif
            #ifdef SETUP_UI_USE_DETAIL_FONT
                ui->setOverviewDetailFont(&SETUP_UI_USE_DETAIL_FONT);
            #endif
            #ifdef SETUP_UI_USE_OVERVIEW_FONT
                ui->setOverviewFont(&SETUP_UI_USE_OVERVIEW_FONT);
            #endif
            #ifdef SETUP_UI_USE_ITEM_FONT
                ui->setMenuItemFont(&SETUP_UI_USE_ITEM_FONT);
            #endif
            #ifdef SETUP_UI_USE_TITLE_FONT
                ui->setMenuTitleFont(&SETUP_UI_USE_TITLE_FONT);
            #endif
            #ifdef SETUP_UI_USE_MENU_FONT
                ui->setMenuFont(&SETUP_UI_USE_MENU_FONT);
            #endif
            helioController.enableUI(ui);
        }
    #endif
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

    // Begin external data storage devices for various library data.
    #if SETUP_EXTDATA_EEPROM_ENABLE
        beginStringsFromEEPROM(SETUP_EEPROM_STRINGS_ADDR);
        #ifdef HELIO_USE_GUI
            beginUIStringsFromEEPROM(SETUP_EEPROM_UIDSTRS_ADDR);
        #endif
    #endif
    #if SETUP_EXTDATA_SD_ENABLE
    {   String libPrefix(F(SETUP_EXTDATA_SD_LIB_PREFIX));
        beginStringsFromSDCard(libPrefix);
        #ifdef HELIO_USE_GUI
            beginUIStringsFromSDCard(libPrefix);
        #endif
    }
    #endif

    // Sets system config name used in any of the following inits.
    #if (defined(HELIO_USE_WIFI_STORAGE) && NOT_SETUP_AS(SETUP_SAVES_WIFISTORAGE_MODE, Disabled)) || \
        (SETUP_SD_CARD_SPI_CS >= 0 && NOT_SETUP_AS(SETUP_SAVES_SD_CARD_MODE, Disabled))
        helioController.setSystemConfigFilename(F(SETUP_SAVES_CONFIG_FILE));
    #endif
    // Sets the EEPROM memory address for system data.
    #if NOT_SETUP_AS(SETUP_EEPROM_DEVICE_TYPE, None) && NOT_SETUP_AS(SETUP_SAVES_EEPROM_MODE, Disabled)
        helioController.setSystemDataAddress(SETUP_EEPROM_SYSDATA_ADDR);
    #endif

    setupPinChannels();

    // Initializes controller with first initialization method that successfully returns.
    if (!(false
        #if defined(HELIO_USE_WIFI_STORAGE) && IS_SETUP_AS(SETUP_SAVES_WIFISTORAGE_MODE, Primary)
            || helioController.initFromWiFiStorage()
        #elif SETUP_SD_CARD_SPI_CS >= 0 && IS_SETUP_AS(SETUP_SAVES_SD_CARD_MODE, Primary)
            || helioController.initFromSDCard()
        #elif NOT_SETUP_AS(SETUP_EEPROM_DEVICE_TYPE, None) && IS_SETUP_AS(SETUP_SAVES_EEPROM_MODE, Primary)
            || helioController.initFromEEPROM()
        #endif
        #if defined(HELIO_USE_WIFI_STORAGE) && IS_SETUP_AS(SETUP_SAVES_WIFISTORAGE_MODE, Fallback)
            || helioController.initFromWiFiStorage()
        #elif SETUP_SD_CARD_SPI_CS >= 0 && IS_SETUP_AS(SETUP_SAVES_SD_CARD_MODE, Fallback)
            || helioController.initFromSDCard()
        #elif NOT_SETUP_AS(SETUP_EEPROM_DEVICE_TYPE, None) && IS_SETUP_AS(SETUP_SAVES_EEPROM_MODE, Fallback)
            || helioController.initFromEEPROM()
        #endif
        )) {
        // First time running controller, set up default initial empty environment.
        helioController.init(JOIN(Helio_SystemMode,SETUP_SYSTEM_MODE),
                             JOIN(Helio_MeasurementMode,SETUP_MEASURE_MODE),
                             JOIN(Helio_DisplayOutputMode,SETUP_DISPLAY_OUT_MODE),
                             JOIN(Helio_ControlInputMode,SETUP_CONTROL_IN_MODE));

        setupOnce();
        setupAlways();
        setupObjects();
    } else {
        setupAlways();
    }

    setupUI();

    // Launches controller into main operation.
    helioController.launch();
}

void loop()
{
    // Helioduino will manage most updates for us.
    helioController.update();
}

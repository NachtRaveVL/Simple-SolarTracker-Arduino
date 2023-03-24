/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino UI Defines
*/

#ifndef HelioUIDefines_H
#define HelioUIDefines_H

#if F_SPD >= 48000000                                       // Resolve an appropriate UI update speed (1-10)
#define HELIO_UI_UPDATE_SPEED           10
#elif F_SPD >= 32000000
#define HELIO_UI_UPDATE_SPEED           5
#elif F_SPD >= 16000000
#define HELIO_UI_UPDATE_SPEED           2
#endif

// The following sizes apply to all architectures
#define HELIO_UI_RENDERER_BUFFERSIZE    32                  // Buffer size for display renderers
#define HELIO_UI_STARFIELD_MAXSIZE      16                  // Starfield map maxsize
// The following sizes only apply to architectures that do not have STL support (AVR/SAM)
#define HELIO_UI_REMOTECONTROLS_MAXSIZE 2                   // Maximum array size for remote controls list (max # of remote controls)

// CustomOLED U8g2 device string
#ifndef HELIO_UI_CUSTOM_OLED_I2C
#define HELIO_UI_CUSTOM_OLED_I2C        U8G2_SSD1309_128X64_NONAME0_F_HW_I2C    // Custom OLED for i2c setup (must be _HW_I2C variant /w 2 init params: rotation, resetPin - Wire# not assertion checked since baked into define)
#endif
#ifndef HELIO_UI_CUSTOM_OLED_SPI
#define HELIO_UI_CUSTOM_OLED_SPI        U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI // Custom OLED for SPI setup (must be _4W_HW_SPI variant /w 4 init params: rotation, csPin, dcPin, resetPin - SPI# not assertion checked since baked into define)
#endif

#define HELIO_UI_I2C_LCD_BASEADDR       0x20                // Base address of I2C LiquidCrystalIO LCDs (bitwise or'ed with passed address - technically base address of i2c expander in use)
#define HELIO_UI_I2C_OLED_BASEADDR      0x78                // Base address of I2C U8g2 OLEDs (bitwise or'ed with passed address, some devices may use 0x7e)
#define HELIO_UI_BACKLIGHT_TIMEOUT      5 * SECS_PER_MIN    // Backlight timeout, in seconds
#define HELIO_UI_START_AT_OVERVIEW      true                // Starts at overview screen (true), else menu screen (false)
#define HELIO_UI_DEALLOC_AFTER_USE      defined(__AVR__)    // If screen data should be unloaded after use (true = lower memory usage, increased screens transition time), or stay memory-resident (false = higher memory usage, more instant screen transitions)
#define HELIO_UI_GFX_VARS_USES_SLIDER   true                // Default analog slider usage for graphical displays displaying variable value ranges
#define HELIO_UI_MENU_FONT_MAG_LEVEL    2                   // Menu font base magnification level

#define HELIO_UI_KEYREPEAT_SPEED        20                  // Default key press repeat speed, in ticks
#define HELIO_UI_REMOTESERVER_PORT      3333                // Default remote control server's listening port
#define HELIO_UI_2X2MATRIX_KEYS         "AB*#"              // 2x2 matrix keyboard keys (U,D,L,R)
#define HELIO_UI_3X4MATRIX_KEYS         "123456789*0#"      // 3x4 matrix keyboard keys (123,456,789,*0#)
#define HELIO_UI_4X4MATRIX_KEYS         "123A456B789C*0#D"  // 4x4 matrix keyboard keys (123A,456B,789C,*0#D)
#define HELIO_UI_MATRIX_ACTIONS         "#*AB"              // Assigned enter/select char, delete/exit char, back char, and next char on keyboard (also 2x2 matrix keyboard keys)
#define HELIO_UI_TFTTOUCH_USES_RAW      false               // Raw touch usage for TFTTouch

// Default graphical display theme base (CoolBlue, DarkMode)
#define HELIO_UI_GFX_DISP_THEME_BASE    CoolBlue            
#define HELIO_UI_GFX_DISP_THEME_SMLMED  SM
#define HELIO_UI_GFX_DISP_THEME_MEDLRG  ML


// Remote Control
// Type of remote control.
enum Helio_RemoteControl : signed char {
    Helio_RemoteControl_Disabled,                           // Disabled remote control
    Helio_RemoteControl_Serial,                             // Remote control by Serial/Bluetooth AT, requires UART setup
    Helio_RemoteControl_Simhub,                             // Remote control by Simhub serial connector, requires UART setup
    Helio_RemoteControl_WiFi,                               // Remote control by WiFi device, requires enabled WiFi
    Helio_RemoteControl_Ethernet,                           // Remote control by Ethernet device, requires enabled Ethernet

    Helio_RemoteControl_Count,                              // Placeholder
    Helio_RemoteControl_Undefined = -1                      // Placeholder
};

// Display Rotation
// Amount of display rotation, or in some cases mirror'ing.
enum Helio_DisplayRotation : signed char {
    Helio_DisplayRotation_R0,                               // 0° clockwise display rotation (0° counter-clockwise device mounting)
    Helio_DisplayRotation_R1,                               // 90° clockwise display rotation (90° counter-clockwise device mounting)
    Helio_DisplayRotation_R2,                               // 180° clockwise display rotation (180° counter-clockwise device mounting)
    Helio_DisplayRotation_R3,                               // 270° clockwise display rotation  (270° counter-clockwise device mounting)
    Helio_DisplayRotation_HorzMirror,                       // Horizontally mirrored (iff supported)
    Helio_DisplayRotation_VertMirror,                       // Vertically mirrored (iff supported)

    Helio_DisplayRotation_Count,                            // Placeholder
    Helio_DisplayRotation_Undefined = -1                    // Placeholder
};

// Touchscreen Orientation
// Touchscreens can be glued on differently than displays, so these allow finer touchscreen setup.
enum Helio_TouchscreenOrientation : signed char {
    Helio_TouchscreenOrientation_Same,                      // Keep same orientation as display rotation (converts display rotation to swapXY/invX/invY values)
    Helio_TouchscreenOrientation_None,                      // No applied orientation (no invX, invY, or swapXY)
    Helio_TouchscreenOrientation_InvertX,                   // Only invert X axis (no invY or swapXY)
    Helio_TouchscreenOrientation_InvertY,                   // Only invert Y axis (no invX or swapXY)
    Helio_TouchscreenOrientation_InvertXY,                  // Invert X & Y axis (no swapXY)
    Helio_TouchscreenOrientation_SwapXY,                    // Only swap X/Y coordinates (aka transpose, no invX or invY)
    Helio_TouchscreenOrientation_InvertX_SwapXY,            // Invert X axis, then swap X/Y coordinates (no invY)
    Helio_TouchscreenOrientation_InvertY_SwapXY,            // Invert Y axis, then swap X/Y coordinates (no invX)
    Helio_TouchscreenOrientation_InvertXY_SwapXY,           // Invert X & Y axis, then swap X/Y coordinates

    Helio_TouchscreenOrientation_Count,                     // Placeholder
    Helio_TouchscreenOrientation_Undefined = -1             // Placeholder
};

// Display Theme
// General color theme and aesthetics.
enum Helio_DisplayTheme : signed char {
    Helio_DisplayTheme_CoolBlue_ML,                         // Cool blue theme for medium to large color displays (larger fonts/more padding)
    Helio_DisplayTheme_CoolBlue_SM,                         // Cool blue theme for small to medium color displays (smaller fonts/less padding)
    Helio_DisplayTheme_DarkMode_ML,                         // Dark mode theme for medium to large color displays (larger fonts/more padding)
    Helio_DisplayTheme_DarkMode_SM,                         // Dark mode theme for small to medium color displays (smaller fonts/less padding)
    Helio_DisplayTheme_MonoOLED,                            // Monochrome/OLED theme for small to medium monochrome displays, /w standard border
    Helio_DisplayTheme_MonoOLED_Inv,                        // Monochrome/OLED theme for small to medium monochrome displays, /w inverted colors

    Helio_DisplayTheme_Count,                               // Placeholder
    Helio_DisplayTheme_Undefined = -1                       // Placeholder
};

// ST77XX Device Kind
// Special device kind identifier for common ST7735 B/S/R color tags and common ST7789 screen resolutions.
enum Helio_ST77XXKind : signed char {
    Helio_ST7735Tag_B                   = (int8_t)0xff,     // ST7735B B model
    Helio_ST7735Tag_Green               = (int8_t)0x00,     // ST7735S Green tag (1.8" TFT, /w offset such as WaveShare)
    Helio_ST7735Tag_Green18             = (int8_t)0x00,     // ST7735S 18Green tag (alias of Green)
    Helio_ST7735Tag_Red                 = (int8_t)0x01,     // ST7735R Red tag
    Helio_ST7735Tag_Red18               = (int8_t)0x01,     // ST7735R 18Red tag (alias of Red)
    Helio_ST7735Tag_Black               = (int8_t)0x02,     // ST7735S Black tag (1.8" TFT)
    Helio_ST7735Tag_Black18             = (int8_t)0x02,     // ST7735S 18Black tag (alias of Black)
    Helio_ST7735Tag_Green144            = (int8_t)0x01,     // ST7735R 144Green tag (1.44" TFT)
    Helio_ST7735Tag_Mini                = (int8_t)0x04,     // ST7735S Mini160x80 tag (0.96" TFT, 160x80, 12800 pixels - if inverted try Mini_Plugin)
    Helio_ST7735Tag_Mini_Plugin         = (int8_t)0x06,     // ST7735S Mini160x80_Plugin tag (0.96" TFT /w plug-in FPC, 160x80, 12800 pixels)
    Helio_ST7735Tag_Hallowing           = (int8_t)0x05,     // ST7735R Hallowing tag (various)

    Helio_ST7789Res_128x128             = (int8_t)0x10,     // ST7789 128x128 (0.85", 1.44" & 1.5" TFTs, 16384 pixels)
    Helio_ST7789Res_135x240,                                // ST7789 135x240 (1.14" TFT, 32400 pixels)
    Helio_ST7789Res_170x320,                                // ST7789 170x320 (1.9" TFT, 54400 pixels)
    Helio_ST7789Res_172x320,                                // ST7789 172x320 (1.47" TFT, 55040 pixels)
    Helio_ST7789Res_240x240,                                // ST7789 240x240 (1.3" & 1.54" TFT, 57600 pixels)
    Helio_ST7789Res_240x280,                                // ST7789 240x280 (1.69" TFTs, 67200 pixels)
    Helio_ST7789Res_240x320,                                // ST7789 240x320 (2", 2.4", & 2.8" TFTs, 76800 pixels)
    Helio_ST7789Res_CustomTFT,                              // Custom ST7789 TFT resolution (defined statically by TFT_GFX_WIDTH & TFT_GFX_HEIGHT - override via build defines, or edit directly)

    Helio_ST77XXKind_Undefined          = (int8_t)0xff,     // Placeholder
    Helio_ST7735Tag_Undefined           = (int8_t)0xff,     // Placeholder
    Helio_ST7789Res_Undefined           = (int8_t)0xff,     // Placeholder
    Helio_ST7789Res_Start               = Helio_ST7789Res_128x128 // ST7789 enum start (alias of 128x128)
};

// Backlight Operation Mode
// How the backlight gets handled. Derived from LCD usage.
enum Helio_BacklightMode : signed char {
    Helio_BacklightMode_Normal,                             // The backlight is active HIGH, standard amongst most displays
    Helio_BacklightMode_Inverted,                           // The backlight is active LOW, inverted ouput signal
    Helio_BacklightMode_PWM,                                // The backlight uses analog PWM for variable intensity control

    Helio_BacklightMode_Count,                              // Placeholder
    Helio_BacklightMode_Undefined = -1                      // Placeholder
};

// Rotary Encoder Speed
// Essentially how far the rotary encoder must physically travel before the UI responds (selection change, scroll to prev/next, etc.).
// Note: Smaller cycle length = faster item selection/scroll speed, but more physical precision required (accessibility concern).
enum Helio_EncoderSpeed : signed char {
    Helio_EncoderSpeed_FullCycle,                           // Detent after every full cycle of both signals, A and B
    Helio_EncoderSpeed_HalfCycle,                           // Detent on every position where A == B
    Helio_EncoderSpeed_QuarterCycle,                        // Detent after every signal change, A or B

    Helio_EncoderSpeed_Count,                               // Placeholder
    Helio_EncoderSpeed_Undefined = -1                       // Placeholder
};

// ESP32 Touch Key High Reference Voltage
// High reference voltage for press detection.
enum Helio_ESP32Touch_HighRef : signed char {
    Helio_ESP32Touch_HighRef_Keep,                          // No change
    Helio_ESP32Touch_HighRef_V_2V4,                         // 2.4v
    Helio_ESP32Touch_HighRef_V_2V5,                         // 2.5v
    Helio_ESP32Touch_HighRef_V_2V6,                         // 2.6v
    Helio_ESP32Touch_HighRef_V_2V7,                         // 2.7v
    Helio_ESP32Touch_HighRef_Max,                           // Max voltage

    Helio_ESP32Touch_HighRef_Count,                         // Placeholder
    Helio_ESP32Touch_HighRef_Undefined = -1                 // Placeholder
};

// ESP32 Touch Key Low Reference Voltage
// Low reference voltage for press detection.
enum Helio_ESP32Touch_LowRef : signed char {
    Helio_ESP32Touch_LowRef_Keep,                           // No change
    Helio_ESP32Touch_LowRef_V_0V5,                          // 0.5v
    Helio_ESP32Touch_LowRef_V_0V6,                          // 0.6v
    Helio_ESP32Touch_LowRef_V_0V7,                          // 0.7v
    Helio_ESP32Touch_LowRef_V_0V8,                          // 0.8v
    Helio_ESP32Touch_LowRef_Max,                            // Max voltage

    Helio_ESP32Touch_LowRef_Count,                          // Placeholder
    Helio_ESP32Touch_LowRef_Undefined = -1                  // Placeholder
};

// ESP32 Touch Key High Ref Volt Attenuation
// High reference voltage attenuation for press detection.
enum Helio_ESP32Touch_HighRefAtten : signed char {
    Helio_ESP32Touch_HighRefAtten_Keep,                     // No change
    Helio_ESP32Touch_HighRefAtten_V_1V5,                    // 1.5v
    Helio_ESP32Touch_HighRefAtten_V_1V,                     // 1v
    Helio_ESP32Touch_HighRefAtten_V_0V5,                    // 0.5v
    Helio_ESP32Touch_HighRefAtten_V_0V,                     // 0v
    Helio_ESP32Touch_HighRefAtten_Max,                      // Max voltage

    Helio_ESP32Touch_HighRefAtten_Count,                    // Placeholder
    Helio_ESP32Touch_HighRefAtten_Undefined = -1            // Placeholder
};


class HelioduinoBaseUI;
class HelioDisplayDriver;
class HelioInputDriver;
class HelioRemoteControl;
class HelioMenu;
class HelioHomeMenu;
class HelioOverview;
struct HelioUIData;


// tcMenu Callbacks
#define CALLBACK_FUNCTION
#define NO_ADDRESS                      0xffff              // No EEPROM address
extern void CALLBACK_FUNCTION gotoScreen(int id);
extern void CALLBACK_FUNCTION debugAction(int id);

#endif // /ifndef HelioUIDefines_H

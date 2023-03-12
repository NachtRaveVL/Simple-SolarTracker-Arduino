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

// The following sizes only matter for architectures that do not have STL support (AVR/SAM)
#define HELIO_UI_REMOTECONTROLS_MAXSIZE 2                   // Maximum array size for remote controls list (max # of remote controls)

#define HELIO_UI_RENDERER_BUFFERSIZE    30                  // Default buffer size for display renderer
#define HELIO_UI_I2C_LCD_BASEADDR       0x20                // Base address of I2C LiquidCrystalIO LCDs (bitwise or'ed with passed address)
#define HELIO_UI_I2C_OLED_BASEADDR      0x78                // Base address of I2C U8G2 OLEDs (bitwise or'ed with passed address)
#define HELIO_UI_KEYREPEAT_SPEED        20                  // Default key press repeat speed
#define HELIO_UI_REMOTESERVER_PORT      3333                // Default remote control server's listening port
#define HELIO_UI_3X4MATRIX_KEYS         "123456789*0#"      // Default 3x4 matrix keyboard keys
#define HELIO_UI_4X4MATRIX_KEYS         "123A456B789C*0#D"  // Default 4x4 matrix keyboard keys
#define HELIO_UI_MATRIX_ACTIONS         "#*AB"              // Default enter char, delete/exit char, back char, and next on keyboard (also default 2x2 matrix keyboard keys)
#define HELIO_UI_GFXTFT_USES_AN_SLIDER  true                // Default analog slider usage for AdafruitGFX/TFTe_SPI displays
#define HELIO_UI_TFTTOUCH_USES_RAW      false               // Default raw touch usage for TFTTouch

// Default graphical display theme base (CoolBlue, DarkMode)
#define HELIO_UI_GFX_DISP_THEME_BASE    CoolBlue            
#define HELIO_UI_GFX_DISP_THEME_SMLMED  SM
#define HELIO_UI_GFX_DISP_THEME_MEDLRG  ML


// ST77XX Device Tab
enum Helio_ST7735Tab : signed char {
    Helio_ST7735Tab_Green               = 0x00,             // Green tag
    Helio_ST7735Tab_Red                 = 0x01,             // Red tag
    Helio_ST7735Tab_Black               = 0x02,             // Black tag
    Helio_ST7735Tab_Green144            = 0x01,             // Green144 tag
    Helio_ST7735Tab_Mini160x80          = 0x04,             // Mini160x80 tag
    Helio_ST7735Tab_Hallowing           = 0x05,             // Hallowing tag
    Helio_ST7735Tab_Mini160x80_Plugin   = 0x06,             // Mini160x80_Plugin tag

    Helio_ST7735Tab_Undefined           = (int8_t)0xff      // Placeholder
};

// Display Orientation
enum Helio_DisplayOrientation : signed char {
    Helio_DisplayOrientation_R0,                            // Standard landscape orientation
    Helio_DisplayOrientation_R1,                            // 90 degree clockwise rotation
    Helio_DisplayOrientation_R2,                            // 180 degree clockwise rotation
    Helio_DisplayOrientation_R3,                            // 270 degree clockwise rotation
    Helio_DisplayOrientation_HorzMirror,                    // Horizontally mirrored (if supported)
    Helio_DisplayOrientation_VertMirror,                    // Vertically mirrored (if supported)

    Helio_DisplayOrientation_Count,                         // Placeholder
    Helio_DisplayOrientation_Undefined = -1                 // Placeholder
};

// Display Theme
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

// Remote Control
enum Helio_RemoteControl : signed char {
    Helio_RemoteControl_Disabled,                           // Disabled remote control
    Helio_RemoteControl_Serial,                             // Remote control by Serial or Bluetooth AT, requires UART setup
    Helio_RemoteControl_Simhub,                             // Remote control by Simhub serial connector, requires UART setup
    Helio_RemoteControl_WiFi,                               // Remote control by WiFi, requires enabled WiFi
    Helio_RemoteControl_Ethernet,                           // Remote control by Ethernet, requires enabled Ethernet

    Helio_RemoteControl_Count,                              // Placeholder
    Helio_RemoteControl_Undefined = -1                      // Placeholder
};

// Rotary Encoder Speed
enum Helio_EncoderSpeed : signed char {
    Helio_EncoderSpeed_FullCycle,                           // Detent after every full cycle of both signals, A and B
    Helio_EncoderSpeed_HalfCycle,                           // Detent on every position where A == B
    Helio_EncoderSpeed_QuarterCycle,                        // Detent after every signal change, A or B

    Helio_EncoderSpeed_Undefined = -1                       // Placeholder
};

// ESP32 Touch Key High Reference Voltage
enum Helio_ESP32Touch_HighRef : signed char {
    Helio_ESP32Touch_HighRef_Keep,                          // No change
    Helio_ESP32Touch_HighRef_V_2V4,                         // 2.4v
    Helio_ESP32Touch_HighRef_V_2V5,                         // 2.5v
    Helio_ESP32Touch_HighRef_V_2V6,                         // 2.6v
    Helio_ESP32Touch_HighRef_V_2V7,                         // 2.7v
    Helio_ESP32Touch_HighRef_Max,                           // Max voltage

    Helio_ESP32Touch_HighRef_Undefined = -1                 // Placeholder
};

// ESP32 Touch Key Low Reference Voltage
enum Helio_ESP32Touch_LowRef : signed char {
    Helio_ESP32Touch_LowRef_Keep,                           // No change
    Helio_ESP32Touch_LowRef_V_0V5,                          // 0.5v
    Helio_ESP32Touch_LowRef_V_0V6,                          // 0.6v
    Helio_ESP32Touch_LowRef_V_0V7,                          // 0.7v
    Helio_ESP32Touch_LowRef_V_0V8,                          // 0.8v
    Helio_ESP32Touch_LowRef_Max,                            // Max voltage

    Helio_ESP32Touch_LowRef_Undefined = -1                  // Placeholder
};

// ESP32 Touch Key High Ref Volt Attenuation
enum Helio_ESP32Touch_HighRefAtten : signed char {
    Helio_ESP32Touch_HighRefAtten_Keep,                     // No change
    Helio_ESP32Touch_HighRefAtten_V_1V5,                    // 1.5v
    Helio_ESP32Touch_HighRefAtten_V_1V,                     // 1v
    Helio_ESP32Touch_HighRefAtten_V_0V5,                    // 0.5v
    Helio_ESP32Touch_HighRefAtten_V_0V,                     // 0v
    Helio_ESP32Touch_HighRefAtten_Max,                      // Max voltage

    Helio_ESP32Touch_HighRefAtten_Undefined = -1            // Placeholder
};

// LCD Backlight Mode
enum Helio_BacklightMode : signed char {
    Helio_BacklightMode_Normal,                             // The backlight is active HIGH
    Helio_BacklightMode_Inverted,                           // The backlight is active LOW
    Helio_BacklightMode_PWM,                                // The backlight is connected directly to a PWM pin

    Helio_BacklightMode_Undefined = -1                      // Placeholder
};


class HydruinoBaseUI;
class HelioDisplayDriver;
class HelioInputDriver;
class HelioRemoteControl;
struct HelioUIData;

#endif // /ifndef HelioUIDefines_H

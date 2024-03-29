/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino UI Common Inlines
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioUIInlines_HPP
#define HelioUIInlines_HPP

struct LCDDisplaySetup;
struct PixelDisplaySetup;
struct ST7735DisplaySetup;
struct TFTDisplaySetup;
struct UIDisplaySetup;

struct RotaryControlSetup;
struct ButtonsControlSetup;
struct ESP32TouchControlSetup;
struct JoystickControlSetup;
struct MatrixControlSetup;
struct TouchscreenSetup;
struct UIControlSetup;

#include "HelioduinoUI.h"

// Returns the active base UI instance. Not guaranteed to be non-null.
inline HelioduinoBaseUI *getBaseUI() { return reinterpret_cast<HelioduinoBaseUI *>(getUI()); }

// Returns the first theme in parameter list that isn't undefined, allowing defaulting chains to be nicely defined.
inline Helio_DisplayTheme definedThemeElse(Helio_DisplayTheme theme1, Helio_DisplayTheme theme2) {
    return theme1 != Helio_DisplayTheme_Undefined ? theme1 : theme2;
}


// LCD Display Setup
struct LCDDisplaySetup {
    Helio_BacklightMode ledMode;        // LCD backlight pin mode (default: Helio_BacklightMode_Normal)
    bool isDFRobotShield;               // Using DF robot shield

    inline LCDDisplaySetup(Helio_BacklightMode ledModeIn = Helio_BacklightMode_Normal, bool isDFRobotShieldIn = false) : ledMode(ledModeIn), isDFRobotShield(isDFRobotShieldIn) { ; }

    static inline LCDDisplaySetup usingDFRobotShield() { return LCDDisplaySetup(Helio_BacklightMode_Normal, true); }
};

// Standard Pixel Display Setup (U8g2OLED & AdaGfx/AdaTFT)
struct PixelDisplaySetup {
    Helio_DisplayRotation rotation;     // Display orientation/rotation (default: R0)
    pintype_t dcPin;                    // DC/RS pin, else -1 (default: -1)
    pintype_t resetPin;                 // Optional reset/RST pin, else -1 (default: -1, note: Unused reset pin typically needs tied to HIGH for display to function)
    pintype_t ledPin;                   // Optional backlight/LED/BL pin, else -1 (default: -1, note: Unused backlight pin can optionally be tied typically to HIGH for always-on)
    Helio_BacklightMode ledMode;        // Backlight/LED/BL pin mode (default: Helio_BacklightMode_Normal)
    uint8_t ledBitRes;                  // Backlight PWM output bit resolution, if PWM
#ifdef ESP32
    uint8_t ledChannel;                 // Backlight PWM output channel (0 reserved for buzzer), if PWM/ESP
#endif
#ifdef ESP_PLATFORM
    float ledFrequency;                 // Backlight PWM output frequency, if PWM/ESP
#endif
    Helio_ST77XXKind st77Kind;          // ST7735 tag color or ST7789 screen resolution (default: undef/-1), if ST77XX

    inline PixelDisplaySetup(Helio_DisplayRotation rotationIn = Helio_DisplayRotation_R0, pintype_t dcPinIn = -1, pintype_t resetPinIn = -1, pintype_t ledPinIn = -1, Helio_BacklightMode ledModeIn = Helio_BacklightMode_Normal, uint8_t ledBitResIn = DAC_RESOLUTION,
#ifdef ESP32
                             uint8_t ledChannel = 1,
#endif
#ifdef ESP_PLATFORM
                             float ledFrequencyIn = 1000,
#endif
                             Helio_ST77XXKind st77KindIn = Helio_ST77XXKind_Undefined)
        : rotation(rotationIn), dcPin(dcPinIn), resetPin(resetPinIn), ledPin(ledPinIn), ledMode(ledModeIn), ledBitRes(ledBitResIn),
#ifdef ESP32
          ledChannel(ledChannelIn),
#endif
#ifdef ESP_PLATFORM
          ledFrequency(ledFrequencyIn),
#endif
          st77Kind(st77KindIn)
        { ; }
};

// Advanced TFT Display Setup (TFT_eSPI)
struct TFTDisplaySetup {
    Helio_DisplayRotation rotation;     // Display orientation/rotation (default: R0)
    pintype_t ledPin;                   // Optional backlight/LED/BL pin, else -1 (default: -1, note: Unused backlight pin can optionally be tied typically to HIGH for always-on)
    Helio_BacklightMode ledMode;        // Backlight/LED/BL pin mode (default: Helio_BacklightMode_Normal)
    uint8_t ledBitRes;                  // Backlight PWM output bit resolution, if PWM
#ifdef ESP32
    uint8_t ledChannel;                 // Backlight PWM output channel (0 reserved for buzzer), if PWM/ESP
#endif
#ifdef ESP_PLATFORM
    float ledFrequency;                 // Backlight PWM output frequency, if PWM/ESP
#endif
    Helio_ST77XXKind st77Kind;          // ST7735 tag color (default: undef/-1), if ST7735 (ST7789 enums not used)

    inline TFTDisplaySetup(Helio_DisplayRotation rotationIn = Helio_DisplayRotation_R0, pintype_t ledPinIn = -1, Helio_BacklightMode ledModeIn = Helio_BacklightMode_Normal, uint8_t ledBitResIn = DAC_RESOLUTION,
#ifdef ESP32
                           uint8_t ledChannel = 1,
#endif
#ifdef ESP_PLATFORM
                           float ledFrequencyIn = 1000,
#endif
                           Helio_ST77XXKind st77KindIn = Helio_ST77XXKind_Undefined)
        : rotation(rotationIn), ledPin(ledPinIn), ledMode(ledModeIn), ledBitRes(ledBitResIn),
#ifdef ESP32
          ledChannel(ledChannelIn),
#endif
#ifdef ESP_PLATFORM
          ledFrequency(ledFrequencyIn),
#endif
          st77Kind(st77KindIn)
        { ; }
};

// Combined UI Display Setup
// A union of the various UI display setup structures, to assist with user display output settings.
struct UIDisplaySetup {
    enum : signed char { None, LCD, Pixel, TFT } dispCfgType; // Display config type
    union {
        LCDDisplaySetup lcd;            // LCD display setup
        PixelDisplaySetup gfx;          // Pixel display setup
        TFTDisplaySetup tft;            // TFT display setup
    } dispCfgAs;                        // Display config data

    inline UIDisplaySetup() : dispCfgType(None), dispCfgAs{} { ; }
    inline UIDisplaySetup(LCDDisplaySetup displaySetup) : dispCfgType(LCD), dispCfgAs{.lcd=displaySetup} { ; }
    inline UIDisplaySetup(PixelDisplaySetup displaySetup) : dispCfgType(Pixel), dispCfgAs{.gfx=displaySetup} { ; }
    inline UIDisplaySetup(TFTDisplaySetup displaySetup) : dispCfgType(TFT), dispCfgAs{.tft=displaySetup} { ; }

    static inline UIDisplaySetup usingDFRobotShield() { return UIDisplaySetup(LCDDisplaySetup::usingDFRobotShield()); }

    inline Helio_DisplayRotation getDisplayRotation() const { return dispCfgType == Pixel ? dispCfgAs.gfx.rotation : dispCfgType == TFT ? dispCfgAs.tft.rotation : Helio_DisplayRotation_R0; }
    inline pintype_t getBacklightPin() const { return dispCfgType == LCD ? (pintype_t)(dispCfgAs.lcd.isDFRobotShield ? 10 : 3) : dispCfgType == Pixel ? dispCfgAs.gfx.ledPin : dispCfgType == TFT ? dispCfgAs.tft.ledPin : hpin_none; }
    inline Helio_BacklightMode getBacklightMode() const { return dispCfgType == LCD ? dispCfgAs.lcd.ledMode : dispCfgType == Pixel ? dispCfgAs.gfx.ledMode : dispCfgType == TFT ? dispCfgAs.tft.ledMode : Helio_BacklightMode_Normal; }
    inline uint8_t getBacklightBitRes() const { return dispCfgType == Pixel ? dispCfgAs.gfx.ledBitRes : dispCfgType == TFT ? dispCfgAs.tft.ledBitRes : DAC_RESOLUTION; }
#ifdef ESP32
    inline uint8_t getBacklightChannel() const { return dispCfgType == Pixel ? dispCfgAs.gfx.ledChannel : dispCfgType == TFT ? dispCfgAs.tft.ledChannel : 1; }
#endif
#ifdef ESP_PLATFORM
    inline float getBacklightFrequency() const { return dispCfgType == Pixel ? dispCfgAs.gfx.ledFrequency : dispCfgType == TFT ? dispCfgAs.tft.ledFrequency : 1000; }
#endif
};


// Rotary Encoder Input Setup
struct RotaryControlSetup {
    Helio_EncoderSpeed encoderSpeed;    // Encoder cycling speed (detent freq)

    inline RotaryControlSetup(Helio_EncoderSpeed encoderSpeedIn = Helio_EncoderSpeed_HalfCycle) : encoderSpeed(encoderSpeedIn) { ; }
};

// Up/Down Buttons Input Setup
struct ButtonsControlSetup {
    uint8_t repeatSpeed;                // Key repeat speed, in ticks (lower = faster)
    bool isDFRobotShield;               // Using DF robot shield

    inline ButtonsControlSetup(uint8_t repeatSpeedIn = HELIO_UI_KEYREPEAT_SPEED, bool isDFRobotShieldIn = false) : repeatSpeed(repeatSpeedIn), isDFRobotShield(isDFRobotShieldIn) { ; }

    static inline ButtonsControlSetup usingDFRobotShield() { return ButtonsControlSetup(HELIO_UI_KEYREPEAT_SPEED, true); }
};

// ESP32 Touch Keys Input Setup
struct ESP32TouchControlSetup {
    uint8_t repeatSpeed;                        // Key repeat speed, in ticks (lower = faster)
    uint16_t switchThreshold;                   // Switch activation threshold (default: 800)
    Helio_ESP32Touch_HighRef highVoltage;       // High reference voltage (default: V_2V7)
    Helio_ESP32Touch_LowRef lowVoltage;         // Low reference voltage (default: V_0V5)
    Helio_ESP32Touch_HighRefAtten attenuation;  // High reference voltage attention (default: V_1V)

    inline ESP32TouchControlSetup(uint8_t repeatSpeedIn = HELIO_UI_KEYREPEAT_SPEED, uint16_t switchThresholdIn = 800, Helio_ESP32Touch_HighRef highVoltageIn = Helio_ESP32Touch_HighRef_V_2V7, Helio_ESP32Touch_LowRef lowVoltageIn = Helio_ESP32Touch_LowRef_V_0V5, Helio_ESP32Touch_HighRefAtten attenuationIn = Helio_ESP32Touch_HighRefAtten_V_1V) : repeatSpeed(repeatSpeedIn), switchThreshold(switchThresholdIn), highVoltage(highVoltageIn), lowVoltage(lowVoltageIn), attenuation(attenuationIn) { ; }
};

// Analog Joystick Input Setup
struct JoystickControlSetup {
    millis_t repeatDelay;               // Repeat delay, in milliseconds (default: 750)
    float decreaseDivisor;              // Repeat decrease divisor

    inline JoystickControlSetup(millis_t repeatDelayIn = 750, float decreaseDivisorIn = 3.0f) : repeatDelay(repeatDelayIn), decreaseDivisor(decreaseDivisorIn) { ; }
};

// Matrix Keyboard Input Setup
struct MatrixControlSetup {
    millis_t repeatDelay;               // Repeat delay, in milliseconds
    millis_t repeatInterval;            // Repeat interval, in milliseconds
    Helio_EncoderSpeed encoderSpeed;    // Encoder cycling speed (optional)

    inline MatrixControlSetup(millis_t repeatDelayIn = 850, millis_t repeatIntervalIn = 350, Helio_EncoderSpeed encoderSpeedIn = Helio_EncoderSpeed_HalfCycle) : repeatDelay(repeatDelayIn), repeatInterval(repeatIntervalIn), encoderSpeed(encoderSpeedIn) { ; }
};

// Touchscreen Input Setup
struct TouchscreenSetup {
    Helio_TouchscreenOrientation orient; // Touchscreen orientation tuning (default: Same)
    #ifdef HELIO_UI_ENABLE_XPT2046TS
        SPIClass *spiClass;
    #endif

    #ifndef HELIO_UI_ENABLE_XPT2046TS
        inline TouchscreenSetup(Helio_TouchscreenOrientation orientIn = Helio_TouchscreenOrientation_Same) : orient(orientIn) { ; }
    #else
        inline TouchscreenSetup(Helio_TouchscreenOrientation orientIn = Helio_TouchscreenOrientation_Same, SPIClass *spiClassIn = HELIO_USE_SPI) : orient(orientIn), spiClass(spiClassIn) { ; }
    #endif
};

// Combined UI Control Setup
// A union of the various UI control setup structures, to assist with user control input settings.
struct UIControlSetup {
    enum : signed char { None, Encoder, Buttons, ESP32Touch, Joystick, Matrix, Touchscreen } ctrlCfgType; // Control config type
    union {
        RotaryControlSetup encoder;     // Rotary encoder setup
        ButtonsControlSetup buttons;    // Up/Down buttons setup
        ESP32TouchControlSetup espTouch; // ESP32 touch keys setup
        JoystickControlSetup joystick;  // Analog joystick setup
        MatrixControlSetup matrix;      // Matrix keyboard setup
        TouchscreenSetup touchscreen;   // Touchscreen setup
    } ctrlCfgAs;

    inline UIControlSetup() : ctrlCfgType(None), ctrlCfgAs{} { ; }
    inline UIControlSetup(RotaryControlSetup ctrlSetup) : ctrlCfgType(Encoder), ctrlCfgAs{.encoder=ctrlSetup} { ; }
    inline UIControlSetup(ButtonsControlSetup ctrlSetup) : ctrlCfgType(Buttons), ctrlCfgAs{.buttons=ctrlSetup} { ; }
    inline UIControlSetup(ESP32TouchControlSetup ctrlSetup) : ctrlCfgType(ESP32Touch), ctrlCfgAs{.espTouch=ctrlSetup} { ; }
    inline UIControlSetup(JoystickControlSetup ctrlSetup) : ctrlCfgType(Joystick), ctrlCfgAs{.joystick=ctrlSetup} { ; }
    inline UIControlSetup(MatrixControlSetup ctrlSetup) : ctrlCfgType(Matrix), ctrlCfgAs{.matrix=ctrlSetup} { ; }
    inline UIControlSetup(TouchscreenSetup ctrlSetup) : ctrlCfgType(Touchscreen), ctrlCfgAs{.touchscreen=ctrlSetup} { ; }

    static inline UIControlSetup usingDFRobotShield() { return UIControlSetup(ButtonsControlSetup::usingDFRobotShield()); }
};

#endif // /ifndef HelioUIInlines_HPP
#endif

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Input Drivers
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioInputDrivers_H
#define HelioInputDrivers_H

class HelioInputDriver;
class HelioInputRotary;
class HelioInputUpDownButtons;
class HelioInputESP32TouchKeys;
class HelioInputJoystick;
class HelioInputMatrix2x2;
class HelioInputMatrix3x4;
class HelioInputMatrix4x4;
class HelioInputResistiveTouch;
class HelioInputTouchscreen;
class HelioInputTFTTouch;

#include "HelioduinoUI.h"
#include "KeyboardManager.h"
#include "tcMenuKeyboard.h"
#include "graphics\MenuTouchScreenEncoder.h"
#include "JoystickSwitchInput.h"

// Input Driver Base
// Base input driver class that manages control input mode selection.
class HelioInputDriver {
public:
    HelioInputDriver(Pair<uint8_t, const pintype_t *> controlPins);

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) = 0;

    bool areAllPinsInterruptable() const;
    virtual bool areMainPinsInterruptable() const = 0;

    virtual IoAbstractionRef getIoAbstraction() = 0;

    inline const Pair<uint8_t, const pintype_t *> &getPins() const { return _pins; }

protected:
    const Pair<uint8_t, const pintype_t *> _pins;
};


// Rotary Encoder Input Driver
// Rotary encoder that uses a twisting motion, along with momentary push-down.
class HelioInputRotary : public HelioInputDriver {
public:
    HelioInputRotary(Pair<uint8_t, const pintype_t *> controlPins, Helio_EncoderSpeed encoderSpeed);
    virtual ~HelioInputRotary() = default;

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override;

    virtual IoAbstractionRef getIoAbstraction() override { return nullptr; }

    inline Helio_EncoderSpeed getEncoderSpeed() const { return _encoderSpeed; }

protected:
    const Helio_EncoderSpeed _encoderSpeed;
};


// Up/Down Buttons Input Driver
// Standard momentary buttons input.
class HelioInputUpDownButtons : public HelioInputDriver {
public:
    HelioInputUpDownButtons(Pair<uint8_t, const pintype_t *> controlPins, uint16_t keyRepeatSpeed);
    // Special constructor for DFRobotShield /w buttons (isDFRobotShield_unused tossed, only used for constructor resolution)
    HelioInputUpDownButtons(bool isDFRobotShield_unused, uint16_t keyRepeatSpeed);
    virtual ~HelioInputUpDownButtons();

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override;

    virtual IoAbstractionRef getIoAbstraction() override { return _dfRobotIORef; }

    inline uint16_t getKeySpeed() const { return _keySpeed; }

protected:
    const uint16_t _keySpeed;
    IoAbstractionRef _dfRobotIORef;
};


// ESP32 ESPTouch Keys Input Driver
// For ESP32 only, uses integrated touch keys library.
class HelioInputESP32TouchKeys : public HelioInputDriver {
public:
    HelioInputESP32TouchKeys(Pair<uint8_t, const pintype_t *> controlPins, uint16_t keyRepeatSpeed, uint16_t switchThreshold, Helio_ESP32Touch_HighRef highVoltage, Helio_ESP32Touch_LowRef lowVoltage, Helio_ESP32Touch_HighRefAtten attenuation);
    virtual ~HelioInputESP32TouchKeys() = default;

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override { return false; }

    #ifdef ESP32
        virtual IoAbstractionRef getIoAbstraction() override { return &_esp32Touch; }
    #else
        virtual IoAbstractionRef getIoAbstraction() override { return nullptr; }
    #endif

    inline uint16_t getKeySpeed() const { return _keySpeed; }

protected:
    const uint16_t _keySpeed;
    #ifdef ESP32
        ESP32TouchKeysAbstraction _esp32Touch;
    #endif
};


// Analog Joystick Input Driver
// Analog joystick control with momentary press button.
class HelioInputJoystick : public HelioInputDriver {
public:
    HelioInputJoystick(Pair<uint8_t, const pintype_t *> controlPins, millis_t repeatDelay, float decreaseDivisor, float jsCenterX = 0.5f, float jsCenterY = 0.5f, float jsZeroTol = 0.05f);
    virtual ~HelioInputJoystick() = default;

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override { return false; }

    virtual IoAbstractionRef getIoAbstraction() override { return &_joystickMultiIo; }

    inline millis_t getRepeatDelay() const { return _repeatDelay; }
    inline float getDecreaseDivisor() const { return _decreaseDivisor; }
    inline const float *getJoystickCalib() const { return _joystickCalib; }

protected:
    const millis_t _repeatDelay;
    const float _decreaseDivisor;
    const float _joystickCalib[3];
    MultiIoAbstraction _joystickMultiIo;
    AnalogJoystickToButtons _joystickIoXAxis;
};


// 2x2 Button Matrix Input Driver
// For matrix-style input with 2 rows and 2 columns of cross-tied momentary buttons.
class HelioInputMatrix2x2 : public HelioInputDriver {
public:
    HelioInputMatrix2x2(Pair<uint8_t, const pintype_t *> controlPins, millis_t repeatDelay, millis_t repeatInterval);
    virtual ~HelioInputMatrix2x2();

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override { return false; }

    virtual IoAbstractionRef getIoAbstraction() override { return nullptr; }

    inline MatrixKeyboardManager &getKeyboard() { return _keyboard; }

protected:
    MatrixKeyboardManager _keyboard;
    KeyboardLayout _keyboardLayout;
    MenuEditingKeyListener _tcMenuKeyListener;
};


// 3x4 Button Matrix Input Driver
// For matrix-style numeric input with 4 rows and 3 columns of cross-tied momentary buttons.
class HelioInputMatrix3x4 : public HelioInputDriver {
public:
    HelioInputMatrix3x4(Pair<uint8_t, const pintype_t *> controlPins, millis_t repeatDelay, millis_t repeatInterval, Helio_EncoderSpeed encoderSpeed);
    virtual ~HelioInputMatrix3x4();

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override { return _rotaryEncoder ? _rotaryEncoder->areMainPinsInterruptable() : false; }

    virtual IoAbstractionRef getIoAbstraction() override { return nullptr; }

    inline MatrixKeyboardManager &getKeyboard() { return _keyboard; }
    inline HelioInputRotary *getRotaryEncoder() { return _rotaryEncoder; }

protected:
    MatrixKeyboardManager _keyboard;
    KeyboardLayout _keyboardLayout;
    MenuEditingKeyListener _tcMenuKeyListener;
    HelioInputRotary *_rotaryEncoder;
};


// 4x4 Button Matrix Input Driver
// For matrix-style alpha-numeric input with 4 rows and 4 columns of cross-tied momentary buttons.
class HelioInputMatrix4x4 : public HelioInputDriver {
public:
    HelioInputMatrix4x4(Pair<uint8_t, const pintype_t *> controlPins, millis_t repeatDelay, millis_t repeatInterval, Helio_EncoderSpeed encoderSpeed = Helio_EncoderSpeed_HalfCycle);
    virtual ~HelioInputMatrix4x4();

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override { return _rotaryEncoder ? _rotaryEncoder->areMainPinsInterruptable() : false; }

    virtual IoAbstractionRef getIoAbstraction() override { return nullptr; }

    inline MatrixKeyboardManager &getKeyboard() { return _keyboard; }
    inline HelioInputRotary *getRotaryEncoder() { return _rotaryEncoder; }

protected:
    MatrixKeyboardManager _keyboard;
    KeyboardLayout _keyboardLayout;
    MenuEditingKeyListener _tcMenuKeyListener;
    HelioInputRotary *_rotaryEncoder;
};


// Resistive Touch Screen Input Driver
// A style of touch screen that uses resistive measurements for touch detection.
class HelioInputResistiveTouch : public HelioInputDriver {
public:
    HelioInputResistiveTouch(Pair<uint8_t, const pintype_t *> controlPins, HelioDisplayDriver *displayDriver);
    virtual ~HelioInputResistiveTouch() = default;

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override { return false; }

    virtual IoAbstractionRef getIoAbstraction() override { return nullptr; }

    inline MenuTouchScreenManager &getTouchScreen() { return _touchScreen; }

protected:
    iotouch::ResistiveTouchInterrogator _touchInterrogator;
    iotouch::TouchOrientationSettings _touchOrientation;
    MenuTouchScreenManager _touchScreen;
};


// Touch Screen Input Driver
// Standard touch screen driver using FT6206 (i2c based) or XPT2046 (SPI based).
// BSP touch interrogator uses TFT_GFX_WIDTH/HEIGHT for screen device width/height.
class HelioInputTouchscreen : public HelioInputDriver {
public:
    HelioInputTouchscreen(Pair<uint8_t, const pintype_t *> controlPins, Helio_DisplayRotation displayRotation);
    virtual ~HelioInputTouchscreen() = default;

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override { return false; }

    virtual IoAbstractionRef getIoAbstraction() override { return nullptr; }

    #ifdef HELIO_UI_ENABLE_XPT2046TS
        inline XPT2046_Touchscreen &getTouchScreen() { return _touchScreen; }
    #else
        inline Adafruit_FT6206 &getTouchScreen() { return _touchScreen; }
    #endif
protected:
    #ifdef HELIO_UI_ENABLE_XPT2046TS
        XPT2046_Touchscreen _touchScreen;
    #else
        Adafruit_FT6206 _touchScreen;
    #endif
    #ifdef HELIO_UI_ENABLE_BSP_TOUCH
        StBspTouchInterrogator _touchInterrogator;
    #else
        iotouch::AdaLibTouchInterrogator _touchInterrogator;
    #endif
    iotouch::TouchOrientationSettings _touchOrientation;
};


// TFT Touch Screen Input Driver
// Standard XPT2046 touch screen, but using TFT_eSPI library. Must be paired with TFTeSPI display driver.
class HelioInputTFTTouch : public HelioInputDriver {
public:
    HelioInputTFTTouch(Pair<uint8_t, const pintype_t *> controlPins, HelioDisplayTFTeSPI *displayDriver, bool useRawTouch = false);
    virtual ~HelioInputTFTTouch() = default;

    virtual void begin(MenuRenderer *renderer, MenuItem *initialItem) override;

    virtual bool areMainPinsInterruptable() const override { return false; }

    virtual IoAbstractionRef getIoAbstraction() override { return nullptr; }

    inline MenuTouchScreenManager &getTouchScreen() { return _touchScreen; }

protected:
    iotouch::TftSpiTouchInterrogator _touchInterrogator;
    iotouch::TouchOrientationSettings _touchOrientation;
    MenuTouchScreenManager _touchScreen;
};

#endif // /ifndef HelioInputDrivers_H
#endif

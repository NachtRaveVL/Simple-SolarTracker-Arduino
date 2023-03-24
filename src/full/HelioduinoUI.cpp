/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Full/RW UI
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioduinoFullUI::HelioduinoFullUI(UIControlSetup uiControlSetup, UIDisplaySetup uiDisplaySetup, bool isActiveLowIO, bool allowInterruptableIO, bool enableTcUnicodeFonts)
    : HelioduinoBaseUI(uiControlSetup, uiDisplaySetup, isActiveLowIO, allowInterruptableIO, enableTcUnicodeFonts)
{
    auto controller = getController();
    HELIO_HARD_ASSERT(controller, SFP(HStr_Err_InitializationFailure));

    if (controller) {
        // Input driver setup
        auto ctrlInMode = controller->getControlInputMode();
        auto ctrlInPins = controller->getControlInputPins();
        switch (ctrlInMode) {
            case Helio_ControlInputMode_RotaryEncoderOk:
            case Helio_ControlInputMode_RotaryEncoderOkLR: {
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Encoder, SFP(HStr_Err_InvalidParameter));
                _input = new HelioInputRotary(ctrlInPins, _uiCtrlSetup.ctrlCfgAs.encoder.encoderSpeed);
            } break;

            case Helio_ControlInputMode_UpDownButtonsOk:
            case Helio_ControlInputMode_UpDownButtonsOkLR: {
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Buttons, SFP(HStr_Err_InvalidParameter));
                if (!_uiCtrlSetup.ctrlCfgAs.buttons.isDFRobotShield) {
                    _input = new HelioInputUpDownButtons(ctrlInPins, _uiCtrlSetup.ctrlCfgAs.buttons.repeatSpeed);
                } else {
                    _input = new HelioInputUpDownButtons(true, _uiCtrlSetup.ctrlCfgAs.buttons.repeatSpeed);
                }
            } break;

            case Helio_ControlInputMode_UpDownESP32TouchOk:
            case Helio_ControlInputMode_UpDownESP32TouchOkLR: {
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::ESP32Touch, SFP(HStr_Err_InvalidParameter));
                _input = new HelioInputESP32TouchKeys(ctrlInPins, _uiCtrlSetup.ctrlCfgAs.espTouch.repeatSpeed, _uiCtrlSetup.ctrlCfgAs.espTouch.switchThreshold,
                                                      _uiCtrlSetup.ctrlCfgAs.espTouch.highVoltage, _uiCtrlSetup.ctrlCfgAs.espTouch.lowVoltage, _uiCtrlSetup.ctrlCfgAs.espTouch.attenuation);
            } break;

            case Helio_ControlInputMode_AnalogJoystickOk: {
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Joystick, SFP(HStr_Err_InvalidParameter));
                if (_uiData) {
                    _input = new HelioInputJoystick(ctrlInPins, _uiCtrlSetup.ctrlCfgAs.joystick.repeatDelay, _uiCtrlSetup.ctrlCfgAs.joystick.decreaseDivisor,
                                                    _uiData->joystickCalib[0], _uiData->joystickCalib[1], _uiData->joystickCalib[2]);
                } else {
                    _input = new HelioInputJoystick(ctrlInPins, _uiCtrlSetup.ctrlCfgAs.joystick.repeatDelay, _uiCtrlSetup.ctrlCfgAs.joystick.decreaseDivisor);
                }
            } break;

            case Helio_ControlInputMode_Matrix2x2UpDownButtonsOkL: {
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Matrix, SFP(HStr_Err_InvalidParameter));
                _input = new HelioInputMatrix2x2(ctrlInPins, _uiCtrlSetup.ctrlCfgAs.matrix.repeatDelay, _uiCtrlSetup.ctrlCfgAs.matrix.repeatInterval);
            } break;

            case Helio_ControlInputMode_Matrix3x4Keyboard_OptRotEncOk:
            case Helio_ControlInputMode_Matrix3x4Keyboard_OptRotEncOkLR: {
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Matrix, SFP(HStr_Err_InvalidParameter));
                _input = new HelioInputMatrix3x4(ctrlInPins, _uiCtrlSetup.ctrlCfgAs.matrix.repeatDelay, _uiCtrlSetup.ctrlCfgAs.matrix.repeatInterval,
                                                 _uiCtrlSetup.ctrlCfgAs.matrix.encoderSpeed);
            } break;

            case Helio_ControlInputMode_Matrix4x4Keyboard_OptRotEncOk:
            case Helio_ControlInputMode_Matrix4x4Keyboard_OptRotEncOkLR: {
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Matrix, SFP(HStr_Err_InvalidParameter));
                _input = new HelioInputMatrix4x4(ctrlInPins, _uiCtrlSetup.ctrlCfgAs.matrix.repeatDelay, _uiCtrlSetup.ctrlCfgAs.matrix.repeatInterval,
                                                 _uiCtrlSetup.ctrlCfgAs.matrix.encoderSpeed);
            } break;

            case Helio_ControlInputMode_TouchScreen: {
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Touchscreen, SFP(HStr_Err_InvalidParameter));
                #ifdef HELIO_UI_ENABLE_XPT2046TS
                    HELIO_SOFT_ASSERT(ctrlInPins.first && ctrlInPins.second && isValidPin(ctrlInPins.second[0]), SFP(HStr_Err_InvalidPinOrType));
                #endif
                _input = new HelioInputTouchscreen(ctrlInPins, _uiDispSetup.getDisplayRotation(), _uiCtrlSetup.ctrlCfgAs.touchscreen.orient);
            } break;

            default: break;
        }
        HELIO_SOFT_ASSERT(!(ctrlInMode >= Helio_ControlInputMode_RotaryEncoderOk && ctrlInMode != Helio_ControlInputMode_ResistiveTouch && ctrlInMode <= Helio_ControlInputMode_TouchScreen) || _input, SFP(HStr_Err_AllocationFailure));

        // Display driver setup
        auto dispOutMode = controller->getDisplayOutputMode();
        auto displaySetup = controller->getDisplaySetup();

        // LiquidCrystalIO supports only i2c, /w LCD setup
        HELIO_SOFT_ASSERT(!(dispOutMode >= Helio_DisplayOutputMode_LCD16x2_EN && dispOutMode <= Helio_DisplayOutputMode_LCD20x4_RS) || displaySetup.cfgType == DeviceSetup::I2CSetup, SFP(HStr_Err_InvalidParameter));
        HELIO_SOFT_ASSERT(!(dispOutMode >= Helio_DisplayOutputMode_LCD16x2_EN && dispOutMode <= Helio_DisplayOutputMode_LCD20x4_RS) || _uiDispSetup.dispCfgType == UIDisplaySetup::LCD, SFP(HStr_Err_InvalidParameter));
        // U8g2 supports either i2c or SPI, /w Pixel setup
        HELIO_SOFT_ASSERT(!(dispOutMode >= Helio_DisplayOutputMode_SSD1305 && dispOutMode <= Helio_DisplayOutputMode_CustomOLED) || (displaySetup.cfgType == DeviceSetup::I2CSetup || displaySetup.cfgType == DeviceSetup::SPISetup), SFP(HStr_Err_InvalidParameter));
        HELIO_SOFT_ASSERT(!(dispOutMode >= Helio_DisplayOutputMode_SSD1607 && dispOutMode <= Helio_DisplayOutputMode_IL3820_V2) || displaySetup.cfgType == DeviceSetup::SPISetup, SFP(HStr_Err_InvalidParameter));
        HELIO_SOFT_ASSERT(!(dispOutMode >= Helio_DisplayOutputMode_SSD1305 && dispOutMode <= Helio_DisplayOutputMode_IL3820_V2) || _uiDispSetup.dispCfgType == UIDisplaySetup::Pixel, SFP(HStr_Err_InvalidParameter));
        // AdafruitGFX supports only SPI, /w Pixel setup
        HELIO_SOFT_ASSERT(!(dispOutMode >= Helio_DisplayOutputMode_ST7735 && dispOutMode <= Helio_DisplayOutputMode_ILI9341) || displaySetup.cfgType == DeviceSetup::SPISetup, SFP(HStr_Err_InvalidParameter));
        HELIO_SOFT_ASSERT(!(dispOutMode >= Helio_DisplayOutputMode_ST7735 && dispOutMode <= Helio_DisplayOutputMode_ILI9341) || _uiDispSetup.dispCfgType == UIDisplaySetup::Pixel, SFP(HStr_Err_InvalidParameter));
        // TFT_eSPI supports only SPI, /w TFT setup
        HELIO_SOFT_ASSERT(dispOutMode != Helio_DisplayOutputMode_TFT || displaySetup.cfgType == DeviceSetup::SPISetup, SFP(HStr_Err_InvalidParameter));
        HELIO_SOFT_ASSERT(dispOutMode != Helio_DisplayOutputMode_TFT || _uiDispSetup.dispCfgType == UIDisplaySetup::TFT, SFP(HStr_Err_InvalidParameter));

        switch (dispOutMode) {
            // LiquidCrystalIO
            case Helio_DisplayOutputMode_LCD16x2_EN:
            case Helio_DisplayOutputMode_LCD16x2_RS:
            case Helio_DisplayOutputMode_LCD20x4_EN:
            case Helio_DisplayOutputMode_LCD20x4_RS: {
                if (!_uiDispSetup.dispCfgAs.lcd.isDFRobotShield) {
                    _display = new HelioDisplayLiquidCrystal(dispOutMode, displaySetup.cfgAs.i2c, _uiDispSetup.dispCfgAs.lcd.ledMode);
                } else {
                    _display = new HelioDisplayLiquidCrystal(true, displaySetup.cfgAs.i2c, _uiDispSetup.dispCfgAs.lcd.ledMode);
                }
            } break;

            // U8g2OLED
            case Helio_DisplayOutputMode_SSD1305: {
                if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305Wire(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305Wire1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305SPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305SPI1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;
            case Helio_DisplayOutputMode_SSD1305_x32Ada: {
                if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305x32AdaWire(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305x32AdaWire1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305x32AdaSPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305x32AdaSPI1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;
            case Helio_DisplayOutputMode_SSD1305_x64Ada: {
                if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305x64AdaWire(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305x64AdaWire1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305x64AdaSPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1305x64AdaSPI1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;
            case Helio_DisplayOutputMode_SSD1306: {
                if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1306Wire(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1306Wire1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1306SPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1306SPI1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;
            case Helio_DisplayOutputMode_SH1106: {
                if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE) {
                    _display = HelioDisplayU8g2OLED::allocateSH1106Wire(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::I2CSetup && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1) {
                    _display = HelioDisplayU8g2OLED::allocateSH1106Wire1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI) {
                    _display = HelioDisplayU8g2OLED::allocateSH1106SPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup && displaySetup.cfgAs.spi.spi == HELIO_USE_SPI1) {
                    _display = HelioDisplayU8g2OLED::allocateSH1106SPI1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;
            case Helio_DisplayOutputMode_CustomOLED: {
                if (displaySetup.cfgType == DeviceSetup::I2CSetup) {
                    _display = HelioDisplayU8g2OLED::allocateCustomOLEDI2C(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgType == DeviceSetup::SPISetup) {
                    _display = HelioDisplayU8g2OLED::allocateCustomOLEDSPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;
            case Helio_DisplayOutputMode_SSD1607: {
                if (displaySetup.cfgAs.spi.spi == HELIO_USE_SPI) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1607SPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgAs.spi.spi == HELIO_USE_SPI1) {
                    _display = HelioDisplayU8g2OLED::allocateSSD1607SPI1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;
            case Helio_DisplayOutputMode_IL3820: {
                if (displaySetup.cfgAs.spi.spi == HELIO_USE_SPI) {
                    _display = HelioDisplayU8g2OLED::allocateIL3820SPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgAs.spi.spi == HELIO_USE_SPI1) {
                    _display = HelioDisplayU8g2OLED::allocateIL3820SPI1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;
            case Helio_DisplayOutputMode_IL3820_V2: {
                if (displaySetup.cfgAs.spi.spi == HELIO_USE_SPI) {
                    _display = HelioDisplayU8g2OLED::allocateIL3820V2SPI(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else if (displaySetup.cfgAs.spi.spi == HELIO_USE_SPI1) {
                    _display = HelioDisplayU8g2OLED::allocateIL3820V2SPI1(displaySetup, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
                } else {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_InvalidParameter));
                }
            } break;

            // AdafruitGFX
            case Helio_DisplayOutputMode_ST7735: {
                _display = new HelioDisplayAdafruitGFX<Adafruit_ST7735>(displaySetup.cfgAs.spi, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.st77Kind, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
            } break;
            case Helio_DisplayOutputMode_ST7789: {
                _display = new HelioDisplayAdafruitGFX<Adafruit_ST7789>(displaySetup.cfgAs.spi, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.st77Kind, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
            } break;
            case Helio_DisplayOutputMode_ILI9341: {
                _display = new HelioDisplayAdafruitGFX<Adafruit_ILI9341>(displaySetup.cfgAs.spi, _uiDispSetup.dispCfgAs.gfx.rotation, _uiDispSetup.dispCfgAs.gfx.dcPin, _uiDispSetup.dispCfgAs.gfx.resetPin);
            } break;

            // TFT_eSPI
            case Helio_DisplayOutputMode_TFT: {
                HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
                #ifdef TFT_CS
                    HELIO_SOFT_ASSERT(displaySetup.cfgAs.spi.cs == TFT_CS, SFP(HStr_Err_NotConfiguredProperly));
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
                _display = new HelioDisplayTFTeSPI(displaySetup.cfgAs.spi, _uiDispSetup.dispCfgAs.tft.rotation, _uiDispSetup.dispCfgAs.tft.st77Kind);
            } break;

            default: break;
        }
        HELIO_SOFT_ASSERT(!(dispOutMode >= Helio_DisplayOutputMode_LCD16x2_EN && dispOutMode <= Helio_DisplayOutputMode_TFT) || _display, SFP(HStr_Err_AllocationFailure));

        // Late input driver setup
        switch (ctrlInMode) {
            case Helio_ControlInputMode_ResistiveTouch: {
                HELIO_SOFT_ASSERT(_display, SFP(HStr_Err_NotYetInitialized));
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Touchscreen, SFP(HStr_Err_InvalidParameter));
                _input = new HelioInputResistiveTouch(ctrlInPins, _display, _uiCtrlSetup.ctrlCfgAs.touchscreen.orient);
            } break;

            // TFT_eSPI
            case Helio_ControlInputMode_TFTTouch: {
                HELIO_SOFT_ASSERT(_display, SFP(HStr_Err_NotYetInitialized));
                HELIO_SOFT_ASSERT(dispOutMode == Helio_DisplayOutputMode_TFT, SFP(HStr_Err_InvalidParameter));
                HELIO_SOFT_ASSERT(_uiCtrlSetup.ctrlCfgType == UIControlSetup::Touchscreen, SFP(HStr_Err_InvalidParameter));
                HELIO_SOFT_ASSERT(ctrlInPins.first && ctrlInPins.second && isValidPin(ctrlInPins.second[0]), SFP(HStr_Err_InvalidPinOrType));
                #ifdef TOUCH_CS
                    HELIO_SOFT_ASSERT(ctrlInPins.first && ctrlInPins.second && ctrlInPins.second[0] == TOUCH_CS, SFP(HStr_Err_NotConfiguredProperly));
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
                _input = new HelioInputTFTTouch(ctrlInPins, (HelioDisplayTFTeSPI *)_display, _uiCtrlSetup.ctrlCfgAs.touchscreen.orient, HELIO_UI_TFTTOUCH_USES_RAW);
            } break;

            default: break;
        }
        HELIO_SOFT_ASSERT(!(ctrlInMode == Helio_ControlInputMode_ResistiveTouch || ctrlInMode == Helio_ControlInputMode_TFTTouch) || _input, SFP(HStr_Err_AllocationFailure));
    }
}

HelioduinoFullUI::~HelioduinoFullUI()
{ ; }

void HelioduinoFullUI::addRemote(Helio_RemoteControl rcType, UARTDeviceSetup rcSetup, uint16_t rcServerPort)
{
    HelioRemoteControl *remoteControl = nullptr;
    menuid_t statusMenuId = -1; // todo

    switch (rcType) {
        case Helio_RemoteControl_Serial: {
            remoteControl = new HelioRemoteSerialControl(rcSetup);
            HELIO_SOFT_ASSERT(remoteControl, SFP(HStr_Err_AllocationFailure));
        } break;

        case Helio_RemoteControl_Simhub: {
            remoteControl = new HelioRemoteSimhubControl(rcSetup, statusMenuId);
            HELIO_SOFT_ASSERT(remoteControl, SFP(HStr_Err_AllocationFailure));
        } break;

        case Helio_RemoteControl_WiFi: {
            #ifdef HELIO_USE_WIFI
                remoteControl = new HelioRemoteWiFiControl(rcServerPort);
                HELIO_SOFT_ASSERT(remoteControl, SFP(HStr_Err_AllocationFailure));
            #endif
        } break;

        case Helio_RemoteControl_Ethernet: {
            #ifdef HELIO_USE_ETHERNET
                remoteControl = new HelioRemoteEthernetControl(rcServerPort);
                HELIO_SOFT_ASSERT(remoteControl, SFP(HStr_Err_AllocationFailure));
            #endif
        } break;

        default: break;
    }

    if (remoteControl && remoteControl->getConnection()) {
        if (!_remoteServer) { _remoteServer = new TcMenuRemoteServer(getApplicationInfo()); }
        if (_remoteServer) { _remoteServer->addConnection(remoteControl->getConnection()); }
        _remotes.push_back(remoteControl);
    } else {
        if (remoteControl) { delete remoteControl; }
    }
}

bool HelioduinoFullUI::isFullUI()
{
    return true;
}

#endif

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Display Drivers
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioDisplayDrivers_H
#define HelioDisplayDrivers_H

class HelioDisplayDriver;
class HelioDisplayLiquidCrystal;
class HelioDisplayU8g2OLED;
template <class T> class HelioDisplayAdafruitGFX;
class HelioDisplayTFTeSPI;

#include "HelioduinoUI.h"

// Driver Display Base
// Base display driver class that manages display output mode selection.
class HelioDisplayDriver {
public:
    HelioDisplayDriver(Helio_DisplayRotation displayRotation = Helio_DisplayRotation_Undefined);
    virtual ~HelioDisplayDriver() = default;

    virtual void initBaseUIFromDefaults() = 0;
    virtual void begin() = 0;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) = 0;

    void setupRendering(uint8_t titleMode, Helio_DisplayTheme displayTheme, const void *itemFont = nullptr, const void *titleFont = nullptr, bool analogSlider = false, bool editingIcons = false, bool utf8Fonts = false);

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool withRot = true) const = 0;
    virtual bool isLandscape(bool withRot = true) const = 0;
    inline bool isPortrait(bool withRot = true) const { return !isLandscape(withRot); }
    inline bool isSquare() const { return getScreenSize().first == getScreenSize().second; }
    virtual uint8_t getScreenBits() const = 0;
    inline bool isMonochrome() const { return getScreenBits() == 1; }
    inline bool isColor() const { return getScreenBits() > 1; }
    inline bool is16BitColor() const { return getScreenBits() == 16; }
    inline bool isFullColor() const { return getScreenBits() == 24; }

    virtual BaseMenuRenderer *getBaseRenderer() = 0;
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() = 0;

    inline Helio_DisplayRotation getRotation() const { return _rotation; }
    inline Helio_DisplayTheme getDisplayTheme() const { return _displayTheme; }

protected:
    const Helio_DisplayRotation _rotation;
    Helio_DisplayTheme _displayTheme;
};


// Liquid Crystal Display Driver
// Display driver for text-only monochrome LCDs, typically ones that talk through a PCF857X i2c expander or similar.
// Note: Parallel 6800/8080 raw data connections are not supported at this time.
class HelioDisplayLiquidCrystal : public HelioDisplayDriver {
public:
    HelioDisplayLiquidCrystal(Helio_DisplayOutputMode displayMode, I2CDeviceSetup displaySetup, Helio_BacklightMode ledMode = Helio_BacklightMode_Normal);
    // Special constructor for DFRobotShield /w 16x2 LCD (isDFRobotShield_unused tossed, only used for constructor resolution)
    HelioDisplayLiquidCrystal(bool isDFRobotShield_unused, I2CDeviceSetup displaySetup, Helio_BacklightMode ledMode = Helio_BacklightMode_Normal);
    virtual ~HelioDisplayLiquidCrystal() = default;

    virtual void initBaseUIFromDefaults() override;
    virtual void begin() override;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) override;

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool withRot = true) const override { return isLandscape(withRot) ? make_pair((uint16_t)max(_screenSize[0],_screenSize[1]), (uint16_t)min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair((uint16_t)min(_screenSize[0],_screenSize[1]), (uint16_t)max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool withRot = true) const override { return _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 1; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return nullptr; }

    inline LiquidCrystal &getLCD() { return _lcd; }

protected:
    uint8_t _screenSize[2];
    LiquidCrystal _lcd;
    LiquidCrystalRenderer _renderer;
};


// Monochrome OLED Display Driver
// Display driver for smaller monochrome OLED LCDs, typically less than 1.5" in size.
// Note: CustomOLED option uses HELIO_UI_CUSTOM_OLED_* defines for custom OLED support.
class HelioDisplayU8g2OLED : public HelioDisplayDriver {
public:
    HelioDisplayU8g2OLED(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, U8G2 *gfx);
    virtual ~HelioDisplayU8g2OLED();

    virtual void initBaseUIFromDefaults() override;
    virtual void begin() override;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) override;

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool withRot = true) const override { return isLandscape(withRot) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool withRot = true) const override { return withRot ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 1; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return _renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return _renderer; }

    inline U8G2 &getGfx() { return *_gfx; }
    #ifdef HELIO_UI_ENABLE_STCHROMA_LDTC
        inline StChromaArtDrawable &getDrawable() { return *_drawable; }
    #else
        inline U8g2Drawable &getDrawable() { return *_drawable; }
    #endif

protected:
    uint16_t _screenSize[2];
    U8G2 *_gfx;
    #ifdef HELIO_UI_ENABLE_STCHROMA_LDTC
        StChromaArtDrawable *_drawable;
    #else
        U8g2Drawable *_drawable;
    #endif
    GraphicsDeviceRenderer *_renderer;

public:
    static inline HelioDisplayU8g2OLED *allocateSSD1305SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305Wire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305Wire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305x32AdaSPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305x32AdaSPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305x32AdaWire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305x32AdaWire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305x64AdaSPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305x64AdaSPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305x64AdaWire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1305x64AdaWire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1306SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1306SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1306Wire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1306Wire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSH1106SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSH1106SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSH1106Wire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSH1106Wire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateCustomOLEDI2C(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateCustomOLEDSPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1607SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateSSD1607SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateIL3820SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateIL3820SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateIL3820V2SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    static inline HelioDisplayU8g2OLED *allocateIL3820V2SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
};


// Generic AdafruitGFX Display Driver
// A generic base AdafruitGFX driver that serves as a template for all general AdafruitGFX color display drivers.
// Note: It is likely that the initializer/begin calls used will not be correct for any given AdafruitGFX class used. Consider always specializing.
template <class T>
class HelioDisplayAdafruitGFX : public HelioDisplayDriver {
public:
    HelioDisplayAdafruitGFX(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    virtual ~HelioDisplayAdafruitGFX() = default;

    virtual void initBaseUIFromDefaults() override;
    virtual void begin() override;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) override;

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool withRot = true) const override { return isLandscape(withRot) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool withRot = true) const override { return withRot ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 16; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    inline T &getGfx() { return _gfx; }
    inline AdafruitDrawable<T> &getDrawable() { return _drawable; }

protected:
    uint16_t _screenSize[2];
    T _gfx;
    AdafruitDrawable<T> _drawable;
    GraphicsDeviceRenderer _renderer;
};


// ST7735 AdafruitSPITFT Display Driver
// Advanced color display. Modern & widely available, typically less than 1" in size.
// Note: Requires proper ST7735 tag color enum (ST7735 section of Helio_ST77XXKind). Custom resolutions are not supported.
template <>
class HelioDisplayAdafruitGFX<Adafruit_ST7735> : public HelioDisplayDriver {
public:
    HelioDisplayAdafruitGFX(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, Helio_ST77XXKind st77Kind, pintype_t dcPin, pintype_t resetPin);
    virtual ~HelioDisplayAdafruitGFX() = default;

    virtual void initBaseUIFromDefaults() override;
    virtual void begin() override;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) override;

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool withRot = true) const override { return isLandscape(withRot) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool withRot = true) const override { return withRot ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 16; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    inline Adafruit_ST7735 &getGfx() { return _gfx; }
    inline AdafruitDrawable<Adafruit_ST7735> &getDrawable() { return _drawable; }

protected:
    uint16_t _screenSize[2];
    const Helio_ST77XXKind _kind;
    Adafruit_ST7735 _gfx;
    AdafruitDrawable<Adafruit_ST7735> _drawable;
    GraphicsDeviceRenderer _renderer;
};


// ST7789 AdafruitSPITFT Display Driver
// Advanced color display. Modern & widely available, typically in the 1" to 3" size range.
// Note: Requires proper ST7789 screen resolution enum (ST7789 section of Helio_ST77XXKind).
// Note: CustomTFT option uses TFT_GFX_WIDTH & TFT_GFX_HEIGHT for custom screen size.
template <>
class HelioDisplayAdafruitGFX<Adafruit_ST7789> : public HelioDisplayDriver {
public:
    HelioDisplayAdafruitGFX(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, Helio_ST77XXKind st77Kind, pintype_t dcPin, pintype_t resetPin);
    virtual ~HelioDisplayAdafruitGFX() = default;

    virtual void initBaseUIFromDefaults() override;
    virtual void begin() override;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) override;

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool withRot = true) const override { return isLandscape(withRot) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool withRot = true) const override { return withRot ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 16; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    inline Adafruit_ST7789 &getGfx() { return _gfx; }
    inline AdafruitDrawable<Adafruit_ST7789> &getDrawable() { return _drawable; }

protected:
    uint16_t _screenSize[2];
    const Helio_ST77XXKind _kind;
    Adafruit_ST7789 _gfx;
    AdafruitDrawable<Adafruit_ST7789> _drawable;
    GraphicsDeviceRenderer _renderer;
};


// ILI9341 AdafruitSPITFT Display Driver
// Advanced color display. Modern & widely-available, typically in the 2" to 3" size range.
template <>
class HelioDisplayAdafruitGFX<Adafruit_ILI9341> : public HelioDisplayDriver {
public:
    HelioDisplayAdafruitGFX(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin);
    virtual ~HelioDisplayAdafruitGFX() = default;

    virtual void initBaseUIFromDefaults() override;
    virtual void begin() override;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) override;

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool withRot = true) const override { return isLandscape(withRot) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool withRot = true) const override { return withRot ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 16; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    inline Adafruit_ILI9341 &getGfx() { return _gfx; }
    inline AdafruitDrawable<Adafruit_ILI9341> &getDrawable() { return _drawable; }

protected:
    uint16_t _screenSize[2];
    Adafruit_ILI9341 _gfx;
    AdafruitDrawable<Adafruit_ILI9341> _drawable;
    GraphicsDeviceRenderer _renderer;
};


// TFT_eSPI Display Driver
// Fastest library for working with TFT displays, but requires advanced setup.
// Library always compiles statically - changes will always require sketch modify/re-upload.
// Drivers supported: ILI9341, ILI9341/alt, ST7735, ILI9163, S6D02A1, ILI9486/rpi, HX8357D,
//                    ILI9481, ILI9486, ILI9488, ST7789, ST7789/min, R61581, RM68140, ST7796,
//                    SSD1351, SSD1963/480, SSD1963/800, SSD1963/800alt, ILI9225, GC9A01
// Note: Usage of TFT_eSPI requires library setup via its TFT_eSPI/User_Setup.h.
// Note: Uses TFT_GFX_WIDTH & TFT_GFX_HEIGHT for custom screen size, which defaults to
//       TFT_WIDTH & TFT_HEIGHT as defined in User_Setup.h (if not overriden by build defines).
class HelioDisplayTFTeSPI : public HelioDisplayDriver {
public:
    HelioDisplayTFTeSPI(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, Helio_ST77XXKind st77Kind = Helio_ST77XXKind_Undefined);
    virtual ~HelioDisplayTFTeSPI() = default;

    virtual void initBaseUIFromDefaults() override;
    virtual void begin() override;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) override;

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool withRot = true) const override { return isLandscape(withRot) ? make_pair(max(TFT_GFX_WIDTH,TFT_GFX_HEIGHT), min(TFT_GFX_WIDTH,TFT_GFX_HEIGHT))
                                                                                                                    : make_pair(min(TFT_GFX_WIDTH,TFT_GFX_HEIGHT), max(TFT_GFX_WIDTH,TFT_GFX_HEIGHT)); }
    virtual bool isLandscape(bool withRot = true) const override { return withRot ? (TFT_GFX_WIDTH >= TFT_GFX_HEIGHT ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : TFT_GFX_WIDTH >= TFT_GFX_HEIGHT; }
    inline bool isRound() const
        #ifdef GC9A01_DRIVER
            { return TFT_GFX_WIDTH == TFT_GFX_HEIGHT; }
        #else
            { return false; }
        #endif
    virtual uint8_t getScreenBits() const override { return 24; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    inline TFT_eSPI &getGfx() { return _gfx; }
    inline TfteSpiDrawable &getDrawable() { return _drawable; }

protected:
    const Helio_ST77XXKind _kind;
    TFT_eSPI _gfx;
    TfteSpiDrawable _drawable;
    GraphicsDeviceRenderer _renderer;
};

#endif // /ifndef HelioDisplayDrivers_H
#endif

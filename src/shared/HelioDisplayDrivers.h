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
    inline HelioDisplayDriver(Helio_DisplayRotation displayRotation = Helio_DisplayRotation_Undefined, uint16_t screenWidth = 0, uint16_t screenHeight = 0) : _rotation(displayRotation), _displayTheme(Helio_DisplayTheme_Undefined), _screenSize{screenWidth, screenHeight} { ; }
    virtual ~HelioDisplayDriver() = default;

    // Uses display driver to initialize base UI with its own unique defaults.
    // Called when no stored UI data loaded.
    virtual void initBaseUIFromDefaults() = 0;
    // Begins the display driver.
    virtual void begin() = 0;

    // Allocates display-driver specific system-at-a-glance overview screen
    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) = 0;

    // Setups any graphical rendering options, including theme, font usage, and default editing icongraphy
    virtual void setupRendering(Helio_DisplayTheme displayTheme, Helio_TitleMode titleMode, const void *itemFont = nullptr, const void *titleFont = nullptr, bool analogSlider = false, bool editingIcons = false, bool tcUnicodeFonts = false);

    // Screen size accessor, with or without current rotation mode applied
    // Note: May return invalid screen size until after display driver is began.
    virtual Pair<uint16_t,uint16_t> getScreenSize(bool rotated = true) const = 0;
    // Determines if screen is in landscape screen mode, with or without current rotation mode included
    virtual bool isLandscape(bool rotated = true) const = 0;
    // Determines if screen is in portrait screen mode, with or without current rotation mode included
    inline bool isPortrait(bool rotated = true) const { return !isLandscape(rotated); }
    // Determines if screen is in square screen mode
    inline bool isSquare() const { return getScreenSize().first == getScreenSize().second; }
    // Screen bit depth accessor
    virtual uint8_t getScreenBits() const = 0;
    // Returns true if screen is monochrome
    inline bool isMonochrome() const { return getScreenBits() == 1; }
    // Returns true if screen is color (not monochrome)
    inline bool isColor() const { return !isMonochrome(); }
    // Returns true if screen is 16-bpp rgb565 color mode
    inline bool is16BitColor() const { return getScreenBits() == 16; }
    // Returns true if screen is 24-bpp rgb888 color mode
    inline bool isFullColor() const { return getScreenBits() == 24; }

    // System name accessor, guaranteeing PGM memory under AVR/ESP_H at expense of custom system naming
    static inline const char *getSystemName()
        #if (defined __AVR__ || defined ESP_H) && !defined __MBED__
            { return CFP(HStr_Default_SystemName); }
        #else
            { return getController() ? getController()->getSystemNameChars() : CFP(HStr_Default_SystemName); }
        #endif

    // Base menu renderer accessor, else nullptr
    virtual BaseMenuRenderer *getBaseRenderer() = 0;
    // Graphical device renderer accessor, else nullptr
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() = 0;

    // Rotation accessor
    inline Helio_DisplayRotation getRotation() const { return _rotation; }
    // Display theme accessor
    inline Helio_DisplayTheme getDisplayTheme() const { return _displayTheme; }

protected:
    const Helio_DisplayRotation _rotation;
    Helio_DisplayTheme _displayTheme;
    uint16_t _screenSize[2];
};


// Liquid Crystal Display Driver
// Display driver for text-only monochrome LCDs, typically ones that talk through a PCF857X i2c expander or similar.
// Note: Parallel 6800/8080 raw data connections are not supported.
class HelioDisplayLiquidCrystal : public HelioDisplayDriver {
public:
    HelioDisplayLiquidCrystal(Helio_DisplayOutputMode displayMode, I2CDeviceSetup displaySetup, Helio_BacklightMode ledMode = Helio_BacklightMode_Normal);
    // Special constructor for DFRobotShield /w 16x2 LCD (isDFRobotShield_unused tossed, only used for constructor resolution)
    HelioDisplayLiquidCrystal(bool isDFRobotShield_unused, I2CDeviceSetup displaySetup, Helio_BacklightMode ledMode = Helio_BacklightMode_Normal);
    virtual ~HelioDisplayLiquidCrystal() = default;

    virtual void initBaseUIFromDefaults() override;
    virtual void begin() override;

    virtual void setupRendering(Helio_DisplayTheme displayTheme, Helio_TitleMode titleMode, const void *itemFont = nullptr, const void *titleFont = nullptr, bool analogSlider = false, bool editingIcons = false, bool tcUnicodeFonts = false) override;

    virtual HelioOverview *allocateOverview(const void *clockFont = nullptr, const void *detailFont = nullptr) override;

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool rotated = true) const override { return isLandscape(rotated) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool rotated = true) const override { return _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 1; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return nullptr; }

    // LCD accessor
    inline LiquidCrystal &getLCD() { return _lcd; }

protected:
    LiquidCrystal _lcd;                                     // LCD instance
    LiquidCrystalRenderer _renderer;                        // LCD renderer
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

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool rotated = true) const override { return isLandscape(rotated) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool rotated = true) const override { return !rotated ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                       : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                   : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 1; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return _renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return _renderer; }

    // GFX accessor
    inline U8G2 &getGfx() { return *_gfx; }
#ifdef HELIO_UI_ENABLE_STCHROMA_LDTC
    // Drawable accessor
    inline StChromaArtDrawable &getDrawable() { return *_drawable; }
#else
    // Drawable accessor
    inline U8g2Drawable &getDrawable() { return *_drawable; }
#endif

protected:
    U8G2 *_gfx;                                             // Graphics device instance
#ifdef HELIO_UI_ENABLE_STCHROMA_LDTC
    StChromaArtDrawable *_drawable;                         // StChromaArt drawable
#else
    U8g2Drawable *_drawable;                                // U8g2 drawable
#endif
    GraphicsDeviceRenderer *_renderer;                      // Graphics renderer

public: // U8g2 factory
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

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool rotated = true) const override { return isLandscape(rotated) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool rotated = true) const override { return rotated ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 16; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    // GFX accessor
    inline T &getGfx() { return _gfx; }
    // Drawable accessor
    inline AdafruitDrawable<T> &getDrawable() { return _drawable; }

protected:
    T _gfx;                                                 // GFX instance
    AdafruitDrawable<T> _drawable;                          // AdafruitGFX drawable
    GraphicsDeviceRenderer _renderer;                       // Graphics renderer
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

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool rotated = true) const override { return isLandscape(rotated) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool rotated = true) const override { return rotated ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 16; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    // GFX accessor
    inline Adafruit_ST7735 &getGfx() { return _gfx; }
    // Drawable accessor
    inline AdafruitDrawable<Adafruit_ST7735> &getDrawable() { return _drawable; }

protected:
    const Helio_ST77XXKind _kind;                           // ST7735 kind enum
    Adafruit_ST7735 _gfx;                                   // GFX instance
    AdafruitDrawable<Adafruit_ST7735> _drawable;            // AdafruitGFX drawable
    GraphicsDeviceRenderer _renderer;                       // Graphics renderer
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

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool rotated = true) const override { return isLandscape(rotated) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool rotated = true) const override { return rotated ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 16; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    // GFX accessor
    inline Adafruit_ST7789 &getGfx() { return _gfx; }
    // Drawable accessor
    inline AdafruitDrawable<Adafruit_ST7789> &getDrawable() { return _drawable; }

protected:
    const Helio_ST77XXKind _kind;                           // ST7789 kind enum
    Adafruit_ST7789 _gfx;                                   // GFX instance
    AdafruitDrawable<Adafruit_ST7789> _drawable;            // AdafruitGFX drawable
    GraphicsDeviceRenderer _renderer;                       // Graphics renderer
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

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool rotated = true) const override { return isLandscape(rotated) ? make_pair(max(_screenSize[0],_screenSize[1]), min(_screenSize[0],_screenSize[1]))
                                                                                                                    : make_pair(min(_screenSize[0],_screenSize[1]), max(_screenSize[0],_screenSize[1])); }
    virtual bool isLandscape(bool rotated = true) const override { return rotated ? (_screenSize[0] >= _screenSize[1] ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : _screenSize[0] >= _screenSize[1]; }
    virtual uint8_t getScreenBits() const override { return 16; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    // GFX accessor
    inline Adafruit_ILI9341 &getGfx() { return _gfx; }
    // Drawable accessor
    inline AdafruitDrawable<Adafruit_ILI9341> &getDrawable() { return _drawable; }

protected:
    Adafruit_ILI9341 _gfx;                                  // GFX instance
    AdafruitDrawable<Adafruit_ILI9341> _drawable;           // AdafruitGFX drawable
    GraphicsDeviceRenderer _renderer;                       // Graphics renderer
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

    virtual Pair<uint16_t,uint16_t> getScreenSize(bool rotated = true) const override { return isLandscape(rotated) ? make_pair(max(TFT_GFX_WIDTH,TFT_GFX_HEIGHT), min(TFT_GFX_WIDTH,TFT_GFX_HEIGHT))
                                                                                                                    : make_pair(min(TFT_GFX_WIDTH,TFT_GFX_HEIGHT), max(TFT_GFX_WIDTH,TFT_GFX_HEIGHT)); }
    virtual bool isLandscape(bool rotated = true) const override { return rotated ? (TFT_GFX_WIDTH >= TFT_GFX_HEIGHT ? !(_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3)
                                                                                                                      : (_rotation == Helio_DisplayRotation_R1 || _rotation == Helio_DisplayRotation_R3))
                                                                                  : TFT_GFX_WIDTH >= TFT_GFX_HEIGHT; }
    // Determines if screen is in round screen mode.
    inline bool isRound() const
        #ifdef GC9A01_DRIVER
            { return TFT_GFX_WIDTH == TFT_GFX_HEIGHT; }
        #else
            { return false; }
        #endif
    virtual uint8_t getScreenBits() const override { return 24; }

    virtual BaseMenuRenderer *getBaseRenderer() override { return &_renderer; }
    virtual GraphicsDeviceRenderer *getGraphicsRenderer() override { return &_renderer; }

    // GFX accessor
    inline TFT_eSPI &getGfx() { return _gfx; }
    // Drawable accessor
    inline TfteSpiDrawable &getDrawable() { return _drawable; }

protected:
    const Helio_ST77XXKind _kind;                           // ST7735 kind enum
    TFT_eSPI _gfx;                                          // GFX instance
    TfteSpiDrawable _drawable;                              // TFTeSPI drawable
    GraphicsDeviceRenderer _renderer;                       // Graphics renderer
};

#endif // /ifndef HelioDisplayDrivers_H
#endif

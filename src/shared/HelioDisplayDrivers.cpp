/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Display Drivers
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI
#include "BaseRenderers.h"
#include "graphics/BaseGraphicalRenderer.h"
#include "IoAbstractionWire.h"
#include "DfRobotInputAbstraction.h"

HelioDisplayDriver::HelioDisplayDriver(Helio_DisplayRotation displayRotation)
    : _rotation(displayRotation), _displayTheme(Helio_DisplayTheme_Undefined)
{ ; }

void HelioDisplayDriver::setupRendering(uint8_t titleMode, Helio_DisplayTheme displayTheme, const void *itemFont, const void *titleFont, bool analogSlider, bool editingIcons, bool utf8Fonts)
{
    auto graphicsRenderer = getGraphicsRenderer();
    if (graphicsRenderer) {
        if (getController()->getControlInputMode() >= Helio_ControlInputMode_ResistiveTouch &&
            getController()->getControlInputMode() < Helio_ControlInputMode_RemoteControl) {
            graphicsRenderer->setHasTouchInterface(true);
        }
        graphicsRenderer->setTitleMode((BaseGraphicalRenderer::TitleMode)titleMode);
        graphicsRenderer->setUseSliderForAnalog(analogSlider);
        if (utf8Fonts) { graphicsRenderer->enableTcUnicode(); }

        if (_displayTheme != displayTheme) {
            switch ((_displayTheme = displayTheme)) {
                case Helio_DisplayTheme_CoolBlue_ML:
                    installCoolBlueModernTheme(*graphicsRenderer, MenuFontDef(itemFont, HELIO_UI_MENU_ITEM_MAG_LEVEL), MenuFontDef(titleFont, HELIO_UI_MENU_TITLE_MAG_LEVEL), editingIcons);
                    break;
                case Helio_DisplayTheme_CoolBlue_SM:
                    installCoolBlueTraditionalTheme(*graphicsRenderer, MenuFontDef(itemFont, HELIO_UI_MENU_ITEM_MAG_LEVEL), MenuFontDef(titleFont, HELIO_UI_MENU_TITLE_MAG_LEVEL), editingIcons);
                    break;
                case Helio_DisplayTheme_DarkMode_ML:
                    installDarkModeModernTheme(*graphicsRenderer, MenuFontDef(itemFont, HELIO_UI_MENU_ITEM_MAG_LEVEL), MenuFontDef(titleFont, HELIO_UI_MENU_TITLE_MAG_LEVEL), editingIcons);
                    break;
                case Helio_DisplayTheme_DarkMode_SM:
                    installDarkModeTraditionalTheme(*graphicsRenderer, MenuFontDef(itemFont, HELIO_UI_MENU_ITEM_MAG_LEVEL), MenuFontDef(titleFont, HELIO_UI_MENU_TITLE_MAG_LEVEL), editingIcons);
                    break;
                case Helio_DisplayTheme_MonoOLED:
                    installMonoBorderedTheme(*graphicsRenderer, MenuFontDef(itemFont, HELIO_UI_MENU_ITEM_MAG_LEVEL), MenuFontDef(titleFont, HELIO_UI_MENU_TITLE_MAG_LEVEL), editingIcons);
                    break;
                case Helio_DisplayTheme_MonoOLED_Inv:
                    installMonoInverseTitleTheme(*graphicsRenderer, MenuFontDef(itemFont, HELIO_UI_MENU_ITEM_MAG_LEVEL), MenuFontDef(titleFont, HELIO_UI_MENU_TITLE_MAG_LEVEL), editingIcons);
                    break;
            }
        }
    }
}


HelioDisplayLiquidCrystal::HelioDisplayLiquidCrystal(Helio_DisplayOutputMode displayMode, I2CDeviceSetup displaySetup, Helio_BacklightMode ledMode)
    : _screenSize{displayMode < Helio_DisplayOutputMode_LCD20x4_EN ? 16 : 20, displayMode < Helio_DisplayOutputMode_LCD20x4_EN ? 2 : 4},
      _lcd(displayMode == Helio_DisplayOutputMode_LCD16x2_EN || displayMode == Helio_DisplayOutputMode_LCD20x4_EN ? 2 : 0, 1,
           displayMode == Helio_DisplayOutputMode_LCD16x2_EN || displayMode == Helio_DisplayOutputMode_LCD20x4_EN ? 0 : 2, 4, 5, 6, 7,
           ledMode == Helio_BacklightMode_Normal ? LiquidCrystal::BACKLIGHT_NORMAL : ledMode == Helio_BacklightMode_Inverted ? LiquidCrystal::BACKLIGHT_INVERTED : LiquidCrystal::BACKLIGHT_PWM,
           ioFrom8574(HELIO_UI_I2C_LCD_BASEADDR | displaySetup.address, 0xff, displaySetup.wire, false)),
      _renderer(_lcd, _screenSize[0], _screenSize[1], getController()->getSystemNameChars())
{
    _lcd.configureBacklightPin(3);
}

HelioDisplayLiquidCrystal::HelioDisplayLiquidCrystal(bool isDFRobotShield_unused, I2CDeviceSetup displaySetup, Helio_BacklightMode ledMode)
    : _screenSize{16, 2},
      _lcd(8, 9, 4, 5, 6, 7,
           ledMode == Helio_BacklightMode_Normal ? LiquidCrystal::BACKLIGHT_NORMAL : ledMode == Helio_BacklightMode_Inverted ? LiquidCrystal::BACKLIGHT_INVERTED : LiquidCrystal::BACKLIGHT_PWM,
           ioFrom8574(HELIO_UI_I2C_LCD_BASEADDR | displaySetup.address, 0xff, displaySetup.wire, false)),
      _renderer(_lcd, _screenSize[0], _screenSize[1], getController()->getSystemNameChars())
{
    _lcd.configureBacklightPin(10);
}

void HelioDisplayLiquidCrystal::initBaseUIFromDefaults()
{
    getBaseUI()->init(HELIO_UI_UPDATE_SPEED, Helio_DisplayTheme_Undefined);
}

void HelioDisplayLiquidCrystal::begin()
{
    _lcd.begin(_screenSize[0], _screenSize[1]);
    _renderer.setTitleRequired(false);
}

HelioOverview *HelioDisplayLiquidCrystal::allocateOverview(const void *clockFont, const void *detailFont)
{
    return new HelioOverviewLCD(this); // todo: font handling (# based)
}


HelioDisplayU8g2OLED::HelioDisplayU8g2OLED(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, U8G2 *gfx)
    : HelioDisplayDriver(displayRotation),
      _screenSize{gfx->getDisplayWidth(), gfx->getDisplayHeight()}, _gfx(gfx), _drawable(nullptr), _renderer(nullptr)
{
    HELIO_SOFT_ASSERT(_gfx, SFP(HStr_Err_AllocationFailure));
    if (_gfx) {
        if (displaySetup.cfgType == DeviceSetup::I2CSetup) {
            _gfx->setI2CAddress(HELIO_UI_I2C_OLED_BASEADDR | displaySetup.cfgAs.i2c.address);
        }
        #ifdef HELIO_UI_ENABLE_STCHROMA_LDTC
            _drawable = new StChromaArtDrawable();
        #else
            if (displaySetup.cfgType == DeviceSetup::I2CSetup) {
                _drawable = new U8g2Drawable(_gfx, displaySetup.cfgAs.i2c.wire);
            } else {
                _drawable = new U8g2Drawable(_gfx);
            }
        #endif
        HELIO_SOFT_ASSERT(_drawable, SFP(HStr_Err_AllocationFailure));

        if (_drawable) {
            _renderer = new GraphicsDeviceRenderer(HELIO_UI_RENDERER_BUFFERSIZE, getController()->getSystemNameChars(), _drawable);
            HELIO_SOFT_ASSERT(_renderer, SFP(HStr_Err_AllocationFailure));
        }
    }
}

HelioDisplayU8g2OLED::~HelioDisplayU8g2OLED()
{
    if (_renderer) { delete _renderer; }
    if (_drawable) { delete _drawable; }
    if (_gfx) { delete _gfx; }
}

void HelioDisplayU8g2OLED::initBaseUIFromDefaults()
{
    getBaseUI()->init(HELIO_UI_UPDATE_SPEED, definedThemeElse(getDisplayTheme(), Helio_DisplayTheme_MonoOLED), BaseGraphicalRenderer::TITLE_FIRST_ROW);
}

void HelioDisplayU8g2OLED::begin()
{
    if (_gfx) { _gfx->begin(); }
}

HelioOverview *HelioDisplayU8g2OLED::allocateOverview(const void *clockFont, const void *detailFont)
{
    return new HelioOverviewOLED(this); // todo: font handling (# based)
}


HelioDisplayAdafruitGFX<Adafruit_ST7735>::HelioDisplayAdafruitGFX(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, Helio_ST77XXKind st77Kind, pintype_t dcPin, pintype_t resetPin)
    : HelioDisplayDriver(displayRotation), _kind(st77Kind),
      #ifndef ESP8266
          _gfx(displaySetup.spi, intForPin(dcPin), intForPin(displaySetup.cs), intForPin(resetPin)),
      #else
          _gfx(intForPin(displaySetup.cs), intForPin(dcPin), intForPin(resetPin)),
      #endif
      _screenSize{_gfx.width(),_gfx.height()}, // incorrect until after begin
      _drawable(&_gfx, 0),
      _renderer(HELIO_UI_RENDERER_BUFFERSIZE, getController()->getSystemNameChars(), &_drawable)
{
    HELIO_SOFT_ASSERT(_kind != Helio_ST77XXKind_Undefined, SFP(HStr_Err_InvalidParameter));
    #ifdef ESP8266
        HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    #endif
}

void HelioDisplayAdafruitGFX<Adafruit_ST7735>::initBaseUIFromDefaults()
{
    getBaseUI()->init(HELIO_UI_UPDATE_SPEED, definedThemeElse(getDisplayTheme(), JOIN3(Helio_DisplayTheme, HELIO_UI_GFX_DISP_THEME_BASE, HELIO_UI_GFX_DISP_THEME_SMLMED)), BaseGraphicalRenderer::TITLE_ALWAYS, HELIO_UI_GFX_VARS_USES_SLIDER);
}

void HelioDisplayAdafruitGFX<Adafruit_ST7735>::begin()
{
    if (_kind == Helio_ST7735Tag_B) {
        _gfx.initB();
    } else {
        _gfx.initR((uint8_t)_kind);
    }
    _screenSize[0] = _gfx.width();
    _screenSize[1] = _gfx.height();
    _gfx.setRotation((uint8_t)_rotation);
}

HelioOverview *HelioDisplayAdafruitGFX<Adafruit_ST7735>::allocateOverview(const void *clockFont, const void *detailFont)
{
    return new HelioOverviewGFX<Adafruit_ST7735>(this, clockFont, detailFont);
}


HelioDisplayAdafruitGFX<Adafruit_ST7789>::HelioDisplayAdafruitGFX(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, Helio_ST77XXKind st77Kind, pintype_t dcPin, pintype_t resetPin)
    : HelioDisplayDriver(displayRotation), _kind(st77Kind),
      #ifndef ESP8266
          _gfx(displaySetup.spi, intForPin(dcPin), intForPin(displaySetup.cs), intForPin(resetPin)),
      #else
          _gfx(intForPin(displaySetup.cs), intForPin(dcPin), intForPin(resetPin)),
      #endif
      _screenSize{_gfx.width(),_gfx.height()}, // incorrect until after begin
      _drawable(&_gfx, 0),
      _renderer(HELIO_UI_RENDERER_BUFFERSIZE, getController()->getSystemNameChars(), &_drawable)
{
    HELIO_SOFT_ASSERT(_kind != Helio_ST77XXKind_Undefined, SFP(HStr_Err_InvalidParameter));
    #ifdef ESP8266
        HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    #endif
}

void HelioDisplayAdafruitGFX<Adafruit_ST7789>::initBaseUIFromDefaults()
{
    getBaseUI()->init(HELIO_UI_UPDATE_SPEED, definedThemeElse(getDisplayTheme(), JOIN3(Helio_DisplayTheme, HELIO_UI_GFX_DISP_THEME_BASE, HELIO_UI_GFX_DISP_THEME_SMLMED)), BaseGraphicalRenderer::TITLE_ALWAYS, HELIO_UI_GFX_VARS_USES_SLIDER);
}

void HelioDisplayAdafruitGFX<Adafruit_ST7789>::begin()
{
    switch (_kind) {
        case Helio_ST7789Res_128x128:
            _gfx.init(128, 128);
            break;
        case Helio_ST7789Res_135x240:
            _gfx.init(135, 240);
            break;
        case Helio_ST7789Res_170x320:
            _gfx.init(170, 320);
            break;
        case Helio_ST7789Res_172x320:
            _gfx.init(172, 320);
            break;
        case Helio_ST7789Res_240x240:
            _gfx.init(240, 240);
            break;
        case Helio_ST7789Res_240x280:
            _gfx.init(240, 280);
            break;
        case Helio_ST7789Res_240x320:
            _gfx.init(240, 320);
            break;
        default:
            _gfx.init(TFT_GFX_WIDTH, TFT_GFX_HEIGHT);
            break;
    }
    _screenSize[0] = _gfx.width();
    _screenSize[1] = _gfx.height();
    _gfx.setRotation((uint8_t)_rotation);
}

HelioOverview *HelioDisplayAdafruitGFX<Adafruit_ST7789>::allocateOverview(const void *clockFont, const void *detailFont)
{
    return new HelioOverviewGFX<Adafruit_ST7789>(this, clockFont, detailFont);
}


HelioDisplayAdafruitGFX<Adafruit_ILI9341>::HelioDisplayAdafruitGFX(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
    : HelioDisplayDriver(displayRotation),
      #ifndef ESP8266
          _gfx(displaySetup.spi, intForPin(dcPin), intForPin(displaySetup.cs), intForPin(resetPin)),
      #else
          _gfx(intForPin(displaySetup.cs), intForPin(dcPin), intForPin(resetPin)),
      #endif
      _screenSize{_gfx.width(),_gfx.height()}, // incorrect until after begin
      _drawable(&_gfx, 0),
      _renderer(HELIO_UI_RENDERER_BUFFERSIZE, getController()->getSystemNameChars(), &_drawable)
{
    #ifdef ESP8266
        HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    #endif
}

void HelioDisplayAdafruitGFX<Adafruit_ILI9341>::initBaseUIFromDefaults()
{
    getBaseUI()->init(HELIO_UI_UPDATE_SPEED, definedThemeElse(getDisplayTheme(), JOIN3(Helio_DisplayTheme, HELIO_UI_GFX_DISP_THEME_BASE, HELIO_UI_GFX_DISP_THEME_SMLMED)), BaseGraphicalRenderer::TITLE_ALWAYS, HELIO_UI_GFX_VARS_USES_SLIDER);
}

void HelioDisplayAdafruitGFX<Adafruit_ILI9341>::begin()
{
    _gfx.begin(getController()->getDisplaySetup().cfgAs.spi.speed);
    _screenSize[0] = _gfx.width();
    _screenSize[1] = _gfx.height();
    _gfx.setRotation((uint8_t)_rotation);
}

HelioOverview *HelioDisplayAdafruitGFX<Adafruit_ILI9341>::allocateOverview(const void *clockFont, const void *detailFont)
{
    return new HelioOverviewGFX<Adafruit_ILI9341>(this, clockFont, detailFont);
}


HelioDisplayTFTeSPI::HelioDisplayTFTeSPI(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, Helio_ST77XXKind st77Kind)
    : HelioDisplayDriver(displayRotation), _kind(st77Kind),
      _gfx(TFT_GFX_WIDTH, TFT_GFX_HEIGHT),
      _drawable(&_gfx, 0),
      _renderer(HELIO_UI_RENDERER_BUFFERSIZE, getController()->getSystemNameChars(), &_drawable)
{ ; }

void HelioDisplayTFTeSPI::initBaseUIFromDefaults()
{
    getBaseUI()->init(HELIO_UI_UPDATE_SPEED, definedThemeElse(getDisplayTheme(), JOIN3(Helio_DisplayTheme, HELIO_UI_GFX_DISP_THEME_BASE, HELIO_UI_GFX_DISP_THEME_MEDLRG)), BaseGraphicalRenderer::TITLE_ALWAYS, HELIO_UI_GFX_VARS_USES_SLIDER);
}

void HelioDisplayTFTeSPI::begin()
{
    if (_kind == Helio_ST7735Tag_B || _kind >= Helio_ST7789Res_Start) {
        _gfx.begin();
    } else {
        _gfx.begin((uint8_t)_kind);
    }
    _gfx.setRotation((uint8_t)_rotation);
    _renderer.setDisplayDimensions(getScreenSize().first, getScreenSize().second);
}

HelioOverview *HelioDisplayTFTeSPI::allocateOverview(const void *clockFont, const void *detailFont)
{
    return new HelioOverviewTFT(this); // todo: font handling (# based)
}

#endif

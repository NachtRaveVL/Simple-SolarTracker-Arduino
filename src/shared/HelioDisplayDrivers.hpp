/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Display Drivers
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI

static inline const u8g2_cb_t *dispRotToU8g2Rot(Helio_DisplayRotation displayRotation)
{
    switch (displayRotation) {
        case Helio_DisplayRotation_R0: return U8G2_R0;
        case Helio_DisplayRotation_R1: return U8G2_R1;
        case Helio_DisplayRotation_R2: return U8G2_R2;
        case Helio_DisplayRotation_R3: return U8G2_R3;
        case Helio_DisplayRotation_HorzMirror: return U8G2_MIRROR;
        case Helio_DisplayRotation_VertMirror: return U8G2_MIRROR_VERTICAL;
        default: return U8G2_R0;
    }
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X32_NONAME_F_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X32_NONAME_F_2ND_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305Wire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X32_NONAME_F_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305Wire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || ((bool)HELIO_USE_WIRE1 && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1)), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X32_NONAME_F_2ND_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305x32AdaSPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X32_ADAFRUIT_F_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305x32AdaSPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X32_ADAFRUIT_F_2ND_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305x32AdaWire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}
inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305x32AdaWire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || ((bool)HELIO_USE_WIRE1 && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1)), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X32_ADAFRUIT_F_2ND_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305x64AdaSPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X64_ADAFRUIT_F_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305x64AdaSPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X64_ADAFRUIT_F_2ND_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305x64AdaWire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X64_ADAFRUIT_F_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1305x64AdaWire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || ((bool)HELIO_USE_WIRE1 && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1)), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1305_128X64_ADAFRUIT_F_2ND_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1306SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1306SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1306_128X64_NONAME_F_2ND_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1306Wire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1306Wire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || ((bool)HELIO_USE_WIRE1 && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1)), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSH1106SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSH1106SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup && (!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SH1106_128X64_NONAME_F_2ND_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSH1106Wire(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SH1106_128X64_NONAME_F_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSH1106Wire1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup && (!(bool)HELIO_USE_WIRE || ((bool)HELIO_USE_WIRE1 && displaySetup.cfgAs.i2c.wire == HELIO_USE_WIRE1)), SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SH1106_128X64_NONAME_F_2ND_HW_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateCustomOLEDI2C(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::I2CSetup, SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new HELIO_UI_CUSTOM_OLED_I2C(dispRotToU8g2Rot(displayRotation), resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateCustomOLEDSPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(displaySetup.cfgType == DeviceSetup::SPISetup, SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new HELIO_UI_CUSTOM_OLED_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1607SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1607_200X200_F_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateSSD1607SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_SSD1607_200X200_F_2ND_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateIL3820SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_IL3820_296X128_F_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateIL3820SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_IL3820_296X128_F_2ND_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateIL3820V2SPI(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_IL3820_V2_296X128_F_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}

inline HelioDisplayU8g2OLED *HelioDisplayU8g2OLED::allocateIL3820V2SPI1(DeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
{
    HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.cfgAs.spi.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    return new HelioDisplayU8g2OLED(displaySetup, displayRotation, new U8G2_IL3820_V2_296X128_F_2ND_4W_HW_SPI(dispRotToU8g2Rot(displayRotation), displaySetup.cfgAs.spi.cs, dcPin, resetPin));
}


template <class T>
HelioDisplayAdafruitGFX<T>::HelioDisplayAdafruitGFX(SPIDeviceSetup displaySetup, Helio_DisplayRotation displayRotation, pintype_t dcPin, pintype_t resetPin)
    : HelioDisplayDriver(displayRotation, _gfx.width(),_gfx.height()), // possibly incorrect until after begin
    #ifndef ESP8266
          _gfx(displaySetup.spi, intForPin(dcPin), intForPin(displaySetup.cs), intForPin(resetPin)),
      #else
          _gfx(intForPin(displaySetup.cs), intForPin(dcPin), intForPin(resetPin)),
      #endif
      _drawable(&_gfx, 0),
      _renderer(HELIO_UI_RENDERER_BUFFERSIZE, getController()->getSystemNameChars(), &_drawable)
{
    #ifdef ESP8266
        HELIO_SOFT_ASSERT(!(bool)HELIO_USE_SPI || displaySetup.spi == HELIO_USE_SPI, SFP(HStr_Err_InvalidParameter));
    #endif
}

template <class T>
void HelioDisplayAdafruitGFX<T>::initBaseUIFromDefaults()
{
    getBaseUI()->init(HELIO_UI_UPDATE_SPEED, definedThemeElse(getDisplayTheme(), JOIN3(Helio_DisplayTheme, HELIO_UI_GFX_DISP_THEME_BASE, HELIO_UI_GFX_DISP_THEME_SMLMED)), HELIO_UI_GFX_VARS_USES_SLIDER);
}

template <class T>
void HelioDisplayAdafruitGFX<T>::begin()
{
    _gfx.begin();
    _screenSize[0] = _gfx.width();
    _screenSize[1] = _gfx.height();
    _gfx.setRotation((uint8_t)_rotation);
}

template <class T>
HelioOverview *HelioDisplayAdafruitGFX<T>::allocateOverview(const void *clockFont, const void *detailFont)
{
    return new HelioOverviewGFX<T>(this, clockFont, detailFont);
}

#endif

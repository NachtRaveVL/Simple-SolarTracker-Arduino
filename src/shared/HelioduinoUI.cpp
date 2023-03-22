/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Base UI
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioduinoBaseUI::HelioduinoBaseUI(UIControlSetup uiControlSetup, UIDisplaySetup uiDisplaySetup, bool isActiveLowIO, bool allowInterruptableIO, bool enableTcUnicodeFonts)
{ ; }

HelioduinoBaseUI::~HelioduinoBaseUI()
{ ; }

void HelioduinoBaseUI::init(uint8_t updatesPerSec, Helio_DisplayTheme displayTheme, bool analogSlider, bool editingIcons)
{ ; }

HelioUIData *HelioduinoBaseUI::init(HelioUIData *uiData)
{ return uiData; }

bool HelioduinoBaseUI::begin()
{ return false; }

void HelioduinoBaseUI::setNeedsRedraw()
{ ; }

#endif

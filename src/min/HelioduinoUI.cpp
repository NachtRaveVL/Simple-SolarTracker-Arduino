/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Minimal/RO UI
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioduinoMinUI::HelioduinoMinUI(UIControlSetup uiControlSetup, UIDisplaySetup uiDisplaySetup, bool isActiveLowIO, bool allowInterruptableIO, bool enableTcUnicodeFonts)
    : HelioduinoBaseUI(uiControlSetup, uiDisplaySetup, isActiveLowIO, allowInterruptableIO, enableTcUnicodeFonts)
{ ; }

HelioduinoMinUI::~HelioduinoMinUI()
{ ; }

#endif

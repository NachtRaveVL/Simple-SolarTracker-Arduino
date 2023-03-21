/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Full/RW UI
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioduinoFullUI::HelioduinoFullUI(UIControlSetup uiControlSetup, UIDisplaySetup uiDisplaySetup, bool isActiveLowIO, bool allowInterruptableIO, bool enableTcUnicodeFonts)
    : HelioduinoBaseUI(uiControlSetup, uiDisplaySetup, isActiveLowIO, allowInterruptableIO, enableTcUnicodeFonts)
{ ; }

HelioduinoFullUI::~HelioduinoFullUI()
{ ; }

#endif

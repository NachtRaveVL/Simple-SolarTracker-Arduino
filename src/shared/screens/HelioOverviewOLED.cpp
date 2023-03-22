/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino U8g2 OLED Overview Screen
*/

#include "../HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioOverviewOLED::HelioOverviewOLED(HelioDisplayU8g2OLED *display)
    : HelioOverview(display), _gfx(display->getGfx()), _drawable(display->getDrawable())
{ ; }

HelioOverviewOLED::~HelioOverviewOLED()
{ ; }

void HelioOverviewOLED::renderOverview(bool isLandscape, Pair<uint16_t, uint16_t> screenSize)
{
    // todo
}

#endif

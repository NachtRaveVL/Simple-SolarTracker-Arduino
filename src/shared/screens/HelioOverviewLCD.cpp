/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino LCD Overview Screen
*/

#include "../HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioOverviewLCD::HelioOverviewLCD(HelioDisplayLiquidCrystal *display)
    : HelioOverview(display), _lcd(display->getLCD())
{ ; }

HelioOverviewLCD::~HelioOverviewLCD()
{ ; }

void HelioOverviewLCD::renderOverview(bool isLandscape, Pair<uint16_t, uint16_t> screenSize)
{
    // todo
}

#endif

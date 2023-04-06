/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino TFT_eSPI Overview Screen
*/

#include "../HelioduinoUI.h"
#ifdef HELIO_USE_GUI

extern float skyEaseInOut(float x);
extern void randomStarColor(uint8_t* r, uint8_t* g, uint8_t* b);

HelioOverviewTFT::HelioOverviewTFT(HelioDisplayTFTeSPI *display, const void *clockFont, const void *detailFont)
    : HelioOverview(display), _gfx(display->getGfx()), _drawable(display->getDrawable()), _clockFont(clockFont), _detailFont(detailFont)
{ ; }

HelioOverviewTFT::~HelioOverviewTFT()
{ ; }

void HelioOverviewTFT::renderOverview(bool isLandscape, Pair<uint16_t, uint16_t> screenSize)
{
    // todo
}

#endif

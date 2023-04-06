/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino TFT_eSPI Overview Screen
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioOverviewTFT_H
#define HelioOverviewTFT_H

class HelioOverviewTFT;

#include "../HelioduinoUI.h"

// TFT_eSPI Overview Screen
// Overview screen built for TFT_eSPI displays.
class HelioOverviewTFT : public HelioOverview {
public:
    HelioOverviewTFT(HelioDisplayTFTeSPI *display, const void *clockFont, const void *detailFont);
    virtual ~HelioOverviewTFT();

    virtual void renderOverview(bool isLandscape, Pair<uint16_t, uint16_t> screenSize) override;

protected:
    TFT_eSPI &_gfx;                                         // Graphics (strong)
    TfteSpiDrawable &_drawable;                             // Drawable (strong)
};

#endif // /ifndef HelioOverviewTFT_H
#endif

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Overview Screens
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioOverviews_H
#define HelioOverviews_H

class HelioOverview;
class HelioOverviewLCD;
class HelioOverviewOLED;
template<class T> class HelioOverviewGFX;
class HelioOverviewTFT;

#include "HelioduinoUI.h"

// Overview Screen Base
// Overview screen class that manages the default at-a-glance system overview.
// Meant to be able to be deleted on a moments notice to transition back into menu.
class HelioOverview {
public:
    HelioOverview(HelioDisplayDriver *display);
    virtual ~HelioOverview();

    virtual void renderOverview(bool isLandscape, Pair<uint16_t, uint16_t> screenSize) = 0;

    inline void setNeedsFullRedraw() { _needsFullRedraw = true; }

protected:
    HelioDisplayDriver *_display;                           // Display (strong)
    bool _needsFullRedraw;                                  // Needs full redraw flag
};

#include "screens/HelioOverviewGFX.h"
#include "screens/HelioOverviewLCD.h"
#include "screens/HelioOverviewOLED.h"
#include "screens/HelioOverviewTFT.h"

#endif // /ifndef HelioOverviews_H
#endif

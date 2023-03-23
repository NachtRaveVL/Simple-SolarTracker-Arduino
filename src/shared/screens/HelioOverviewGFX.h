/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino AdafruitGFX Overview Screen
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioOverviewGFX_H
#define HelioOverviewGFX_H

template<class T> class HelioOverviewGFX;

#include "../HelioduinoUI.h"

// AdafruitGFX Overview Screen
// Overview screen built for AdafruitGFX displays.
template<class T>
class HelioOverviewGFX : public HelioOverview {
public:
    HelioOverviewGFX(HelioDisplayAdafruitGFX<T> *display, const void *clockFont = nullptr, const void *detailFont = nullptr);
    virtual ~HelioOverviewGFX();

    virtual void renderOverview(bool isLandscape, Pair<uint16_t, uint16_t> screenSize) override;

protected:
    T &_gfx;                                                // Graphics (strong)
    AdafruitDrawable<T> &_drawable;                         // Drawable (strong)
    const void *_clockFont;                                 // Overview clock font (strong)
    const void *_detailFont;                                // Overview detail font (strong)

    uint8_t _skyBlue, _skyRed;                              // Sky color
    Map<uint16_t,Pair<uint16_t,uint16_t>,HELIO_UI_STARFIELD_MAXSIZE> _stars; // Starfield
    int _timeMag, _dateMag;                                 // Time/date mag level
    DateTime _lastTime;                                     // Last time (local)
    uint16_t _timeHeight, _dateHeight;                      // Pixel height

    void drawBackground(Coord pt, Coord sz, Pair<uint16_t, uint16_t> &screenSize);
};

#endif // /ifndef HelioOverviewGFX_H
#endif

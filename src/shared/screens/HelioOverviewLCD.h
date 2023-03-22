/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino LCD Overview Screen
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioOverviewLCD_H
#define HelioOverviewLCD_H

class HelioOverviewLCD;

#include "../HelioduinoUI.h"

// LCD Overview Screen
// Overview screen built for LCD displays.
class HelioOverviewLCD : public HelioOverview {
public:
    HelioOverviewLCD(HelioDisplayLiquidCrystal *display);
    virtual ~HelioOverviewLCD();

    virtual void renderOverview(bool isLandscape, Pair<uint16_t, uint16_t> screenSize) override;

protected:
    LiquidCrystal &_lcd;                                    // LCD (strong)
};

#endif // /ifndef HelioOverviewLCD_H
#endif

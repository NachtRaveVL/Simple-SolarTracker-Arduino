/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Overview Screens
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioOverview::HelioOverview(HelioDisplayDriver *display)
    : _display(display), _needsFullRedraw(true)
{ ; }

HelioOverview::~HelioOverview()
{ ; }

#endif

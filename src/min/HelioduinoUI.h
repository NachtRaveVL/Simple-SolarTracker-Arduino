/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Minimal/RO UI
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioUI_H
#define HelioUI_H

class HelioduinoMinUI;
typedef HelioduinoMinUI HydruinoUI;

#include "..\shared\HelioduinoUI.h"

class HelioduinoMinUI : public HelioduinoBaseUI {
public:
};

#endif // /ifndef HelioUI_H
#endif

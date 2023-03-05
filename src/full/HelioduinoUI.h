/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Full/RW UI
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioUI_H
#define HelioUI_H

class HelioduinoFullUI;
typedef HelioduinoFullUI HelioduinoUI;

#include "..\shared\HelioduinoUI.h"

class HelioduinoFullUI : public HelioduinoBaseUI {
public:
};

#endif // /ifndef HelioUI_H
#endif

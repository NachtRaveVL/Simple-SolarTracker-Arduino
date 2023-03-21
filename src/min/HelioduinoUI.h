/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Minimal/RO UI
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioUI_H
#define HelioUI_H

class HelioduinoMinUI;
typedef HelioduinoMinUI HelioduinoUI;

#include "..\shared\HelioduinoUI.h"

class HelioduinoMinUI : public HelioduinoBaseUI {
public:
    HelioduinoMinUI(UIControlSetup uiControlSetup = UIControlSetup(),       // UI control input setup
                    UIDisplaySetup uiDisplaySetup = UIDisplaySetup(),       // UI display output setup 
                    bool isActiveLowIO = true,                              // Logic level usage for control & display IO pins
                    bool allowInterruptableIO = true,                       // Allows interruptable pins to interrupt, else forces polling
                    bool enableTcUnicodeFonts = true);                      // Enables tcUnicode UTF8 fonts usage instead of library fonts
    virtual ~HelioduinoMinUI();
};

#endif // /ifndef HelioUI_H
#endif

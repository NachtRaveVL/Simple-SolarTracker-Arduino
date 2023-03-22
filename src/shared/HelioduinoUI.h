/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Base UI
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioBaseUI_H
#define HelioBaseUI_H

#include "HelioUIDefines.h"
#include "HelioUIInlines.hh"
#include "HelioUIData.h"
#include "HelioUIStrings.h"

// tcMenu Plugin Adaptations
// #include "tcMenu_Display_AdaFruitGfx.h"
// #include "tcMenu_Display_LiquidCrystal.h"
// #include "tcMenu_Display_TfteSpi.h"
// #include "tcMenu_Display_U8g2.h"
// #include "tcMenu_Input_AdaTouchDriver.h"
// #include "tcMenu_Input_ESP32TouchKeysAbstraction.h"
// #include "tcMenu_Remote_EthernetTransport.h"
// #include "tcMenu_Remote_SerialTransport.h"
// #include "tcMenu_Remote_SimhubConnector.h"
// #include "tcMenu_Remote_WiFiTransport.h"
// #include "tcMenu_Theme_CoolBlueModern.h"
// #include "tcMenu_Theme_CoolBlueTraditional.h"
// #include "tcMenu_Theme_DarkModeModern.h"
// #include "tcMenu_Theme_DarkModeTraditional.h"
// #include "tcMenu_Theme_MonoBordered.h"
// #include "tcMenu_Theme_MonoInverse.h"

// #include "HelioDisplayDrivers.h"
// #include "HelioInputDrivers.h"
// #include "HelioRemoteControls.h"
// #include "HelioMenus.h"
// #include "HelioOverviews.h"

// Base UI
// The base class that manages interaction with the tcMenu UI system.
class HelioduinoBaseUI : public HelioUIInterface { //, public CustomDrawing {
public:
    HelioduinoBaseUI(UIControlSetup uiControlSetup = UIControlSetup(),      // UI control input setup
                     UIDisplaySetup uiDisplaySetup = UIDisplaySetup(),      // UI display output setup 
                     bool isActiveLowIO = true,                             // Logic level usage for control & display IO pins
                     bool allowInterruptableIO = true,                      // Allows interruptable pins to interrupt, else forces polling
                     bool enableTcUnicodeFonts = true);                     // Enables tcUnicode UTF8 fonts usage instead of library fonts
    virtual ~HelioduinoBaseUI();

    void init(uint8_t updatesPerSec,                                        // Updates per second (1 to 10)
              Helio_DisplayTheme displayTheme,                              // Display theme to apply
              uint8_t titleMode = 0,                                        // Title mode
              bool analogSlider = false,                                    // Slider usage for analog items
              bool editingIcons = false);                                   // Editing icons usage

    virtual HelioUIData *init(HelioUIData *uiData = nullptr) override;      // Standard initializer
    virtual bool begin() override;                                          // Begins UI

    virtual void setNeedsRedraw() override;

    inline HelioUIData *getUIData() { return nullptr; }
};

// #include "tcMenu_Display_AdaFruitGfx.hpp"
// #include "HelioDisplayDrivers.hpp"
// #include "HelioOverviews.hpp"

#endif // /ifndef HelioBaseUI_H
#endif

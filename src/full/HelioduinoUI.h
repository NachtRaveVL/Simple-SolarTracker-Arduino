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

#include "../shared/HelioduinoUI.h"

class HelioduinoFullUI : public HelioduinoBaseUI {
public:
    HelioduinoFullUI(UIControlSetup uiControlSetup = UIControlSetup(),      // UI control input setup
                     UIDisplaySetup uiDisplaySetup = UIDisplaySetup(),      // UI display output setup 
                     bool isActiveLowIO = true,                             // Logic level usage for control & display IO pins
                     bool allowInterruptableIO = true,                      // Allows interruptable pins to interrupt, else forces polling
                     bool enableTcUnicodeFonts = true);                     // Enables tcUnicode UTF8 fonts usage instead of library fonts
    virtual ~HelioduinoFullUI();

    void addRemote(Helio_RemoteControl rcType,                              // Type of remote control
                   UARTDeviceSetup rcSetup = UARTDeviceSetup(),             // Remote control serial setup (if serial based), else ignored
                   uint16_t rcServerPort = HELIO_UI_REMOTESERVER_PORT);     // Remote control server listening port (if networking based), else ignored

    virtual bool isFullUI() override;

protected:
};

#endif // /ifndef HelioUI_H
#endif

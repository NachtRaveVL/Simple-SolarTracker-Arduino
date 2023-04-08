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
    HelioduinoFullUI(String deviceUUID,                                     // Device UUID hex string for remote controllability
                     UIControlSetup uiControlSetup = UIControlSetup(),      // UI control input setup, from controller initialization
                     UIDisplaySetup uiDisplaySetup = UIDisplaySetup(),      // UI display output setup, from controller initialization
                     bool isActiveLowIO = true,                             // Signaling logic level usage for I/O control/display devices
                     bool allowInterruptableIO = true,                      // Allows interruptable pins to interrupt, else forces polling
                     bool enableTcUnicodeFonts = false,                     // Enables tcUnicode fonts usage over GFXfont (Adafruit) fonts
                     bool enableBufferedVRAM = false);                      // Enables sprite-sized buffered video RAM for smooth animations
    virtual ~HelioduinoFullUI();

    void addRemote(Helio_RemoteControl rcType,                              // Type of remote control
                   UARTDeviceSetup rcSetup = UARTDeviceSetup(),             // Remote control serial setup (if serial based), else ignored
                   uint16_t rcServerPort = HELIO_UI_REMOTESERVER_PORT);     // Remote control server listening port (if networking based), else ignored

    virtual bool isFullUI() override;

protected:
};

#endif // /ifndef HelioUI_H
#endif

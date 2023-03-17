/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino UI Data
*/

#ifndef HelioUIData_H
#define HelioUIData_H

struct HelioUIData;

#include "Helioduino.h"
#include "HelioUIDefines.h"

// UI Serialization Data
// id: HUID. Helioduino UI data.
struct HelioUIData : public HelioData {
    uint8_t updatesPerSec;                                  // Updates per second (1-10, default: HELIO_UI_UPDATE_SPEED)
    Helio_DisplayTheme displayTheme;                        // Display theme (if supported)
    float joystickCalib[3];                                 // Joystick calibration ({midX,midY,zeroTol}, default: {0.5,0.5,0.05})

    HelioUIData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioUIData_H

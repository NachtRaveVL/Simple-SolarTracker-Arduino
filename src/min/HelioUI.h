/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Minimal UI
*/

class HelioMinUI;

#include "Helioduino.h"

class HelioMinUI : HelioUIInterface {
public:
    virtual void begin() override;

    virtual void setNeedsLayout() override;

protected:
};

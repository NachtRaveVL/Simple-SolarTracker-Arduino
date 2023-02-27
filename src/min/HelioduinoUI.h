/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Minimal UI
*/

class HelioMinUI;

#include "Helioduino.h"

class HelioMinUI : HelioduinoUIInterface {
public:
    virtual bool begin() override;

    virtual void setNeedsLayout() override;

protected:
};

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Full UI
*/

class HelioFullUI;

#include "Helioduino.h"

class HelioFullUI : HelioduinoUIInterface {
public:
    virtual bool begin() override;

    virtual void setNeedsLayout() override;

protected:
};

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Base UI
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioBaseUI_H
#define HelioBaseUI_H

class HelioduinoBaseUI : public HelioUIInterface {
public:
    virtual void init(HelioUIData *data = nullptr) override;
    virtual bool begin() override;

    virtual void setNeedsRedraw() override;

protected:
};

#endif // /ifndef HelioBaseUI_H
#endif

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino UI Data
*/

#include <Helioduino.h>
#include "HelioUIData.h"

HelioUIData::HelioUIData()
    : HelioData('H','U','I','D', 1),
      updatesPerSec(HELIO_UI_UPDATE_SPEED),
      displayTheme(Helio_DisplayTheme_Undefined),
      titleMode(Helio_TitleMode_Undefined),
      analogSlider(false), editingIcons(false),
      joystickCalib{0.5f,0.5f,0.05f}, touchscreenCalib{0}
{
    _size = sizeof(*this);
}

void HelioUIData::toJSONObject(JsonObject &objectOut) const
{
    HelioData::toJSONObject(objectOut);

    if (updatesPerSec != HELIO_UI_UPDATE_SPEED) { objectOut[SFP(HStr_Key_UpdatesPerSec)] = updatesPerSec; }
    if (displayTheme != Helio_DisplayTheme_Undefined) { objectOut[SFP(HStr_Key_DisplayTheme)] = displayTheme; }
    if (!isFPEqual(joystickCalib[0], 0.5f) || !isFPEqual(joystickCalib[1], 0.5f) || !isFPEqual(joystickCalib[2], 0.05f)) { objectOut[SFP(HStr_Key_JoystickCalib)] = commaStringFromArray(joystickCalib, 3); }
}

void HelioUIData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioData::fromJSONObject(objectIn);

    updatesPerSec = objectIn[SFP(HStr_Key_UpdatesPerSec)] | updatesPerSec;
    displayTheme = objectIn[SFP(HStr_Key_DisplayTheme)] | displayTheme;
    JsonVariantConst joystickCalibVar = objectIn[SFP(HStr_Key_JoystickCalib)];
    commaStringToArray(joystickCalibVar, joystickCalib, 3);
}

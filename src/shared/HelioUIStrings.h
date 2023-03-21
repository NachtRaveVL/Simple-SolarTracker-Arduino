/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino UI Strings/Prototypes
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioUIStrings_H
#define HelioUIStrings_H

// UI Strings Enumeration Table
enum HelioUI_String : unsigned short {
    HUIStr_Keys_MatrixActions,
    HUIStr_Keys_Matrix2x2Keys,
    HUIStr_Keys_Matrix3x4Keys,
    HUIStr_Keys_Matrix4x4Keys,

    HUIStr_Count
};

// Returns memory resident string from PROGMEM (Flash) UI string enumeration.
extern String stringFromPGM(HelioUI_String strNum);

// Makes UI Strings lookup go through EEPROM, with specified data begin address.
extern void beginUIStringsFromEEPROM(uint16_t uiDataAddress);

// Makes UI Strings lookup go through SD card strings file at file prefix.
extern void beginUIStringsFromSDCard(String uiDataFilePrefix);

#ifndef HELIO_DISABLE_BUILTIN_DATA
// Returns PROGMEM (Flash) address pointer given UI string number.
const char *pgmAddrForStr(HelioUI_String strNum);
#endif

#endif // /ifndef HelioUIStrings_H
#endif

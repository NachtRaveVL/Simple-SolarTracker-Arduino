/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino UI Strings/Prototypes
*/

#include "Helioduino.h"
#ifdef HELIO_USE_GUI
#include "HelioUIStrings.h"

#ifndef HELIO_DISABLE_BUILTIN_DATA
String stringFromPGMAddr(const char *flashUIStr); // implemented in base system
const char *pgmAddrForUIStr(HelioUI_String strNum);
#endif

static uint16_t _uiStrDataAddress((uint16_t)-1);
void beginUIStringsFromEEPROM(uint16_t uiDataAddress)
{
    _uiStrDataAddress = uiDataAddress;
}

static String _uiStrDataFilePrefix;
void beginUIStringsFromSDCard(String uiDataFilePrefix)
{
    _uiStrDataFilePrefix = uiDataFilePrefix;
}

inline String getUIStringsFilename()
{
    String filename; filename.reserve(_uiStrDataFilePrefix.length() + 11 + 1);
    filename.concat(_uiStrDataFilePrefix);
    filename.concat('u'); // Cannot use SFP here so have to do it the long way
    filename.concat('i');
    filename.concat('d');
    filename.concat('s');
    filename.concat('t');
    filename.concat('r');
    filename.concat('s');
    filename.concat('.');
    filename.concat('d');
    filename.concat('a');
    filename.concat('t');
    return filename;
}

String stringFromPGM(HelioUI_String strNum)
{
    static HelioUI_String _lookupStrNum = (HelioUI_String)-1; // Simple LRU cache reduces a lot of lookup access
    static String _lookupCachedRes;
    if (strNum == _lookupStrNum) { return _lookupCachedRes; }
    else { _lookupStrNum = strNum; } // _lookupCachedRes set below

    if (_uiStrDataAddress != (uint16_t)-1) {
        auto eeprom = getController()->getEEPROM();

        if (eeprom) {
            uint16_t lookupOffset = 0;
            eeprom->readBlock(_uiStrDataAddress + (sizeof(uint16_t) * ((int)strNum + 1)), // +1 for initial total size word
                              (uint8_t *)&lookupOffset, sizeof(lookupOffset));

            {   String retVal;
                char buffer[HELIO_STRING_BUFFER_SIZE] = {0};
                uint16_t bytesRead = eeprom->readBlock(lookupOffset, (uint8_t *)&buffer[0], HELIO_STRING_BUFFER_SIZE);
                retVal.concat(charsToString(buffer, bytesRead));

                while (strnlen(buffer, HELIO_STRING_BUFFER_SIZE) == HELIO_STRING_BUFFER_SIZE) {
                    lookupOffset += HELIO_STRING_BUFFER_SIZE;
                    bytesRead = eeprom->readBlock(lookupOffset, (uint8_t *)&buffer[0], HELIO_STRING_BUFFER_SIZE);
                    if (bytesRead) { retVal.concat(charsToString(buffer, bytesRead)); }
                }

                if (retVal.length()) {
                    return (_lookupCachedRes = retVal);
                }
            }
        }
    }

    if (_uiStrDataFilePrefix.length()) {
        #if HELIO_SYS_LEAVE_FILES_OPEN
            static
        #endif
        auto sd = getController()->getSDCard();

        if (sd) {
            String retVal;
            #if HELIO_SYS_LEAVE_FILES_OPEN
                static
            #endif
            auto file = sd->open(getUIStringsFilename().c_str(), FILE_READ);

            if (file) {
                uint16_t lookupOffset = 0;
                file.seek(sizeof(uint16_t) * (int)strNum);
                #if defined(ARDUINO_ARCH_RP2040) || defined(ESP_PLATFORM)
                    file.readBytes((char *)&lookupOffset, sizeof(lookupOffset));
                #else
                    file.readBytes((uint8_t *)&lookupOffset, sizeof(lookupOffset));
                #endif

                {   char buffer[HELIO_STRING_BUFFER_SIZE];
                    file.seek(lookupOffset);
                    auto bytesRead = file.readBytesUntil('\0', buffer, HELIO_STRING_BUFFER_SIZE);
                    retVal.concat(charsToString(buffer, bytesRead));

                    while (strnlen(buffer, HELIO_STRING_BUFFER_SIZE) == HELIO_STRING_BUFFER_SIZE) {
                        bytesRead = file.readBytesUntil('\0', buffer, HELIO_STRING_BUFFER_SIZE);
                        if (bytesRead) { retVal.concat(charsToString(buffer, bytesRead)); }
                    }
                }

                #if !HELIO_SYS_LEAVE_FILES_OPEN
                    file.close();
                #endif
            }

            #if !HELIO_SYS_LEAVE_FILES_OPEN
                getController()->endSDCard(sd);
            #endif
            if (retVal.length()) {
                return (_lookupCachedRes = retVal);
            }
        }
    }

    #ifndef HELIO_DISABLE_BUILTIN_DATA
        return (_lookupCachedRes = stringFromPGMAddr(pgmAddrForUIStr(strNum)));
    #else
        return (_lookupCachedRes = String());
    #endif
}

#ifndef HELIO_DISABLE_BUILTIN_DATA

const char *pgmAddrForUIStr(HelioUI_String strNum)
{
    switch(strNum) {
        case HUIStr_MatrixActions: {
            static const char flashUIStr_UI_MatrixActions[] PROGMEM = {HELIO_UI_MATRIX_ACTIONS};
            return flashUIStr_UI_MatrixActions;
        } break;
        case HUIStr_Matrix2x2Keys: {
            static const char flashUIStr_UI_Matrix2x2Keys[] PROGMEM = {HELIO_UI_2X2MATRIX_KEYS};
            return flashUIStr_UI_Matrix2x2Keys;
        } break;
        case HUIStr_Matrix3x4Keys: {
            static const char flashUIStr_UI_Matrix3x4Keys[] PROGMEM = {HELIO_UI_3X4MATRIX_KEYS};
            return flashUIStr_UI_Matrix3x4Keys;
        } break;
        case HUIStr_Matrix4x4Keys: {
            static const char flashUIStr_UI_Matrix4x4Keys[] PROGMEM = {HELIO_UI_4X4MATRIX_KEYS};
            return flashUIStr_UI_Matrix4x4Keys;
        } break;

        case HUIStr_Count: break;
    }
    return nullptr;
}

#endif
#endif

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Menu Screens
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioMenus_H
#define HelioMenus_H

class HelioMenu;

#include "HelioduinoUI.h"
#include "RemoteMenuItem.h"
#include "EditableLargeNumberMenuItem.h"

// Menu Screen Base
class HelioMenu
{
public:
    HelioMenu();
    virtual ~HelioMenu();

    virtual void loadMenu(MenuItem *addFrom = nullptr) = 0; // should call menuMgr.addMenuAfter()
    virtual MenuItem *getRootItem() = 0;

    inline bool isLoaded() const { return _loaded; }

protected:
    bool _loaded;
};

// Initializes an AnyMenuInfo structure
#define InitAnyMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback)\
    safeProgCpy(varName.name, CFP(strNum), NAME_SIZE_T);\
    varName.id = itemId;\
    varName.eepromAddr = eepromPosition;\
    varName.maxValue = valMaximum;\
    varName.callback = fnCallback

// Initializes a BooleanMenuInfo structure
#define InitBooleanMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback,boolNaming)\
    InitAnyMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback);\
    varName.naming = boolNaming

// Initializes a SubMenuInfo structure
#define InitSubMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback)\
    InitAnyMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback)

// Initializes an EnumMenuInfo structure
#define InitEnumMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback,enumItems)\
    InitAnyMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback);\
    varName.menuItems = enumItems

// Initializes an AnalogMenuInfo structure, with units from CFP()
#define InitAnalogMenuInfoUnits(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback,valOffset,valDivisor,unitsStrNum)\
    InitAnyMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback);\
    varName.offset = valOffset;\
    varName.divisor = valDivisor;\
    safeProgCpy(varName.unitName, CFP(unitsStrNum), UNIT_SIZE_T)

// Initializes an AnalogMenuInfo structure, with blank units
#define InitAnalogMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback,valOffset,valDivisor)\
    InitAnyMenuInfo(varName,strNum,itemId,eepromPosition,valMaximum,fnCallback);\
    varName.offset = valOffset;\
    varName.divisor = valDivisor;\
    varName.unitName[0] = '\000'

// Altered rendering callback that uses CFP()
#define H_RENDERING_CALLBACK_NAME_INVOKE(fnName, parent, strNum, eepromPosition, invoke) \
int fnName(RuntimeMenuItem* item, uint8_t row, RenderFnMode mode, char* buffer, int buffSize) { \
	switch(mode) { \
        case RENDERFN_NAME: \
            safeProgCpy(buffer, CFP(strNum), buffSize); \
            return true; \
        case RENDERFN_INVOKE: \
            invokeIfSafe(invoke, item); \
            return true; \
        case RENDERFN_EEPROM_POS: \
            return eepromPosition; \
        default: \
            return parent(item, row, mode, buffer, buffSize); \
	} \
}

#ifdef HELIO_DISABLE_BUILTIN_DATA
#define InfoPtrForItem(itemName,castType) (&init.minfo##itemName)
#define InfoLocation INFO_LOCATION_RAM
#else
#define InfoPtrForItem(itemName,castType) ((const castType *)CFP(HUIStr_Item_##itemName))
#define InfoLocation INFO_LOCATION_PGM
#endif

#include "screens/HelioMenuHome.h"
#include "screens/HelioMenuAlerts.h"
#include "screens/HelioMenuActuators.h"
#include "screens/HelioMenuSensors.h"
#include "screens/HelioMenuCrops.h"
#include "screens/HelioMenuReservoirs.h"
#include "screens/HelioMenuPowerRails.h"
#include "screens/HelioMenuScheduling.h"
#include "screens/HelioMenuSettings.h"
#include "screens/HelioMenuCropsLib.h"
#include "screens/HelioMenuAdditives.h"
#include "screens/HelioMenuCalibrations.h"
#include "screens/HelioMenuInformation.h"

#endif // /ifndef HelioMenus_H
#endif

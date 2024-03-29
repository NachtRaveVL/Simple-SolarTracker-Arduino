/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Home Menu Screen
*/

#include "../HelioduinoUI.h"
#ifdef HELIO_USE_GUI

void CALLBACK_FUNCTION gotoScreen(int id)
{
    switch (id) {
        case 7: // BackToOverview
            taskManager.scheduleOnce(0, []{
                if (getBaseUI()) { getBaseUI()->reset(); }
            });
            break;
        case 5: // Information
            // todo
            break;
        case 42: // Calibrations
            // todo
            break;
        case 3: // Settings
            // todo
            break;
        case 25: // Scheduling
            // todo
            break;
        case 24: // PowerRails
            // todo
            break;
        case 22: // Panels
            // todo
            break;
        case 21: // Sensors
            // todo
            break;
        case 20: // Actuators
            // todo
            break;
        case 1: // Alerts
            // todo
            break;
        default: // Home
            // todo
            break;
    }
}

#ifdef HELIO_UI_ENABLE_DEBUG_MENU

void CALLBACK_FUNCTION debugAction(int id)
{
    switch (id) {
        case 65: // ToggleBadConn
            // todo
            break;
        case 64: // ToggleFastTime
            // todo
            break;
        case 63: // TriggerSigTime
            getScheduler()->setNeedsUpdate();
            break;
        case 62: // TriggerSDCleanup
            // todo
            break;
        case 61: // TriggerLowMem
            // todo
            break;
        default: break;
    }
}

#else

void CALLBACK_FUNCTION debugAction(int id) { ; }

#endif // /ifdef HELIO_UI_ENABLE_DEBUG_MENU


HelioHomeMenu::HelioHomeMenu()
    : HelioMenu(), _items(nullptr)
{ ; }

HelioHomeMenu::~HelioHomeMenu()
{
    if (_items) { delete _items; }
}

void HelioHomeMenu::loadMenu(MenuItem *addFrom)
{
    if (!_items) {
        _loaded = (bool)(_items = new HelioHomeMenuItems());
        HELIO_SOFT_ASSERT(_items, SFP(HStr_Err_AllocationFailure));
    }
}

MenuItem *HelioHomeMenu::getRootItem()
{
    if (!_loaded) { loadMenu(); }
    return _loaded && _items ? &_items->menuAlerts : nullptr;
}

#ifdef HELIO_DISABLE_BUILTIN_DATA

HelioHomeMenuInfo::HelioHomeMenuInfo()
{
    InitAnyMenuInfo(minfoBackToOverview, HUIStr_Item_BackToOverview, 7, NO_ADDRESS, 0, gotoScreen);
    #ifdef HELIO_UI_ENABLE_DEBUG_MENU
        InitBooleanMenuInfo(minfoToggleBadConn, HUIStr_Item_ToggleBadConn, 65, NO_ADDRESS, 1, debugAction, NAMING_ON_OFF);
        InitBooleanMenuInfo(minfoToggleFastTime, HUIStr_Item_ToggleFastTime, 64, NO_ADDRESS, 1, debugAction, NAMING_ON_OFF);
        InitAnyMenuInfo(minfoTriggerSigTime, HUIStr_Item_TriggerSigTime, 63, NO_ADDRESS, 0, debugAction);
        InitAnyMenuInfo(minfoTriggerSDCleanup, HUIStr_Item_TriggerSDCleanup, 62, NO_ADDRESS, 0, debugAction);
        InitAnyMenuInfo(minfoTriggerLowMem, HUIStr_Item_TriggerLowMem, 61, NO_ADDRESS, 0, debugAction);
        InitAnyMenuInfo(minfoTriggerAutosave, HUIStr_Item_TriggerAutosave, 60, NO_ADDRESS, 0, debugAction);
        InitSubMenuInfo(minfoDebug, HUIStr_Item_Debug, 6, NO_ADDRESS, 0, debugAction);
    #endif
    InitAnyMenuInfo(minfoInformation, HUIStr_Item_Information, 5, NO_ADDRESS, 0, gotoScreen);
    InitAnyMenuInfo(minfoCalibrations, HUIStr_Item_Calibrations, 42, NO_ADDRESS, 0, gotoScreen);
    InitSubMenuInfo(minfoLibrary, HUIStr_Item_Library, 4, NO_ADDRESS, 0, NO_CALLBACK);
    InitAnyMenuInfo(minfoSettings, HUIStr_Item_Settings, 3, NO_ADDRESS, 0, gotoScreen);
    InitAnyMenuInfo(minfoScheduling, HUIStr_Item_Scheduling, 25, NO_ADDRESS, 0, gotoScreen);
    InitAnyMenuInfo(minfoPowerRails, HUIStr_Item_PowerRails, 24, NO_ADDRESS, 0, gotoScreen);
    InitAnyMenuInfo(minfoPanels, HUIStr_Item_Panels, 22, NO_ADDRESS, 0, gotoScreen);
    InitAnyMenuInfo(minfoSensors, HUIStr_Item_Sensors, 21, NO_ADDRESS, 0, gotoScreen);
    InitAnyMenuInfo(minfoActuators, HUIStr_Item_Actuators, 20, NO_ADDRESS, 0, gotoScreen);
    InitSubMenuInfo(minfoSystem, HUIStr_Item_System, 2, NO_ADDRESS, 0, NO_CALLBACK);
    InitAnyMenuInfo(minfoAlerts, HUIStr_Item_Alerts, 1, NO_ADDRESS, 0, gotoScreen);
}

#endif // /ifdef HELIO_DISABLE_BUILTIN_DATA

HelioHomeMenuItems::HelioHomeMenuItems() :
    #ifdef HELIO_DISABLE_BUILTIN_DATA
        init(),
    #endif
    menuBackToOverview(InfoPtrForItem(BackToOverview, AnyMenuInfo), nullptr, InfoLocation),
    #ifdef HELIO_UI_ENABLE_DEBUG_MENU
        menuToggleBadConn(InfoPtrForItem(ToggleBadConn, BooleanMenuInfo), false, nullptr, InfoLocation), // todo: initial value
        menuToggleFastTime(InfoPtrForItem(ToggleFastTime, BooleanMenuInfo), false, &menuToggleBadConn, InfoLocation), // todo: initial value
        menuTriggerSigTime(InfoPtrForItem(TriggerSigTime, AnyMenuInfo), &menuToggleFastTime, InfoLocation),
        menuTriggerSDCleanup(InfoPtrForItem(TriggerSDCleanup, AnyMenuInfo), &menuTriggerSigTime, InfoLocation),
        menuTriggerLowMem(InfoPtrForItem(TriggerLowMem, AnyMenuInfo), &menuTriggerSDCleanup, InfoLocation),
        menuTriggerAutosave(InfoPtrForItem(TriggerAutosave, AnyMenuInfo), &menuTriggerLowMem, InfoLocation),
        menuBackDebug(InfoPtrForItem(Debug, SubMenuInfo), &menuTriggerAutosave, InfoLocation),
        menuDebug(InfoPtrForItem(Debug, SubMenuInfo), &menuBackDebug, &menuBackToOverview, InfoLocation),
        menuInformation(InfoPtrForItem(Information, AnyMenuInfo), &menuDebug, InfoLocation),
    #else
        menuInformation(InfoPtrForItem(Information, AnyMenuInfo), &menuBackToOverview, InfoLocation),
    #endif
    menuCalibrations(InfoPtrForItem(Calibrations, AnyMenuInfo), nullptr, InfoLocation),
    menuBackLibrary(InfoPtrForItem(Library, SubMenuInfo), &menuCalibrations, InfoLocation),
    menuLibrary(InfoPtrForItem(Library, SubMenuInfo), &menuBackLibrary, &menuInformation, InfoLocation),
    menuSettings(InfoPtrForItem(Settings, AnyMenuInfo), &menuLibrary, InfoLocation),
    menuScheduling(InfoPtrForItem(Scheduling, AnyMenuInfo), nullptr, InfoLocation),
    menuPowerRails(InfoPtrForItem(PowerRails, AnyMenuInfo), &menuScheduling, InfoLocation),
    menuPanels(InfoPtrForItem(Panels, AnyMenuInfo), &menuPowerRails, InfoLocation),
    menuSensors(InfoPtrForItem(Sensors, AnyMenuInfo), &menuPanels, InfoLocation),
    menuActuators(InfoPtrForItem(Actuators, AnyMenuInfo), &menuSensors, InfoLocation),
    menuBackSystem(InfoPtrForItem(System, SubMenuInfo), &menuActuators, InfoLocation),
    menuSystem(InfoPtrForItem(System, SubMenuInfo), &menuBackSystem, &menuSettings, InfoLocation),
    menuAlerts(InfoPtrForItem(Alerts, AnyMenuInfo), &menuSystem, InfoLocation)
{
    menuBackToOverview.setReadOnly(true);
    #ifdef HELIO_UI_ENABLE_DEBUG_MENU
        menuToggleBadConn.setReadOnly(true);
        menuToggleFastTime.setReadOnly(true);
        menuTriggerSigTime.setReadOnly(true);
        menuTriggerSDCleanup.setReadOnly(true);
        menuTriggerLowMem.setReadOnly(true);
        menuTriggerAutosave.setReadOnly(true);
    #endif
    menuInformation.setReadOnly(true);
    menuCalibrations.setReadOnly(true);
    menuSettings.setReadOnly(true);
    menuScheduling.setReadOnly(true);
    menuPowerRails.setReadOnly(true);
    menuPanels.setReadOnly(true);
    menuSensors.setReadOnly(true);
    menuActuators.setReadOnly(true);
    menuAlerts.setReadOnly(true);
}

#endif

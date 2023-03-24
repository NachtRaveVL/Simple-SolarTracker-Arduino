/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Home Menu Screen
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioMenuHome_H
#define HelioMenuHome_H

class HelioHomeMenu;
#ifdef HELIO_DISABLE_BUILTIN_DATA
struct HelioHomeMenuInfo;
#endif
struct HelioHomeMenuItems;

#include "../HelioduinoUI.h"

class HelioHomeMenu : public HelioMenu
{
public:
    HelioHomeMenu();
    virtual ~HelioHomeMenu();

    virtual void loadMenu(MenuItem *addFrom = nullptr) override;
    virtual MenuItem *getRootItem() override;

    inline HelioHomeMenuItems &getItems() { return *_items; }

protected:
    HelioHomeMenuItems *_items;
};

#ifdef HELIO_DISABLE_BUILTIN_DATA
struct HelioHomeMenuInfo {
    HelioHomeMenuInfo();

    AnyMenuInfo minfoBackToOverview;
#ifdef HELIO_UI_ENABLE_DEBUG_MENU
    BooleanMenuInfo minfoToggleBadConn;
    BooleanMenuInfo minfoToggleFastTime;
    AnyMenuInfo minfoTriggerSigTime;
    AnyMenuInfo minfoTriggerSDCleanup;
    AnyMenuInfo minfoTriggerLowMem;
    AnyMenuInfo minfoTriggerAutosave;
    SubMenuInfo minfoDebug;
#endif // /ifdef HELIO_UI_ENABLE_DEBUG_MENU
    AnyMenuInfo minfoInformation;
    AnyMenuInfo minfoCalibrations;
    SubMenuInfo minfoLibrary;
    AnyMenuInfo minfoSettings;
    AnyMenuInfo minfoScheduling;
    AnyMenuInfo minfoPowerRails;
    AnyMenuInfo minfoPanels;
    AnyMenuInfo minfoSensors;
    AnyMenuInfo minfoActuators;
    SubMenuInfo minfoSystem;
    AnyMenuInfo minfoAlerts;
};
#endif // /ifdef HELIO_DISABLE_BUILTIN_DATA

struct HelioHomeMenuItems {
    HelioHomeMenuItems();

#ifdef HELIO_DISABLE_BUILTIN_DATA
    HelioHomeMenuInfo init;
#endif

    ActionMenuItem menuBackToOverview;
#ifdef HELIO_UI_ENABLE_DEBUG_MENU
    BooleanMenuItem menuToggleBadConn;
    BooleanMenuItem menuToggleFastTime;
    ActionMenuItem menuTriggerSigTime;
    ActionMenuItem menuTriggerSDCleanup;
    ActionMenuItem menuTriggerLowMem;
    ActionMenuItem menuTriggerAutosave;
    BackMenuItem menuBackDebug;
    SubMenuItem menuDebug;
#endif // /ifdef HELIO_UI_ENABLE_DEBUG_MENU
    ActionMenuItem menuInformation;
    ActionMenuItem menuCalibrations;
    BackMenuItem menuBackLibrary;
    SubMenuItem menuLibrary;
    ActionMenuItem menuSettings;
    ActionMenuItem menuScheduling;
    ActionMenuItem menuPowerRails;
    ActionMenuItem menuPanels;
    ActionMenuItem menuSensors;
    ActionMenuItem menuActuators;
    BackMenuItem menuBackSystem;
    SubMenuItem menuSystem;
    ActionMenuItem menuAlerts;
};

#endif // /ifndef HelioMenuHome_H
#endif

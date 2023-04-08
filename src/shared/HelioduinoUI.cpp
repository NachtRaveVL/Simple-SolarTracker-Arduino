/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Base UI
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioduinoBaseUI::HelioduinoBaseUI(String deviceUUID, UIControlSetup uiControlSetup, UIDisplaySetup uiDisplaySetup, bool isActiveLowIO, bool allowInterruptableIO, bool enableTcUnicodeFonts, bool enableBufferedVRAM)
    : _appInfo{0}, _uiCtrlSetup(uiControlSetup), _uiDispSetup(uiDisplaySetup),
      _isActiveLow(isActiveLowIO), _allowISR(allowInterruptableIO), _isTcUnicodeFonts(enableTcUnicodeFonts), _isBufferedVRAM(enableBufferedVRAM),
      _uiData(nullptr), _input(nullptr), _display(nullptr), _remoteServer(nullptr), _backlight(nullptr), _blTimeout(0),
      _overview(nullptr), _homeMenu(nullptr), _clockFont(nullptr), _detailFont(nullptr), _itemFont(nullptr), _titleFont(nullptr)
{
    if (getController()) { strncpy(_appInfo.name, getController()->getSystemNameChars(), 30); }
    strncpy(_appInfo.uuid, deviceUUID.c_str(), 38);
    pintype_t ledPin = uiDisplaySetup.getBacklightPin();
    if (uiDisplaySetup.dispCfgType != UIDisplaySetup::LCD && isValidPin(ledPin)) { // LCD has its own backlight
        switch (uiDisplaySetup.getBacklightMode()) {
            case Helio_BacklightMode_Inverted:
                _backlight = new HelioDigitalPin(ledPin, OUTPUT, ACT_LOW, hpinchnl_none);
                break;
            case Helio_BacklightMode_PWM:
                _backlight = new HelioAnalogPin(ledPin, OUTPUT, uiDisplaySetup.getBacklightBitRes(),
#ifdef ESP32
                                                uiDisplaySetup.getBacklightChannel(),
#endif
#ifdef ESP_PLATFORM
                                                uiDisplaySetup.getBacklightFrequency(),
#endif
                                                hpinchnl_none);
                break;
            default: // Normal
                _backlight = new HelioDigitalPin(ledPin, OUTPUT, ACT_HIGH, hpinchnl_none);
                break;
        }
        HELIO_SOFT_ASSERT(_backlight, SFP(HStr_Err_AllocationFailure));

        if (_backlight) { _backlight->init(); }
    }
}

HelioduinoBaseUI::~HelioduinoBaseUI()
{
    if (_overview) { delete _overview; }
    while (_remotes.size()) {
        delete (*_remotes.begin());
        _remotes.erase(_remotes.begin());
    }
    if (_input) { delete _input; }
    if (_display) { delete _display; }
    if (_remoteServer) { delete _remoteServer; }
    if (_backlight) { delete _backlight; }
}

void HelioduinoBaseUI::init(uint8_t updatesPerSec, Helio_DisplayTheme displayTheme, Helio_TitleMode titleMode, bool analogSlider, bool editingIcons)
{
    if (!_uiData) {
        _uiData = new HelioUIData();
        HELIO_SOFT_ASSERT(_uiData, SFP(HStr_Err_AllocationFailure));
    }
    if (_uiData) {
        _uiData->updatesPerSec = updatesPerSec;
        _uiData->displayTheme = displayTheme;
        _uiData->titleMode = titleMode;
        _uiData->analogSlider = analogSlider;
        _uiData->editingIcons = editingIcons;
    }
    
    if (!_homeMenu) { // must stay allocated while menuMgr active
        _homeMenu = new HelioHomeMenu();
        HELIO_SOFT_ASSERT(_homeMenu, SFP(HStr_Err_AllocationFailure));
    }
}

HelioUIData *HelioduinoBaseUI::init(HelioUIData *uiData)
{
    if (uiData && (_uiData = uiData)) { // Customized data
        init(_uiData->updatesPerSec,
             _uiData->displayTheme, _uiData->titleMode,
             _uiData->analogSlider, _uiData->editingIcons);
    } else if (_display) { // Display driver default
        _display->initBaseUIFromDefaults(); // calls back into above init with default settings for display
    } else { // Remote control default
        init(2, Helio_DisplayTheme_Undefined, Helio_TitleMode_Always);
    }
    return _uiData;
}

void HelioduinoBaseUI::begin()
{
    BaseMenuRenderer *baseRenderer = nullptr;

    if (_display) {
        _display->begin();

        // Base rendering setup for all displays
        baseRenderer = _display->getBaseRenderer();
        if (baseRenderer) {
            baseRenderer->setCustomDrawingHandler(this);
            baseRenderer->setUpdatesPerSecond(_uiData->updatesPerSec);
        }

        setBacklightEnable(true);
    }

    if (_input) { // Driver responsible for call to menuMgr.init[-like]()
        _input->begin(_display, _homeMenu ? _homeMenu->getRootItem() : nullptr);
    } else { // Default/remote init
        menuMgr.initWithoutInput(baseRenderer, _homeMenu ? _homeMenu->getRootItem() : nullptr);
    }

    if (_display) { // setupRendering() typically results in display driver refresh/reorient
        _display->setupRendering(_uiData->displayTheme, _uiData->titleMode,
                                 _itemFont, _titleFont,
                                 _uiData->analogSlider, _uiData->editingIcons,
                                 _isTcUnicodeFonts);
    }

    #if HELIO_UI_START_AT_OVERVIEW
        gotoScreen(HELIO_UI_OVERVIEW_ACT_MENU_ID);
    #endif
}

void HelioduinoBaseUI::setNeedsRedraw()
{
    if (_overview) { _overview->setNeedsFullRedraw(); }
    if (_homeMenu) { menuMgr.notifyStructureChanged(); }
}

SwitchInterruptMode HelioduinoBaseUI::getISRMode() const
{
    SwitchInterruptMode isrMode(SWITCHES_POLL_EVERYTHING);
    if (_allowISR) {
        bool mainPinsInterruptable = _input->areMainPinsInterruptable();
        bool allPinsInterruptable = mainPinsInterruptable && _input->areAllPinsInterruptable();
        isrMode = (allPinsInterruptable ? SWITCHES_NO_POLLING : (mainPinsInterruptable ? SWITCHES_POLL_KEYS_ONLY : SWITCHES_POLL_EVERYTHING));
    }
    return isrMode;
}

void HelioduinoBaseUI::setBacklightEnable(bool enabled)
{
    if (_uiDispSetup.dispCfgType != UIDisplaySetup::LCD && _backlight) {
        if (enabled) {
            if (_backlight->isDigitalType()) {
                ((HelioDigitalPin *)_backlight)->activate();
            } else {
                ((HelioAnalogPin *)_backlight)->analogWrite(1.0f);
            }
        } else {
            if (_backlight->isDigitalType()) {
                ((HelioDigitalPin *)_backlight)->deactivate();
            } else {
                ((HelioAnalogPin *)_backlight)->analogWrite(0.0f); // todo: nice backlight-out anim
            }
            _blTimeout = 0;
        }
    } else if (_uiDispSetup.dispCfgType == UIDisplaySetup::LCD && _display) {
        if (enabled) {
            if (_uiDispSetup.getBacklightMode() != Helio_BacklightMode_PWM) {
                ((HelioDisplayLiquidCrystal *)_display)->getLCD().backlight();
            } else {
                ((HelioDisplayLiquidCrystal *)_display)->getLCD().setBacklight(255);
            }
        } else {
            if (_uiDispSetup.getBacklightMode() != Helio_BacklightMode_PWM) {
                ((HelioDisplayLiquidCrystal *)_display)->getLCD().noBacklight();
            } else {
                ((HelioDisplayLiquidCrystal *)_display)->getLCD().setBacklight(0); // todo: nice backlight-out anim
            }
            _blTimeout = 0;
        }
    }
}

void HelioduinoBaseUI::started(BaseMenuRenderer *currentRenderer)
{
    // overview screen started
    if (_display) {
        if (_overview) { _overview->setNeedsFullRedraw(); }

        _blTimeout = (_backlight || _uiDispSetup.dispCfgType == UIDisplaySetup::LCD ? unixNow() + HELIO_UI_BACKLIGHT_TIMEOUT : 0);
    }
}

void HelioduinoBaseUI::reset()
{
    // menu interaction timeout
    if (_display) {
        #if HELIO_UI_DEALLOC_AFTER_USE
            if (_homeMenu) { _homeMenu->unloadSubMenus(); }
        #endif

        if (!_overview) {
            _overview = _display->allocateOverview(_clockFont, _detailFont);
            HELIO_SOFT_ASSERT(_overview, SFP(HStr_Err_AllocationFailure));
        }

        _display->getBaseRenderer()->takeOverDisplay();
    }
}

void HelioduinoBaseUI::renderLoop(unsigned int currentValue, RenderPressMode userClick)
{
    // render overview screen until key interruption
    if (_display) {
        if (userClick == RPRESS_NONE) {
            if (_overview) { _overview->renderOverview(_display->isLandscape(), _display->getScreenSize()); }

            if (_blTimeout && unixNow() >= _blTimeout) { setBacklightEnable(false); }
        } else {
            _display->getBaseRenderer()->giveBackDisplay();

            setBacklightEnable(true);
            _blTimeout = 0;

            #if HELIO_UI_DEALLOC_AFTER_USE
                if (_overview) { delete _overview; _overview = nullptr; }
            #endif
        }
    }
}

#endif

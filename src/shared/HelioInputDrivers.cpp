/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Input Drivers
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI
#include <DfRobotInputAbstraction.h>


HelioInputDriver::HelioInputDriver(Pair<uint8_t, const pintype_t *> controlPins)
    : _pins(controlPins)
{ ; }

bool HelioInputDriver::areAllPinsInterruptable() const
{
    for (int i = 0; i < _pins.first; ++i) {
        if (!(isValidPin(_pins.second[i]) && checkPinCanInterrupt(_pins.second[i]))) {
            return false;
        }
    }
    return true;
}


HelioInputRotary::HelioInputRotary(Pair<uint8_t, const pintype_t *> controlPins, Helio_EncoderSpeed encoderSpeed)
    : HelioInputDriver(controlPins), _encoderSpeed(encoderSpeed)
{ ; }

void HelioInputRotary::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    menuMgr.initForEncoder(renderer, initialItem, _pins.second[0], _pins.second[1], _pins.second[2], _encoderSpeed == Helio_EncoderSpeed_FullCycle ? FULL_CYCLE : _encoderSpeed == Helio_EncoderSpeed_HalfCycle ? HALF_CYCLE : QUARTER_CYCLE);
    if (_pins.first > 3 && isValidPin(_pins.second[3])) menuMgr.setBackButton(_pins.second[3]);
    if (_pins.first > 4 && isValidPin(_pins.second[4])) menuMgr.setNextButton(_pins.second[4]);    
}

bool HelioInputRotary::areMainPinsInterruptable() const
{
    return _pins.first >= 3 &&
           isValidPin(_pins.second[0]) && checkPinCanInterrupt(_pins.second[0]) &&
           isValidPin(_pins.second[1]) && checkPinCanInterrupt(_pins.second[1]) &&
           isValidPin(_pins.second[2]) && checkPinCanInterrupt(_pins.second[2]);
}


HelioInputUpDownButtons::HelioInputUpDownButtons(Pair<uint8_t, const pintype_t *> controlPins, uint16_t keyRepeatSpeed)
    : HelioInputDriver(controlPins), _keySpeed(keyRepeatSpeed), _dfRobotIORef(nullptr)
{ ; }

HelioInputUpDownButtons::HelioInputUpDownButtons(bool isDFRobotShield_unused, uint16_t keyRepeatSpeed)
    : HelioInputDriver(make_pair((uint8_t)5, (const pintype_t *)(new pintype_t[5]))), _keySpeed(keyRepeatSpeed), _dfRobotIORef(inputFromDfRobotShield())
{
    pintype_t *pins = const_cast<pintype_t *>(_pins.second);
    HELIO_SOFT_ASSERT(pins, SFP(HStr_Err_AllocationFailure));

    if (pins) {
        pins[0] = (pintype_t)DF_KEY_UP;
        pins[1] = (pintype_t)DF_KEY_DOWN;
        pins[2] = (pintype_t)DF_KEY_SELECT;
        pins[3] = (pintype_t)DF_KEY_LEFT;
        pins[4] = (pintype_t)DF_KEY_RIGHT;
    }
    #if NUM_ANALOG_INPUTS > 0
        pinMode(A0, INPUT);
    #endif
}

HelioInputUpDownButtons::~HelioInputUpDownButtons()
{
    if (_dfRobotIORef) {
        pintype_t *pins = const_cast<pintype_t *>(_pins.second);
        if (pins) { delete [] pins; }
    }
}

void HelioInputUpDownButtons::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    menuMgr.initForUpDownOk(renderer, initialItem, _pins.second[1], _pins.second[0], _pins.second[2], _keySpeed);
    if (_pins.first > 3 && isValidPin(_pins.second[3])) menuMgr.setBackButton(_pins.second[3]);
    if (_pins.first > 4 && isValidPin(_pins.second[4])) menuMgr.setNextButton(_pins.second[4]);
}

bool HelioInputUpDownButtons::areMainPinsInterruptable() const
{
    return _pins.first >= 3 &&
           isValidPin(_pins.second[0]) && checkPinCanInterrupt(_pins.second[0]) &&
           isValidPin(_pins.second[1]) && checkPinCanInterrupt(_pins.second[1]) &&
           isValidPin(_pins.second[2]) && checkPinCanInterrupt(_pins.second[2]);
}


HelioInputESP32TouchKeys::HelioInputESP32TouchKeys(Pair<uint8_t, const pintype_t *> controlPins, uint16_t keyRepeatSpeed, uint16_t switchThreshold, Helio_ESP32Touch_HighRef highVoltage, Helio_ESP32Touch_LowRef lowVoltage, Helio_ESP32Touch_HighRefAtten attenuation)
    : HelioInputDriver(controlPins), _keySpeed(keyRepeatSpeed)
#ifdef ESP32
      , _esp32Touch(switchThreshold,
                    highVoltage == Helio_ESP32Touch_HighRef_V_2V4 ? TOUCH_HVOLT_2V4 : highVoltage == Helio_ESP32Touch_HighRef_V_2V5 ? TOUCH_HVOLT_2V5 :
                    highVoltage == Helio_ESP32Touch_HighRef_V_2V6 ? TOUCH_HVOLT_2V6 : highVoltage == Helio_ESP32Touch_HighRef_V_2V7 ? TOUCH_HVOLT_2V7 : 
                    highVoltage == Helio_ESP32Touch_HighRef_Max ? TOUCH_HVOLT_MAX : TOUCH_HVOLT_KEEP,
                    lowVoltage == Helio_ESP32Touch_LowRef_V_0V5 ? TOUCH_LVOLT_0V5 : lowVoltage == Helio_ESP32Touch_LowRef_V_0V6 ? TOUCH_LVOLT_0V6 :
                    lowVoltage == Helio_ESP32Touch_LowRef_V_0V7 ? TOUCH_LVOLT_0V7 : lowVoltage == Helio_ESP32Touch_LowRef_V_0V8 ? TOUCH_LVOLT_0V8 :
                    lowVoltage == Helio_ESP32Touch_LowRef_Max ? TOUCH_LVOLT_MAX : TOUCH_LVOLT_KEEP,
                    attenuation == Helio_ESP32Touch_HighRefAtten_V_0V ? TOUCH_HVOLT_ATTEN_0V : attenuation == Helio_ESP32Touch_HighRefAtten_V_0V5 ? TOUCH_HVOLT_ATTEN_0V5 :
                    attenuation == Helio_ESP32Touch_HighRefAtten_V_1V ? TOUCH_HVOLT_ATTEN_1V : attenuation == Helio_ESP32Touch_HighRefAtten_V_1V5 ? TOUCH_HVOLT_ATTEN_1V5 :
                    attenuation == Helio_ESP32Touch_HighRefAtten_Max ? TOUCH_HVOLT_ATTEN_MAX : TOUCH_HVOLT_ATTEN_KEEP)
#endif
{ ; }

void HelioInputESP32TouchKeys::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    menuMgr.initForUpDownOk(renderer, initialItem, _pins.second[1], _pins.second[0], _pins.second[2], _keySpeed);
    if (_pins.first > 3 && isValidPin(_pins.second[3])) menuMgr.setBackButton(_pins.second[3]);
    if (_pins.first > 4 && isValidPin(_pins.second[4])) menuMgr.setNextButton(_pins.second[4]);
    #ifdef ESP32
        _esp32Touch.ensureInterruptRegistered();
    #endif
}


HelioInputJoystick::HelioInputJoystick(Pair<uint8_t, const pintype_t *> controlPins, millis_t repeatDelay, float decreaseDivisor, float jsCenterX, float jsCenterY, float jsZeroTol)
    : HelioInputDriver(controlPins), _repeatDelay(repeatDelay), _decreaseDivisor(decreaseDivisor),
      _joystickCalib{jsCenterX, jsCenterY, jsZeroTol},
      _joystickMultiIo(200),
      _joystickIoXAxis(internalAnalogIo(), controlPins.second[0], jsCenterX)
{
    multiIoAddExpander(&_joystickMultiIo, &_joystickIoXAxis, 5);
}

static void menuMgrOnMenuSelect(pinid_t /*key*/, bool held)
{
    menuMgr.onMenuSelect(held);
}

static void menuMgrPerformDirectionMoveTrue(pinid_t /*key*/, bool held)
{
    menuMgr.performDirectionMove(true);
}

static void menuMgrPerformDirectionMoveFalse(pinid_t /*key*/, bool held)
{
    menuMgr.performDirectionMove(false);
}

static void menuMgrValueChanged(int val)
{
    menuMgr.valueChanged(val);
}

void HelioInputJoystick::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    if (isValidPin(_pins.second[2])) {
        switches.addSwitch(_pins.second[2], NULL);
        switches.onRelease(_pins.second[2], &menuMgrOnMenuSelect);
    }
    if (isValidPin(_pins.second[0])) {
        switches.addSwitch(200, &menuMgrPerformDirectionMoveTrue);
        switches.addSwitch(201, &menuMgrPerformDirectionMoveFalse);
    }
    if (isValidPin(_pins.second[1])) {
        setupAnalogJoystickEncoder(internalAnalogIo(), _pins.second[1], &menuMgrValueChanged);
    }
    if (switches.getEncoder()) {
        reinterpret_cast<JoystickSwitchInput*>(switches.getEncoder())->setTolerance(_joystickCalib[1], _joystickCalib[2]);
        reinterpret_cast<JoystickSwitchInput*>(switches.getEncoder())->setAccelerationParameters(_repeatDelay, _decreaseDivisor);
    }

    menuMgr.initWithoutInput(renderer, initialItem);
}


HelioInputMatrix2x2::HelioInputMatrix2x2(Pair<uint8_t, const pintype_t *> controlPins, millis_t repeatDelay, millis_t repeatInterval)
    : HelioInputDriver(controlPins),
      _keyboard(),
      _keyboardLayout(2,2,SFP(HUIStr_Keys_Matrix2x2Keys).c_str()),
      _tcMenuKeyListener(SFP(HUIStr_Keys_MatrixActions)[0], SFP(HUIStr_Keys_MatrixActions)[1], SFP(HUIStr_Keys_MatrixActions)[2], SFP(HUIStr_Keys_MatrixActions)[3])
{
    // todo expander setup
    _keyboardLayout.setRowPin(0, controlPins.second[0]);
    _keyboardLayout.setRowPin(1, controlPins.second[1]);
    _keyboardLayout.setColPin(0, controlPins.second[2]);
    _keyboardLayout.setColPin(1, controlPins.second[3]);
    _keyboard.setRepeatKeyMillis(repeatDelay, repeatInterval);
}

void HelioInputMatrix2x2::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    _keyboard.initialise(internalDigitalIo(), &_keyboardLayout, &_tcMenuKeyListener, false);
}


HelioInputMatrix3x4::HelioInputMatrix3x4(Pair<uint8_t, const pintype_t *> controlPins, millis_t repeatDelay, millis_t repeatInterval, Helio_EncoderSpeed encoderSpeed)
    : HelioInputDriver(controlPins),
      _keyboard(),
      _keyboardLayout(4,3,SFP(HUIStr_Keys_Matrix3x4Keys).c_str()),
      _tcMenuKeyListener(SFP(HUIStr_Keys_MatrixActions)[0], SFP(HUIStr_Keys_MatrixActions)[1], SFP(HUIStr_Keys_MatrixActions)[2], SFP(HUIStr_Keys_MatrixActions)[3]),
      _rotaryEncoder(nullptr)
{
    // todo expander setup
    _keyboardLayout.setRowPin(0, controlPins.second[0]);
    _keyboardLayout.setRowPin(1, controlPins.second[1]);
    _keyboardLayout.setRowPin(2, controlPins.second[2]);
    _keyboardLayout.setRowPin(3, controlPins.second[3]);
    _keyboardLayout.setColPin(0, controlPins.second[4]);
    _keyboardLayout.setColPin(1, controlPins.second[5]);
    _keyboardLayout.setColPin(2, controlPins.second[6]);
    _keyboard.setRepeatKeyMillis(repeatDelay, repeatInterval);

    if (isValidPin(controlPins.second[7])) {
        _rotaryEncoder = new HelioInputRotary(make_pair((uint8_t)(controlPins.first - 7), &controlPins.second[7]), encoderSpeed);
    }
}

HelioInputMatrix3x4::~HelioInputMatrix3x4()
{
    if (_rotaryEncoder) { delete _rotaryEncoder; }
}

void HelioInputMatrix3x4::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    _keyboard.initialise(internalDigitalIo(), &_keyboardLayout, &_tcMenuKeyListener, false);
    if (_rotaryEncoder) { _rotaryEncoder->begin(renderer, initialItem); }
}


HelioInputMatrix4x4::HelioInputMatrix4x4(Pair<uint8_t, const pintype_t *> controlPins, millis_t repeatDelay, millis_t repeatInterval, Helio_EncoderSpeed encoderSpeed)
    : HelioInputDriver(controlPins),
      _keyboard(),
      _keyboardLayout(4,4,SFP(HUIStr_Keys_Matrix4x4Keys).c_str()),
      _tcMenuKeyListener(SFP(HUIStr_Keys_MatrixActions)[0], SFP(HUIStr_Keys_MatrixActions)[1], SFP(HUIStr_Keys_MatrixActions)[2], SFP(HUIStr_Keys_MatrixActions)[3]),
      _rotaryEncoder(nullptr)
{
    // todo expander setup
    _keyboardLayout.setRowPin(0, controlPins.second[0]);
    _keyboardLayout.setRowPin(1, controlPins.second[1]);
    _keyboardLayout.setRowPin(2, controlPins.second[2]);
    _keyboardLayout.setRowPin(3, controlPins.second[3]);
    _keyboardLayout.setColPin(0, controlPins.second[4]);
    _keyboardLayout.setColPin(1, controlPins.second[5]);
    _keyboardLayout.setColPin(2, controlPins.second[6]);
    _keyboardLayout.setColPin(3, controlPins.second[7]);
    _keyboard.setRepeatKeyMillis(repeatDelay, repeatInterval);

    if (isValidPin(controlPins.second[8])) {
        _rotaryEncoder = new HelioInputRotary(make_pair((uint8_t)(controlPins.first - 8), &controlPins.second[8]), encoderSpeed);
    }
}

HelioInputMatrix4x4::~HelioInputMatrix4x4()
{
    if (_rotaryEncoder) { delete _rotaryEncoder; }
}

void HelioInputMatrix4x4::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    _keyboard.initialise(internalDigitalIo(), &_keyboardLayout, &_tcMenuKeyListener, false);
    if (_rotaryEncoder) { _rotaryEncoder->begin(renderer, initialItem); }
}


HelioInputResistiveTouch::HelioInputResistiveTouch(Pair<uint8_t, const pintype_t *> controlPins, HelioDisplayDriver *displayDriver)
    : HelioInputDriver(controlPins),
      _touchInterrogator(controlPins.second[0], controlPins.second[1], controlPins.second[2], controlPins.second[3]),
      _touchOrientation(
         /*swap*/ displayDriver->getRotation() == Helio_DisplayRotation_R1 || displayDriver->getRotation() == Helio_DisplayRotation_R3,
         /*invX*/ displayDriver->getRotation() == Helio_DisplayRotation_R1 || displayDriver->getRotation() == Helio_DisplayRotation_R2 || displayDriver->getRotation() == Helio_DisplayRotation_HorzMirror,
         /*invY*/ displayDriver->getRotation() == Helio_DisplayRotation_R3 || displayDriver->getRotation() == Helio_DisplayRotation_R2 || displayDriver->getRotation() == Helio_DisplayRotation_VertMirror
      ),
      _touchScreen(&_touchInterrogator, displayDriver->getGraphicsRenderer(), _touchOrientation)
{ ; }

void HelioInputResistiveTouch::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    _touchScreen.start();
    menuMgr.initWithoutInput(renderer, initialItem);
}


HelioInputTouchscreen::HelioInputTouchscreen(Pair<uint8_t, const pintype_t *> controlPins, Helio_DisplayRotation displayRotation)
    : HelioInputDriver(controlPins),
      #ifdef HELIO_UI_ENABLE_XPT2046TS
          _touchScreen(controlPins.second[0], controlPins.second[1]),
      #else
          _touchScreen(),
      #endif
      #ifdef HELIO_UI_ENABLE_BSP_TOUCH
          _touchInterrogator(TFT_GFX_WIDTH, TFT_GFX_HEIGHT),
      #else
          _touchInterrogator(_touchScreen),
      #endif
      _touchOrientation(
         /*swap*/ displayRotation == Helio_DisplayRotation_R1 || displayRotation == Helio_DisplayRotation_R3,
         /*invX*/ displayRotation == Helio_DisplayRotation_R1 || displayRotation == Helio_DisplayRotation_R2 || displayRotation == Helio_DisplayRotation_HorzMirror,
         /*invY*/ displayRotation == Helio_DisplayRotation_R3 || displayRotation == Helio_DisplayRotation_R2 || displayRotation == Helio_DisplayRotation_VertMirror
      )
{ ; }

void HelioInputTouchscreen::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    _touchInterrogator.init(); // begins touch device
    menuMgr.initWithoutInput(renderer, initialItem);
    #ifdef HELIO_UI_ENABLE_XPT2046TS
        _touchScreen.setRotation(getBaseUI() ? (uint8_t)getBaseUI()->getDisplaySetup().getDisplayRotation() : 0);
    #endif
}


HelioInputTFTTouch::HelioInputTFTTouch(Pair<uint8_t, const pintype_t *> controlPins, HelioDisplayTFTeSPI *displayDriver, bool useRawTouch)
    : HelioInputDriver(controlPins),
      _touchInterrogator(&displayDriver->getGfx(), displayDriver->getScreenSize().first, displayDriver->getScreenSize().second, useRawTouch),
      _touchOrientation(
         /*swap*/ displayDriver->getRotation() == Helio_DisplayRotation_R1 || displayDriver->getRotation() == Helio_DisplayRotation_R3,
         /*invX*/ displayDriver->getRotation() == Helio_DisplayRotation_R1 || displayDriver->getRotation() == Helio_DisplayRotation_R2 || displayDriver->getRotation() == Helio_DisplayRotation_HorzMirror,
         /*invY*/ displayDriver->getRotation() == Helio_DisplayRotation_R3 || displayDriver->getRotation() == Helio_DisplayRotation_R2 || displayDriver->getRotation() == Helio_DisplayRotation_VertMirror
      ),
      _touchScreen(&_touchInterrogator, displayDriver->getGraphicsRenderer(), _touchOrientation)
{ ; }

void HelioInputTFTTouch::begin(MenuRenderer *renderer, MenuItem *initialItem)
{
    _touchScreen.start();
    menuMgr.initWithoutInput(renderer, initialItem);
}

#endif

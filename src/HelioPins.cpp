/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Pins
*/

#include "Helioduino.h"
#include "AnalogDeviceAbstraction.h"

HelioPin *newPinObjectFromSubData(const HelioPinData *dataIn)
{
    if (!dataIn || !isValidType(dataIn->type)) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && isValidType(dataIn->type), SFP(HStr_Err_InvalidParameter));

    if (dataIn) {
        switch (dataIn->type) {
            case HelioPin::Digital:
                return new HelioDigitalPin(dataIn);
            case HelioPin::Analog:
                return new HelioAnalogPin(dataIn);
            default: break;
        }
    }

    return nullptr;
}

HelioPin::HelioPin()
    : type(Unknown), pin(hpin_none), mode(Helio_PinMode_Undefined), channel(hpinchnl_none)
{ ; }

HelioPin::HelioPin(int classType, pintype_t pinNumber, Helio_PinMode pinMode, int8_t pinChannel)
    : type((typeof(type))classType), pin(pinNumber), mode(pinMode),
      channel(isValidChannel(pinChannel) ? pinChannel : (isValidPin(pinNumber) && pinNumber >= hpin_virtual ? pinChannelForExpanderChannel(pinNumber - hpin_virtual) : hpinchnl_none))
{ ; }

HelioPin::HelioPin(const HelioPinData *dataIn)
    : type((typeof(type))(dataIn->type)), pin(dataIn->pin), mode(dataIn->mode), channel(dataIn->channel)
{ ; }

HelioPin::operator HelioDigitalPin() const
{
    return (isDigitalType() || isDigital() || (!isUnknownType() && !isAnalog())) ? HelioDigitalPin(pin, mode, channel) : HelioDigitalPin();
}

HelioPin::operator HelioAnalogPin() const
{
    return (isAnalogType() || isAnalog() || (!isUnknownType() && !isDigital())) ? HelioAnalogPin(pin, mode, isOutput() ? DAC_RESOLUTION : ADC_RESOLUTION, channel) : HelioAnalogPin();
}

void HelioPin::saveToData(HelioPinData *dataOut) const
{
    dataOut->type = (int8_t)type;
    dataOut->pin = pin;
    dataOut->mode = mode;
    dataOut->channel = channel;
}

void HelioPin::init()
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid()) {
            if (!(isExpanded() || isVirtual())) {
                HELIO_SOFT_ASSERT(!isMuxed() || channel == pinChannelForMuxerChannel(muxerChannelForPinChannel(channel)), SFP(HStr_Err_NotConfiguredProperly));

                switch (mode) {
                    case Helio_PinMode_Digital_Input:
                    case Helio_PinMode_Analog_Input:
                        pinMode(pin, INPUT);
                        break;

                    case Helio_PinMode_Digital_Input_PullUp:
                        pinMode(pin, INPUT_PULLUP);
                        break;

                    case Helio_PinMode_Digital_Input_PullDown:
                        #if HAS_INPUT_PULLDOWN
                            pinMode(pin, INPUT_PULLDOWN);
                        #else
                            pinMode(pin, INPUT);
                        #endif
                        break;

                    case Helio_PinMode_Digital_Output:
                    case Helio_PinMode_Digital_Output_PushPull:
                    case Helio_PinMode_Analog_Output:
                        pinMode(pin, OUTPUT);
                        break;

                    default:
                        break;
                }
            } else {
                #ifdef HELIO_USE_MULTITASKING
                    HELIO_SOFT_ASSERT(isVirtual() && pin == pinNumberForPinChannel(channel), SFP(HStr_Err_NotConfiguredProperly));
                    HELIO_SOFT_ASSERT(channel == pinChannelForExpanderChannel(channel), SFP(HStr_Err_NotConfiguredProperly));

                    auto expander = getController() ? getController()->getPinExpander(isValidChannel(channel) ? expanderPosForPinChannel(channel) : expanderPosForPinNumber(pin)) : nullptr;
                    if (expander) {
                        #if HAS_INPUT_PULLDOWN
                            expander->getIoAbstraction()->pinDirection(channel % 16, isOutput() ? OUTPUT : mode == Helio_PinMode_Digital_Input_PullUp ? INPUT_PULLUP : mode == Helio_PinMode_Digital_Input_PullDown ? INPUT_PULLDOWN : INPUT);
                        #else
                            expander->getIoAbstraction()->pinDirection(channel % 16, isOutput() ? OUTPUT : mode == Helio_PinMode_Digital_Input_PullUp ? INPUT_PULLUP : INPUT);
                        #endif
                    }
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
            }
        }
    #endif
}

void HelioPin::deinit()
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid()) {
            if (!(isExpanded() || isVirtual())) {
                pinMode(pin, INPUT);
            } else {
                #ifdef HELIO_USE_MULTITASKING
                    auto expander = getController() ? getController()->getPinExpander(isValidChannel(channel) ? expanderPosForPinChannel(channel) : expanderPosForPinNumber(pin)) : nullptr;
                    if (expander) {
                        expander->getIoAbstraction()->pinDirection(channel % 16, INPUT);
                    }
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
            }
        }
    #endif
}

bool HelioPin::enablePin(int step)
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid() && isValidChannel(channel)) {
            if (isMuxed()) {
                SharedPtr<HelioPinMuxer> muxer = getController() ? getController()->getPinMuxer(pin) : nullptr;
                if (muxer) {
                    switch (step) {
                        case 0: muxer->selectChannel(muxerChannelForPinChannel(channel)); muxer->activate(); return true;
                        case 1: muxer->selectChannel(muxerChannelForPinChannel(channel)); return true;
                        case 2: muxer->activate(); return true;
                        default: return false;
                    }
                }
            } else if (isExpanded() || isVirtual()) {
                #ifdef HELIO_USE_MULTITASKING
                    auto expander = getController() ? getController()->getPinExpander(isValidChannel(channel) ? expanderPosForPinChannel(channel) : expanderPosForPinNumber(pin)) : nullptr;
                    return expander && expander->trySyncChannel();
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
            }
        }
        return false;
    #else
        return isValid() && isValidChannel(channel);
    #endif
}


HelioDigitalPin::HelioDigitalPin()
    : HelioPin(Digital), activeLow(false)
{ ; }

HelioDigitalPin::HelioDigitalPin(pintype_t pinNumber, ard_pinmode_t pinMode, int8_t pinChannel)
    : HelioPin(Digital, pinNumber, pinMode != OUTPUT ? (pinMode != INPUT ? (pinMode == INPUT_PULLUP ? Helio_PinMode_Digital_Input_PullUp : Helio_PinMode_Digital_Input_PullDown)
                                                                         : Helio_PinMode_Digital_Input)
                                                     : (pinMode == OUTPUT ? Helio_PinMode_Digital_Output : Helio_PinMode_Digital_Output_PushPull), pinChannel),
      activeLow(pinMode == INPUT || pinMode == INPUT_PULLUP || pinMode == OUTPUT)
{ ; }

HelioDigitalPin::HelioDigitalPin(pintype_t pinNumber, Helio_PinMode pinMode, int8_t pinChannel)
    : HelioPin(Digital, pinNumber, pinMode, pinChannel),
      activeLow(pinMode == Helio_PinMode_Digital_Input ||
                pinMode == Helio_PinMode_Digital_Input_PullUp ||
                pinMode == Helio_PinMode_Digital_Output)
{ ; }

HelioDigitalPin::HelioDigitalPin(pintype_t pinNumber, ard_pinmode_t pinMode, bool isActiveLow, int8_t pinChannel)
    : HelioPin(Digital, pinNumber, pinMode != OUTPUT ? (isActiveLow ? Helio_PinMode_Digital_Input_PullUp : Helio_PinMode_Digital_Input_PullDown)
                                                     : (isActiveLow ? Helio_PinMode_Digital_Output : Helio_PinMode_Digital_Output_PushPull), pinChannel),
      activeLow(isActiveLow)
{ ; }

HelioDigitalPin::HelioDigitalPin(pintype_t pinNumber, Helio_PinMode pinMode, bool isActiveLow, int8_t pinChannel)
    : HelioPin(Digital, pinNumber, pinMode, pinChannel),
      activeLow(isActiveLow)
{ ; }

HelioDigitalPin::HelioDigitalPin(const HelioPinData *dataIn)
    : HelioPin(dataIn), activeLow(dataIn->dataAs.digitalPin.activeLow)
{ ; }

void HelioDigitalPin::saveToData(HelioPinData *dataOut) const
{
    HelioPin::saveToData(dataOut);

    dataOut->dataAs.digitalPin.activeLow = activeLow;
}

ard_pinstatus_t HelioDigitalPin::digitalRead()
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid()) {
            if (isValidChannel(channel)) { selectAndActivatePin(); }
            if (!(isExpanded() || isVirtual())) {
                return ::digitalRead(pin);
            } else {
                #ifdef HELIO_USE_MULTITASKING
                    auto expander = getController() ? getController()->getPinExpander(isValidChannel(channel) ? expanderPosForPinChannel(channel) : expanderPosForPinNumber(pin)) : nullptr;
                    if (expander) {
                        return (ard_pinstatus_t)(expander->getIoAbstraction()->readValue(channel % 16));
                    }
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
            }
        }
    #endif
    return (ard_pinstatus_t)-1;
}

void HelioDigitalPin::digitalWrite(ard_pinstatus_t status)
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid()) {
            if (!(isExpanded() || isVirtual())) {
                if (isMuxed()) { selectPin(); }
                ::digitalWrite(pin, status);
            } else {
                #ifdef HELIO_USE_MULTITASKING
                    auto expander = getController() ? getController()->getPinExpander(isValidChannel(channel) ? expanderPosForPinChannel(channel) : expanderPosForPinNumber(pin)) : nullptr;
                    if (expander) {
                        expander->getIoAbstraction()->writeValue(channel % 16, (uint8_t)status);
                    }
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
            }
            if (isValidChannel(channel)) { activatePin(); }
        }
    #endif
}


HelioAnalogPin::HelioAnalogPin()
    : HelioPin(Analog), bitRes(0)
#ifdef ESP32
      , pwmChannel(-1)
#endif
#ifdef ESP_PLATFORM
      , pwmFrequency(0)
#endif
{ ; }

HelioAnalogPin::HelioAnalogPin(pintype_t pinNumber, ard_pinmode_t pinMode, uint8_t analogBitRes,
#ifdef ESP32
                               uint8_t pinPWMChannel,
#endif
#ifdef ESP_PLATFORM
                               float pinPWMFrequency,
#endif
                               int8_t pinChannel)
    : HelioPin(Analog, pinNumber, pinMode != OUTPUT ? Helio_PinMode_Analog_Input : Helio_PinMode_Analog_Output, pinChannel),
      bitRes(analogBitRes ? analogBitRes : (pinMode == OUTPUT ? DAC_RESOLUTION : ADC_RESOLUTION))
#ifdef ESP32
      , pwmChannel(pinPWMChannel)
#endif
#ifdef ESP_PLATFORM
      , pwmFrequency(pinPWMFrequency)
#endif
{ ; }

HelioAnalogPin::HelioAnalogPin(pintype_t pinNumber, Helio_PinMode pinMode, uint8_t analogBitRes,
#ifdef ESP32
                               uint8_t pinPWMChannel,
#endif
#ifdef ESP_PLATFORM
                               float pinPWMFrequency,
#endif
                               int8_t pinChannel)
    : HelioPin(Analog, pinNumber, pinMode, pinChannel),
      bitRes(analogBitRes ? analogBitRes : (pinMode == Helio_PinMode_Analog_Output ? DAC_RESOLUTION : ADC_RESOLUTION))
#ifdef ESP32
      , pwmChannel(pinPWMChannel)
#endif
#ifdef ESP_PLATFORM
      , pwmFrequency(pinPWMFrequency)
#endif
{ ; }

HelioAnalogPin::HelioAnalogPin(const HelioPinData *dataIn)
    : HelioPin(dataIn), bitRes(dataIn->dataAs.analogPin.bitRes)
#ifdef ESP32
      , pwmChannel(dataIn->dataAs.analogPin.pwmChannel)
#endif
#ifdef ESP_PLATFORM
      , pwmFrequency(dataIn->dataAs.analogPin.pwmFrequency)
#endif
{ ; }

void HelioAnalogPin::init()
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid()) {
            if (!(isExpanded() || isVirtual())) {
                HelioPin::init();

                #ifdef ESP32
                    ledcAttachPin(pin, pwmChannel);
                    ledcSetup(pwmChannel, pwmFrequency, bitRes.bits);
                #endif
            } else {
                #ifdef HELIO_USE_MULTITASKING
                    HELIO_SOFT_ASSERT(isVirtual() && pin == pinNumberForPinChannel(channel), SFP(HStr_Err_NotConfiguredProperly));
                    HELIO_SOFT_ASSERT(channel == pinChannelForExpanderChannel(channel), SFP(HStr_Err_NotConfiguredProperly));

                    auto expander = getController() ? getController()->getPinExpander(isValidChannel(channel) ? expanderPosForPinChannel(channel) : expanderPosForPinNumber(pin)) : nullptr;
                    if (expander) {
                        auto ioDir = isOutput() ? AnalogDirection::DIR_OUT : AnalogDirection::DIR_IN;
                        auto analogIORef = (AnalogDevice *)(expander->getIoAbstraction());
                        analogIORef->initPin(channel % 16, ioDir);

                        auto ioRefBits = analogIORef->getBitDepth(ioDir, channel % 16);
                        if (bitRes.bits != ioRefBits) {
                            bitRes = BitResolution(ioRefBits);
                        }
                    }
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
            }
        }
    #endif
}

void HelioAnalogPin::saveToData(HelioPinData *dataOut) const
{
    HelioPin::saveToData(dataOut);

    dataOut->dataAs.analogPin.bitRes = bitRes.bits;
    #ifdef ESP32
        dataOut->dataAs.analogPin.pwmChannel = pwmChannel;
    #endif
    #ifdef ESP_PLATFORM
        dataOut->dataAs.analogPin.pwmFrequency = pwmFrequency;
    #endif
}

float HelioAnalogPin::analogRead()
{
    return bitRes.transform(analogRead_raw());
}

int HelioAnalogPin::analogRead_raw()
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid()) {
            if (isValidChannel(channel)) { selectAndActivatePin(); }
            if (!(isExpanded() || isVirtual())) {
                #if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
                    analogReadResolution(bitRes.bits);
                #endif
                return ::analogRead(pin);
            } else {
                #ifdef HELIO_USE_MULTITASKING
                    auto expander = getController() ? getController()->getPinExpander(isValidChannel(channel) ? expanderPosForPinChannel(channel) : expanderPosForPinNumber(pin)) : nullptr;
                    if (expander) {
                        auto analogIORef = (AnalogDevice *)(expander->getIoAbstraction());
                        analogIORef->getCurrentValue(channel % 16);
                    }
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
            }
        }
    #endif
    return 0;
}

void HelioAnalogPin::analogWrite(float amount)
{
    analogWrite_raw(bitRes.inverseTransform(amount));
}

void HelioAnalogPin::analogWrite_raw(int amount)
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid()) {
            if (!(isExpanded() || isVirtual())) {
                if (isMuxed()) { selectPin(); }
                #ifdef ESP32
                    ledcWrite(pwmChannel, amount);
                #else
                    #if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
                        analogWriteResolution(bitRes.bits);
                    #elif defined(ESP8266)
                        analogWriteRange(bitRes.maxVal);
                        analogWriteFreq(pwmFrequency);
                    #endif
                    ::analogWrite(pin, amount);
                #endif
            } else {
                #ifdef HELIO_USE_MULTITASKING
                    auto expander = getController() ? getController()->getPinExpander(isValidChannel(channel) ? expanderPosForPinChannel(channel) : expanderPosForPinNumber(pin)) : nullptr;
                    if (expander) {
                        auto analogIORef = (AnalogDevice *)(expander->getIoAbstraction());
                        analogIORef->setCurrentValue(channel % 16, amount);
                    }
                #else
                    HELIO_HARD_ASSERT(false, SFP(HStr_Err_NotConfiguredProperly));
                #endif
            }
            if (isValidChannel(channel)) { activatePin(); }
        }
    #endif
}


HelioPinData::HelioPinData()
    : HelioSubData((int8_t)HelioPin::Unknown), pin(hpin_none), mode(Helio_PinMode_Undefined), channel(hpinchnl_none), dataAs{0}
{ ; }

void HelioPinData::toJSONObject(JsonObject &objectOut) const
{
    HelioSubData::toJSONObject(objectOut);

    if (isValidPin(pin)) { objectOut[SFP(HStr_Key_Pin)] = pin; }
    if (mode != Helio_PinMode_Undefined) { objectOut[SFP(HStr_Key_Mode)] = pinModeToString(mode); }
    if (isValidChannel(channel)) { objectOut[SFP(HStr_Key_Channel)] = channel; }

    if (mode != Helio_PinMode_Undefined) {
        if (!(mode == Helio_PinMode_Analog_Input || mode == Helio_PinMode_Analog_Output)) {
            objectOut[SFP(HStr_Key_ActiveLow)] = dataAs.digitalPin.activeLow;
        } else {
            objectOut[SFP(HStr_Key_BitRes)] = dataAs.analogPin.bitRes;
            #ifdef ESP32
                objectOut[SFP(HStr_Key_PWMChannel)] = dataAs.analogPin.pwmChannel;
            #endif
            #ifdef ESP_PLATFORM
                objectOut[SFP(HStr_Key_PWMFrequency)] = dataAs.analogPin.pwmFrequency;
            #endif
        }
    }
}

void HelioPinData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioSubData::fromJSONObject(objectIn);

    pin = objectIn[SFP(HStr_Key_Pin)] | pin;
    mode = pinModeFromString(objectIn[SFP(HStr_Key_Mode)]);
    channel = objectIn[SFP(HStr_Key_Channel)] | channel;

    if (mode != Helio_PinMode_Undefined) {
        if (!(mode == Helio_PinMode_Analog_Input || mode == Helio_PinMode_Analog_Output)) {
            type = (int8_t)HelioPin::Digital;
            dataAs.digitalPin.activeLow = objectIn[SFP(HStr_Key_ActiveLow)] | dataAs.digitalPin.activeLow;
        } else {
            type = (int8_t)HelioPin::Analog;
            dataAs.analogPin.bitRes = objectIn[SFP(HStr_Key_BitRes)] | dataAs.analogPin.bitRes;
            #ifdef ESP32
                dataAs.analogPin.pwmChannel = objectIn[SFP(HStr_Key_PWMChannel)] | dataAs.analogPin.pwmChannel;
            #endif
            #ifdef ESP_PLATFORM
                dataAs.analogPin.pwmFrequency = objectIn[SFP(HStr_Key_PWMFrequency)] | dataAs.analogPin.pwmFrequency;
            #endif
        }
    } else {
        type = (int8_t)HelioPin::Unknown;
    }
}


HelioPinMuxer::HelioPinMuxer()
    : _signal(), _chipEnable(), _channelPins{hpin_none},
      _channelBits(0), _channelSelect(-1), _usingISR(false)
{
    _signal.channel = hpinchnl_none; // unused
}

HelioPinMuxer::HelioPinMuxer(HelioPin signalPin,
                             pintype_t *muxChannelPins, int8_t muxChannelBits,
                             HelioDigitalPin chipEnablePin, HelioDigitalPin interruptPin)
    : _signal(signalPin), _chipEnable(chipEnablePin), _interrupt(interruptPin),
      _channelPins{ muxChannelBits > 0 ? muxChannelPins[0] : hpin_none,
                    muxChannelBits > 1 ? muxChannelPins[1] : hpin_none,
                    muxChannelBits > 2 ? muxChannelPins[2] : hpin_none,
                    muxChannelBits > 3 ? muxChannelPins[3] : hpin_none },
      _channelBits(muxChannelBits), _channelSelect(-1), _usingISR(false)
{
    _signal.channel = hpinchnl_none; // unused
}

void HelioPinMuxer::init()
{
    _signal.deinit();
    _chipEnable.init();
    _chipEnable.deactivate();

    if (isValidPin(_channelPins[0])) {
        pinMode(_channelPins[0], OUTPUT);
        ::digitalWrite(_channelPins[0], LOW);

        if (isValidPin(_channelPins[1])) {
            pinMode(_channelPins[1], OUTPUT);
            ::digitalWrite(_channelPins[1], LOW);

            if (isValidPin(_channelPins[2])) {
                pinMode(_channelPins[2], OUTPUT);
                ::digitalWrite(_channelPins[2], LOW);

                if (isValidPin(_channelPins[3])) {
                    pinMode(_channelPins[3], OUTPUT);
                    ::digitalWrite(_channelPins[3], LOW);
                }
            }
        }
    }
    _channelSelect = 0;
}

bool HelioPinMuxer::tryRegisterISR(bool anyChange)
{
    #ifdef HELIO_USE_MULTITASKING
        if (!_usingISR && _interrupt.isValid() && checkPinCanInterrupt(_interrupt.pin)) {
            taskManager.addInterrupt(&interruptImpl, _interrupt.pin, !anyChange ? (_interrupt.activeLow ? FALLING : RISING) : CHANGE);
            _usingISR = true;
        }
    #endif
    return _usingISR;
}

void HelioPinMuxer::selectChannel(uint8_t channelNumber)
{
    if (_channelSelect != channelNumber) {
        #if HELIO_MUXERS_SHARED_ADDR_BUS
            if (getController()) { getController()->deactivatePinMuxers(); }
        #endif

        if (isValidPin(_channelPins[0])) {
            ::digitalWrite(_channelPins[0], (channelNumber >> 0) & 1 ? HIGH : LOW);

            if (isValidPin(_channelPins[1])) {
                ::digitalWrite(_channelPins[1], (channelNumber >> 1) & 1 ? HIGH : LOW);

                if (isValidPin(_channelPins[2])) {
                    ::digitalWrite(_channelPins[2], (channelNumber >> 2) & 1 ? HIGH : LOW);

                    if (isValidPin(_channelPins[3])) {
                        ::digitalWrite(_channelPins[3], (channelNumber >> 3) & 1 ? HIGH : LOW);
                    }
                }
            }
        }
        _channelSelect = channelNumber;
    }
}

void HelioPinMuxer::setIsActive(bool isActive)
{
    if (isActive) {
        _signal.init();
        _chipEnable.activate();
    } else {
        _chipEnable.deactivate();
        _signal.deinit();
    }
}

#ifdef HELIO_USE_MULTITASKING

HelioPinExpander::HelioPinExpander()
    : _expander(0), _channelBits(0), _ioRef(nullptr), _interrupt(), _usingISR(false)
{ ; }

HelioPinExpander::HelioPinExpander(hposi_t expanderPos, uint8_t channelBits, IoAbstractionRef ioRef, HelioDigitalPin interruptPin)
    : _expander(expanderPos), _channelBits(channelBits), _ioRef(ioRef), _interrupt(interruptPin), _usingISR(false)
{ ; }

bool HelioPinExpander::tryRegisterISR(bool anyChange)
{
    #ifdef HELIO_USE_MULTITASKING
        if (!_usingISR && _interrupt.isValid() && checkPinCanInterrupt(_interrupt.pin)) {
            taskManager.addInterrupt(&interruptImpl, _interrupt.pin, !anyChange ? (_interrupt.activeLow ? FALLING : RISING) : CHANGE);
            _usingISR = true;
        }
    #endif
    return _usingISR;
}

bool HelioPinExpander::trySyncChannel()
{
    return _ioRef->sync();
}

#endif // /ifdef HELIO_USE_MULTITASKING

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Pins
*/

#include "Helioduino.h"

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
    : type(Unknown), pin(-1), mode(Helio_PinMode_Undefined), channel(-1)
{ ; }

HelioPin::HelioPin(int classType, pintype_t pinNumber, Helio_PinMode pinMode, uint8_t muxChannel)
    : type((typeof(type))classType), pin(pinNumber), mode(pinMode), channel(muxChannel)
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
            switch (mode) {
                case Helio_PinMode_Digital_Input_Floating:
                case Helio_PinMode_Analog_Input:
                    pinMode(pin, INPUT);
                    break;

                case Helio_PinMode_Digital_Input_PullUp:
                    pinMode(pin, INPUT_PULLUP);
                    break;

                case Helio_PinMode_Digital_Input_PullDown:
                    #if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_MBED) || defined(ESP32) || defined(ARDUINO_ARCH_STM32) || defined(CORE_TEENSY) || defined(INPUT_PULLDOWN)
                        pinMode(pin, INPUT_PULLDOWN);
                    #else
                        pinMode(pin, INPUT);
                    #endif
                    break;

                case Helio_PinMode_Digital_Output_OpenDrain:
                case Helio_PinMode_Digital_Output_PushPull:
                case Helio_PinMode_Analog_Output:
                    pinMode(pin, OUTPUT);
                    break;

                default:
                    break;
            }
        }
    #endif
}

bool HelioPin::tryEnableMuxer()
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid() && isMuxed()) {
            SharedPtr<HelioPinMuxer> muxer = getHelioInstance() ? getHelioInstance()->getPinMuxer(pin) : nullptr;
            if (muxer) {
                muxer->selectChannel(channel);
                return true;
            }
        }
        return false;
    #else
        return isValid() && isMuxed();
    #endif
}


HelioDigitalPin::HelioDigitalPin()
    : HelioPin(Digital)
{ ; }

HelioDigitalPin::HelioDigitalPin(pintype_t pinNumber, ard_pinmode_t pinMode, uint8_t muxChannel)
    : HelioPin(Digital, pinNumber, pinMode != OUTPUT ? (pinMode != INPUT ? (pinMode == INPUT_PULLUP ? Helio_PinMode_Digital_Input_PullUp : Helio_PinMode_Digital_Input_PullDown)
                                                                         : Helio_PinMode_Digital_Input_Floating)
                                                     : (pinMode == OUTPUT ? Helio_PinMode_Digital_Output_OpenDrain : Helio_PinMode_Digital_Output_PushPull), muxChannel),
      activeLow(pinMode == INPUT || pinMode == INPUT_PULLUP || pinMode == OUTPUT)
{ ; }

HelioDigitalPin::HelioDigitalPin(pintype_t pinNumber, Helio_PinMode pinMode, uint8_t muxChannel)
    : HelioPin(Digital, pinNumber, pinMode, muxChannel),
      activeLow(pinMode == Helio_PinMode_Digital_Input_Floating ||
                pinMode == Helio_PinMode_Digital_Input_PullUp ||
                pinMode == Helio_PinMode_Digital_Output_OpenDrain)
{ ; }

HelioDigitalPin::HelioDigitalPin(pintype_t pinNumber, ard_pinmode_t pinMode, bool isActiveLow, uint8_t muxChannel)
    : HelioPin(Digital, pinNumber, pinMode != OUTPUT ? (isActiveLow ? Helio_PinMode_Digital_Input_PullUp : Helio_PinMode_Digital_Input_PullDown)
                                                     : (isActiveLow ? Helio_PinMode_Digital_Output_OpenDrain : Helio_PinMode_Digital_Output_PushPull), muxChannel),
      activeLow(isActiveLow)
{ ; }

HelioDigitalPin::HelioDigitalPin(pintype_t pinNumber, Helio_PinMode pinMode, bool isActiveLow, uint8_t muxChannel)
    : HelioPin(Digital, pinNumber, pinMode, muxChannel),
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
        if (isValid() && (!isMuxed() || tryEnableMuxer())) {
            return ::digitalRead(pin);
        }
    #endif
    return (ard_pinstatus_t)-1;
}

void HelioDigitalPin::digitalWrite(ard_pinstatus_t status)
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid() && (!isMuxed() || tryEnableMuxer())) {
            ::digitalWrite(pin, status);
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
                               uint8_t muxChannel)
    : HelioPin(Analog, pinNumber, pinMode != OUTPUT ? Helio_PinMode_Analog_Input : Helio_PinMode_Analog_Output, muxChannel),
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
                               uint8_t muxChannel)
    : HelioPin(Analog, pinNumber, pinMode, muxChannel),
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
      , pinPWMChannel(dataIn->dataAs.analogPin.pwmChannel)
#endif
#ifdef ESP_PLATFORM
      , pinPWMFrequency(dataIn->dataAs.analogPin.pwmFrequency)
#endif
{ ; }

void HelioAnalogPin::init()
{
    #if !HELIO_SYS_DRY_RUN_ENABLE
        if (isValid()) {
            HelioPin::init();
            #ifdef ESP32
                ledcAttachPin(pin, pwmChannel);
                ledcSetup(pwmChannel, pwmFrequency, bitRes.bits);
            #endif
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
        if (isValid() && (!isMuxed() || tryEnableMuxer())) {
            #if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
                analogReadResolution(bitRes.bits);
            #endif
            return ::analogRead(pin);
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
        if (isValid() && (!isMuxed() || tryEnableMuxer())) {
            #ifdef ESP32
                ledcWrite(pwmChannel, val);
            #else
                #if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
                    analogWriteResolution(bitRes.bits);
                #elif defined(ESP8266)
                    analogWriteRange(bitRes.maxVal);
                    analogWriteFreq(pwmFrequency);
                #endif
                ::analogWrite(pin, amount);
            #endif
        }
    #endif
}


HelioPinData::HelioPinData()
    : HelioSubData((int8_t)HelioPin::Unknown), pin(-1), mode(Helio_PinMode_Undefined), channel(-1), dataAs{0}
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
    : _signal(), _chipEnable(), _channelPins{-1,-1,-1,-1,-1}, _channelSelect(0)
{ ; }

HelioPinMuxer::HelioPinMuxer(HelioPin signalPin,
                             pintype_t *muxChannelPins, uint8_t muxChannelBits,
                             HelioDigitalPin chipEnablePin)
    : _signal(signalPin), _chipEnable(chipEnablePin),
      _channelPins{ muxChannelBits > 0 ? muxChannelPins[0] : -1,
                    muxChannelBits > 1 ? muxChannelPins[1] : -1,
                    muxChannelBits > 2 ? muxChannelPins[2] : -1,
                    muxChannelBits > 3 ? muxChannelPins[3] : -1,
                    muxChannelBits > 4 ? muxChannelPins[4] : -1 },
      _channelBits(muxChannelBits), _channelSelect(0)
{
    _signal.channel = -1; // unused
}

HelioPinMuxer::HelioPinMuxer(const HelioPinMuxerData *dataIn)
    : _signal(&dataIn->signal), _chipEnable(&dataIn->chipEnable),
      _channelPins{ dataIn->channelPins[0],
                    dataIn->channelPins[1],
                    dataIn->channelPins[2],
                    dataIn->channelPins[3],
                    dataIn->channelPins[4] },
      _channelBits(dataIn->channelBits), _channelSelect(0)
{
    _signal.channel = -1; // unused
}

void HelioPinMuxer::saveToData(HelioPinMuxerData *dataOut) const
{
    _signal.saveToData(&dataOut->signal);
    _chipEnable.saveToData(&dataOut->chipEnable);
    dataOut->channelPins[0] = _channelPins[0];
    dataOut->channelPins[1] = _channelPins[1];
    dataOut->channelPins[2] = _channelPins[2];
    dataOut->channelPins[3] = _channelPins[3];
    dataOut->channelPins[4] = _channelPins[4];
    dataOut->channelBits = _channelBits;
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

                    if (isValidPin(_channelPins[4])) {
                        pinMode(_channelPins[4], OUTPUT);
                        ::digitalWrite(_channelPins[4], LOW);
                    }
                }
            }
        }
    }
    _channelSelect = 0;
}

void HelioPinMuxer::selectChannel(uint8_t channelNumber)
{
    if (_channelSelect != channelNumber) {
        // While we could be a bit smarter about which muxers we disable, storing that
        // wouldn't necessarily be worth the gain. The assumption is all that muxers in
        // system occupy the same channel select bus, even if that isn't the case.
        if (getHelioInstance()) { getHelioInstance()->deselectPinMuxers(); }

        if (isValidPin(_channelPins[0])) {
            ::digitalWrite(_channelPins[0], (channelNumber >> 0) & 1 ? HIGH : LOW);

            if (isValidPin(_channelPins[1])) {
                ::digitalWrite(_channelPins[1], (channelNumber >> 1) & 1 ? HIGH : LOW);

                if (isValidPin(_channelPins[2])) {
                    ::digitalWrite(_channelPins[2], (channelNumber >> 2) & 1 ? HIGH : LOW);

                    if (isValidPin(_channelPins[3])) {
                        ::digitalWrite(_channelPins[3], (channelNumber >> 3) & 1 ? HIGH : LOW);

                        if (isValidPin(_channelPins[4])) {
                            ::digitalWrite(_channelPins[4], (channelNumber >> 4) & 1 ? HIGH : LOW);
                        }
                    }
                }
            }
        }
        _channelSelect = channelNumber;
    }

    _signal.init();
    _chipEnable.activate();
}

void HelioPinMuxer::deselect()
{
    _chipEnable.deactivate();
    _signal.deinit();
}


HelioPinMuxerData::HelioPinMuxerData()
    : HelioSubData(0), signal(), chipEnable(), channelPins{-1,-1,-1,-1,-1}, channelBits(0)
{ ; }

void HelioPinMuxerData::toJSONObject(JsonObject &objectOut) const
{
    //HelioSubData::toJSONObject(objectOut); // purposeful no call to base method (ignores type)

    if (isValidPin(signal.pin)) {
        JsonObject signalPinObj = objectOut.createNestedObject(SFP(HStr_Key_SignalPin));
        signal.toJSONObject(signalPinObj);
    }
    if (isValidPin(chipEnable.pin)) {
        JsonObject chipEnablePinObj = objectOut.createNestedObject(SFP(HStr_Key_ChipEnablePin));
        chipEnable.toJSONObject(chipEnablePinObj);
    }
    if (channelBits && isValidPin(channelPins[0])) {
        objectOut[SFP(HStr_Key_ChannelPins)] = commaStringFromArray(channelPins, channelBits);
    }
}

void HelioPinMuxerData::fromJSONObject(JsonObjectConst &objectIn)
{
    //HelioSubData::fromJSONObject(objectIn); // purposeful no call to base method (ignores type)

    JsonObjectConst signalPinObj = objectIn[SFP(HStr_Key_SignalPin)];
    if (!signalPinObj.isNull()) { signal.fromJSONObject(signalPinObj); }
    JsonObjectConst chipEnablePinObj = objectIn[SFP(HStr_Key_ChipEnablePin)];
    if (!chipEnablePinObj.isNull()) { chipEnable.fromJSONObject(chipEnablePinObj); }
    JsonVariantConst channelPinsVar = objectIn[SFP(HStr_Key_ChannelPins)];
    commaStringToArray(channelPinsVar, channelPins, 5);
    for (channelBits = 0; channelBits < 5 && isValidPin(channelPins[channelBits]); ++channelBits) { ; }
}

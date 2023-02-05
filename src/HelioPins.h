/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Pins
*/

#ifndef HelioPin_H
#define HelioPin_H

struct HelioPin;
struct HelioDigitalPin;
struct HelioAnalogPin;
struct HelioPinData;

class HelioPinMuxer;
struct HelioPinMuxerData;

#include "Helioduino.h"
#include "HelioUtils.h"

// Creates Pin from passed Pin data (return ownership transfer - user code *must* delete returned Pin)
extern HelioPin *newPinObjectFromSubData(const HelioPinData *dataIn);


// Pin Base
struct HelioPin {
    enum : signed char { Digital, Analog, Unknown = -1 } type;  // Pin type (custom RTTI)
    inline bool isDigitalType() const { return type == Digital; }
    inline bool isAnalogType() const { return type == Analog; }
    inline bool isUnknownType() const { return type <= Unknown; }

    pintype_t pin;                                          // Pin number (else -1 if undefined)
    Helio_PinMode mode;                                     // Pin mode setting
    uint8_t channel;                                        // Muxing channel select (else -1 if unused)

    HelioPin();
    HelioPin(int classType, pintype_t pinNumber = -1, Helio_PinMode pinMode = Helio_PinMode_Undefined, uint8_t muxChannel = -1);
    HelioPin(const HelioPinData *dataIn);

    explicit operator HelioDigitalPin() const;
    explicit operator HelioAnalogPin() const;

    // Initializes pin to mode settings. Re-entrant.
    void init();
    // De-initializes pin to floating state. Re-entrant.
    inline void deinit() { pinMode(pin, INPUT); }

    void saveToData(HelioPinData *dataOut) const;

    // Attempts to enable the muxer for the pin on its channel number. Returns success boolean. May return early.
    bool tryEnableMuxer();

    inline bool isValid() const { return isValidPin(pin) && mode != Helio_PinMode_Undefined; }
    inline bool isMuxed() const { return isValidChannel(channel); }
    inline bool isInput() const { return mode == Helio_PinMode_Digital_Input_PullUp ||
                                         mode == Helio_PinMode_Digital_Input_PullDown ||
                                         mode == Helio_PinMode_Digital_Input_Floating ||
                                         mode == Helio_PinMode_Analog_Input; }
    inline bool canRead() const { return isValid() && isInput(); }
    inline bool isOutput() const { return mode == Helio_PinMode_Digital_Output_OpenDrain ||
                                          mode == Helio_PinMode_Digital_Output_PushPull ||
                                          mode == Helio_PinMode_Analog_Output; }
    inline bool canWrite() const { return isValid() && isOutput(); }
    inline bool isDigital() const { return mode == Helio_PinMode_Digital_Input_PullUp ||
                                           mode == Helio_PinMode_Digital_Input_PullDown ||
                                           mode == Helio_PinMode_Digital_Input_Floating ||
                                           mode == Helio_PinMode_Digital_Output_OpenDrain ||
                                           mode == Helio_PinMode_Digital_Output_PushPull; }
    inline bool isAnalog() const { return mode == Helio_PinMode_Analog_Input ||
                                          mode == Helio_PinMode_Analog_Output; }
};

// Digital Pin
struct HelioDigitalPin : public HelioPin, public HelioDigitalInputPinInterface, public HelioDigitalOutputPinInterface
{
    bool activeLow;                                         // Active-low trigger state boolean

    HelioDigitalPin();
    HelioDigitalPin(pintype_t pinNumber,                    // Digital pin number (e.g. D0, D1)
                    ard_pinmode_t pinMode,            // Arduino pin mode (e.g. INPUT, OUTPUT, determines activeLow trigger state)
                    uint8_t muxChannel = -1);               // Muxing channel select (else -1 if unused)
    HelioDigitalPin(pintype_t pinNumber,                    // Digital pin number (e.g. D0, D1)
                    Helio_PinMode pinMode,                  // Helioduino pin mode (determines activeLow trigger state)
                    uint8_t muxChannel = -1);               // Muxing channel select (else -1 if unused)
    HelioDigitalPin(pintype_t pinNumber,                    // Digital pin number (e.g. D0, D1)
                    ard_pinmode_t pinMode,            // Arduino pin mode (e.g. INPUT, OUTPUT)
                    bool isActiveLow,                       // Explicit pin active-low trigger state boolean
                    uint8_t muxChannel = -1);               // Muxing channel select (else -1 if unused)
    HelioDigitalPin(pintype_t pinNumber,                    // Digital pin number (e.g. D0, D1)
                    Helio_PinMode pinMode,                  // Helioduino pin mode
                    bool isActiveLow,                       // Explicit pin active-low trigger state boolean
                    uint8_t muxChannel = -1);               // Muxing channel select (else -1 if unused)
    HelioDigitalPin(const HelioPinData *dataIn);

    inline operator HelioPin() const { return HelioPin((int)type, pin, mode, channel); }

    void saveToData(HelioPinData *dataOut) const;

    virtual ard_pinstatus_t digitalRead() override;
    inline bool isActive() { return this->digitalRead() == (activeLow ? LOW : HIGH); }

    virtual void digitalWrite(ard_pinstatus_t status) override;
    inline void activate() { this->digitalWrite((activeLow ? LOW : HIGH)); }
    inline void deactivate() { this->digitalWrite((activeLow ? HIGH : LOW)); }
};

// Analog Pin
struct HelioAnalogPin : public HelioPin, public HelioAnalogInputPinInterface, public HelioAnalogOutputPinInterface
{
    HelioBitResolution bitRes;                              // Bit resolution
#ifdef ESP32
    uint8_t pwmChannel;                                     // PWM channel
#endif
#ifdef ESP_PLATFORM
    float pwmFrequency;                                     // PWM frequency
#endif

    HelioAnalogPin();
    HelioAnalogPin(pintype_t pinNumber,                     // Analog pin number (e.g. A0, A1)
                   ard_pinmode_t pinMode,             // Arduino pin mode (e.g. INPUT, OUTPUT)
                   uint8_t analogBitRes = 0,                // Bit resolution (0 for std DAC/ADC res by mode i/o)
#ifdef ESP32
                   uint8_t pinPWMChannel = 1,               // PWM channel (0 reserved for buzzer)
#endif
#ifdef ESP_PLATFORM
                   float pinPWMFrequency = 1000,            // PWM frequency
#endif
                   uint8_t muxChannel = -1);                // Muxing channel select (else -1 if unused)
    HelioAnalogPin(pintype_t pinNumber,                     // Analog pin number (e.g. A0, A1)
                   Helio_PinMode pinMode,                   // Helioduino pin mode
                   uint8_t analogBitRes = 0,                // Bit resolution (0 for std DAC/ADC res by mode i/o)
#ifdef ESP32
                   uint8_t pinPWMChannel = 1,               // PWM channel (0 reserved for buzzer)
#endif
#ifdef ESP_PLATFORM
                   float pinPWMFrequency = 1000,            // PWM frequency
#endif
                   uint8_t muxChannel = -1);                // Muxing channel select (else -1 if unused)
    HelioAnalogPin(const HelioPinData *dataIn);

    inline operator HelioPin() const { return HelioPin((int)type, pin, mode, channel); }

    void init();

    void saveToData(HelioPinData *dataOut) const;

    float analogRead();
    int analogRead_raw();

    void analogWrite(float val);
    void analogWrite_raw(int val);
};

// Combined Pin Serialization Sub Data
struct HelioPinData : public HelioSubData
{
    pintype_t pin;                                          // Pin number
    Helio_PinMode mode;                                     // Pin mode
    uint8_t channel;                                        // Muxing channel
    union
    {
        struct
        {
            bool activeLow;                                 // Active low trigger state
        } digitalPin;                                       // Digital pin
        struct
        {
            uint8_t bitRes;                                 // Bit resolution
#ifdef ESP32
            uint8_t pwmChannel;                             // PWM channel
#endif
#ifdef ESP_PLATFORM
            float pwmFrequency;                             // PWM frequency
#endif
        } analogPin;
    } dataAs;                                               // Enumeration type union

    HelioPinData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};


// Pin Muxer
// A muxer reduces the number of individual i/o wires needed by instead going through
// a multiplexer that takes in a channel address through a small channel address bus.
// This allows a single pin access to much more than one single circuit/device. Pins that
// are muxed will enable their associated pin muxer (which deactivates all other muxers)
// prior to utilizing them. This muxer will set i/o pins to floating state as appropriate
// during channel switches, with optional usage of chip enable.
class HelioPinMuxer
{
public:
    HelioPinMuxer();
    HelioPinMuxer(HelioPin signalPin,
                  pintype_t *muxChannelPins, uint8_t muxChannelBits,
                  HelioDigitalPin chipEnablePin = HelioDigitalPin());
    HelioPinMuxer(const HelioPinMuxerData *dataIn);

    void saveToData(HelioPinMuxerData *dataOut) const;

    void init();

    inline const HelioPin &getSignalPin() { return _signal; }
    inline const HelioDigitalPin &getChipEnablePin() { return _chipEnable; }
    inline uint8_t getChannelSelectBits() { return _channelBits; }
    inline uint8_t getSelectedChannel() { return _channelSelect; }

protected:
    HelioPin _signal;                                       // Muxed signal pin (only needs/saves pin/mode)
    HelioDigitalPin _chipEnable;                            // Muxing chip enable pin (optional)
    pintype_t _channelPins[5];                              // Channel select bus pins
    uint8_t _channelBits;                                   // Channel select bits (# of bus pins available)
    uint8_t _channelSelect;                                 // Channel select (active channel)

    void selectChannel(uint8_t channelNumber);
    void deselect();

    friend class Helioduino;
    friend class HelioPin;
};

// Pin Muxer Serialization Sub Data
struct HelioPinMuxerData : HelioSubData
{
    HelioPinData signal;                                    // Signal pin data
    HelioPinData chipEnable;                                // Chip enable pin data
    pintype_t channelPins[5];                               // Channel select bus pins
    uint8_t channelBits;                                    // Channel select bits

    HelioPinMuxerData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioPin_H

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
class HelioPinExpander;
struct HelioPinMuxerData;
struct HelioPinExpanderData;

#include "Helioduino.h"
#include "HelioUtils.h"
#include "HelioModules.h"

// Creates Pin from passed Pin data (return ownership transfer - user code *must* delete returned Pin)
extern HelioPin *newPinObjectFromSubData(const HelioPinData *dataIn);


// Pin Base
struct HelioPin {
    enum : signed char { Digital, Analog, Unknown = -1 } type; // Pin type (custom RTTI)
    inline bool isDigitalType() const { return type == Digital; }
    inline bool isAnalogType() const { return type == Analog; }
    inline bool isUnknownType() const { return type <= Unknown; }

    pintype_t pin;                                          // Pin number, else -1/undef or 100+/virtual
    Helio_PinMode mode;                                     // Pin mode setting
    int8_t channel;                                         // Pin muxer/expander channel #, else -127/none

    HelioPin();
    HelioPin(int classType, pintype_t pinNumber = hpin_none, Helio_PinMode pinMode = Helio_PinMode_Undefined, int8_t pinChannel = hpinchnl_none);
    HelioPin(const HelioPinData *dataIn);

    explicit operator HelioDigitalPin() const;
    explicit operator HelioAnalogPin() const;

    // Initializes pin to mode settings. Re-entrant.
    void init();
    // De-initializes pin to floating state. Re-entrant.
    void deinit();

    void saveToData(HelioPinData *dataOut) const;

    // Returns virtual pin number to use for given pin channel # and expander index, else -1/invalid
    static inline pintype_t pinNumberForExpander(int8_t pinChannel, hposi_t expanderIndex) { return isValidChannel(pinChannel) ? (pintype_t)(hpin_virtual + (expanderIndex * 16) + (abs(pinChannel) % 16)) : hpin_none; }
    // Returns expander index to use for given pin number, else -1
    static inline hposi_t expanderForPinNumber(pintype_t pinNumber) { return isValidPin(pinNumber) ? (pinNumber > hpin_virtual ? pinNumber - hpin_virtual : pinNumber) * 16 : -1; }
    // Returns pin channel # to use given a valid muxer channel # (ranged: [0,127]), else -127/none
    static inline int8_t pinChannelForMuxer(uint8_t muxChannel) { return muxChannel != (uint8_t)-1 && muxChannel != (uint8_t)hpinchnl_none ? (int8_t)constrain(muxChannel,0,127) : hpinchnl_none; }
    // Returns pin channel # to use given a valid expander channel # (aka expander pin number, ranged: [0,127]), else -127/none
    static inline int8_t pinChannelForExpander(uint8_t expChannel) { return expChannel != (uint8_t)-1 && expChannel != (uint8_t)hpinchnl_none ? -((int8_t)constrain(expChannel,0,127)) : hpinchnl_none; }

    // Attempts to both select the pin muxer (set address/ready pin state) for the pin on its channel number and activate it (toggle chip enable).
    // Typically called pre-read. Returns success boolean. May return early.
    inline bool selectAndActivatePin() { return enablePin(0); }
    // Attempts to only select the pin muxer (set address/ready pin state) for the pin on its channel number.
    // Typically called pre-write. Returns success boolean. May return early.
    inline bool selectPin() { return enablePin(1); }
    // Attempts to only activate the pin muxer (toggle chip enable).
    // Typically called post-write. Returns success boolean. May return early.
    inline bool activatePin() { return enablePin(2); }

    inline bool isValid() const { return isValidPin(pin) && mode != Helio_PinMode_Undefined; }
    inline bool isVirtual() const { return isValidPin(pin) && pin >= hpin_virtual; }
    inline bool isMuxed() const { return isValidChannel(channel) && channel >= 0; }
    inline bool isExpanded() const { return isValidChannel(channel) && channel < 0; }
    inline bool isInput() const { return mode == Helio_PinMode_Digital_Input ||
                                         mode == Helio_PinMode_Digital_Input_PullUp ||
                                         mode == Helio_PinMode_Digital_Input_PullDown ||
                                         mode == Helio_PinMode_Analog_Input; }
    inline bool canRead() const { return isValid() && isInput(); }
    inline bool isOutput() const { return mode == Helio_PinMode_Digital_Output ||
                                          mode == Helio_PinMode_Digital_Output_PushPull ||
                                          mode == Helio_PinMode_Analog_Output; }
    inline bool canWrite() const { return isValid() && isOutput(); }
    inline bool isDigital() const { return mode == Helio_PinMode_Digital_Input ||
                                           mode == Helio_PinMode_Digital_Input_PullUp ||
                                           mode == Helio_PinMode_Digital_Input_PullDown ||
                                           mode == Helio_PinMode_Digital_Output ||
                                           mode == Helio_PinMode_Digital_Output_PushPull; }
    inline bool isAnalog() const { return mode == Helio_PinMode_Analog_Input ||
                                          mode == Helio_PinMode_Analog_Output; }

protected:
    bool enablePin(int step);
};

// Digital Pin
struct HelioDigitalPin : public HelioPin, public HelioDigitalInputPinInterface, public HelioDigitalOutputPinInterface {
    bool activeLow;                                         // Active-low trigger state boolean

    HelioDigitalPin();
    HelioDigitalPin(pintype_t pinNumber,                    // Digital pin number (e.g. D0, D1)
                    ard_pinmode_t pinMode,                  // Arduino pin mode (e.g. INPUT, OUTPUT, determines activeLow trigger state)
                    int8_t pinChannel = hpinchnl_none);     // Pin muxer/expander channel #, else -127/none
    HelioDigitalPin(pintype_t pinNumber,                    // Digital pin number (e.g. D0, D1)
                    Helio_PinMode pinMode,                  // Hydruino pin mode (determines activeLow trigger state)
                    int8_t pinChannel = hpinchnl_none);     // Pin muxer/expander channel #, else -127/none
    HelioDigitalPin(pintype_t pinNumber,                    // Digital pin number (e.g. D0, D1)
                    ard_pinmode_t pinMode,                  // Arduino pin mode (e.g. INPUT, OUTPUT)
                    bool isActiveLow,                       // Explicit pin active-low trigger state boolean
                    int8_t pinChannel = hpinchnl_none);     // Pin muxer/expander channel #, else -127/none
    HelioDigitalPin(pintype_t pinNumber,                    // Digital pin number (e.g. D0, D1)
                    Helio_PinMode pinMode,                  // Hydruino pin mode
                    bool isActiveLow,                       // Explicit pin active-low trigger state boolean
                    int8_t pinChannel = hpinchnl_none);     // Pin muxer/expander channel #, else -127/none
    HelioDigitalPin(const HelioPinData *dataIn);

    void saveToData(HelioPinData *dataOut) const;

    virtual ard_pinstatus_t digitalRead() override;
    inline bool isActive() { return digitalRead() == (activeLow ? LOW : HIGH); }

    virtual void digitalWrite(ard_pinstatus_t status) override;
    inline void activate() { digitalWrite((activeLow ? LOW : HIGH)); }
    inline void deactivate() { digitalWrite((activeLow ? HIGH : LOW)); }
};

// Analog Pin
struct HelioAnalogPin : public HelioPin, public HelioAnalogInputPinInterface, public HelioAnalogOutputPinInterface {
    BitResolution bitRes;                                   // Bit resolution
#ifdef ESP32
    uint8_t pwmChannel;                                     // PWM channel
#endif
#ifdef ESP_PLATFORM
    float pwmFrequency;                                     // PWM frequency
#endif

    HelioAnalogPin();
    HelioAnalogPin(pintype_t pinNumber,                     // Analog pin number (e.g. A0, A1)
                   ard_pinmode_t pinMode,                   // Arduino pin mode (e.g. INPUT, OUTPUT)
                   uint8_t analogBitRes = 0,                // Bit resolution (0 for std DAC/ADC res by mode i/o)
#ifdef ESP32
                   uint8_t pinPWMChannel = 1,               // PWM channel (0 reserved for buzzer)
#endif
#ifdef ESP_PLATFORM
                   float pinPWMFrequency = 1000,            // PWM frequency
#endif
                   int8_t pinChannel = hpinchnl_none);      // Pin muxer/expander channel #, else -127/none
    HelioAnalogPin(pintype_t pinNumber,                     // Analog pin number (e.g. A0, A1)
                   Helio_PinMode pinMode,                   // Hydruino pin mode
                   uint8_t analogBitRes = 0,                // Bit resolution (0 for std DAC/ADC res by mode i/o)
#ifdef ESP32
                   uint8_t pinPWMChannel = 1,               // PWM channel (0 reserved for buzzer)
#endif
#ifdef ESP_PLATFORM
                   float pinPWMFrequency = 1000,            // PWM frequency
#endif
                   int8_t pinChannel = hpinchnl_none);      // Pin muxer/expander channel #, else -127/none
    HelioAnalogPin(const HelioPinData *dataIn);

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
    int8_t channel;                                         // Pin muxer/expander channel #
    union {
        struct {
            bool activeLow;                                 // Active low trigger state
        } digitalPin;                                       // Digital pin
        struct {
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
// A muxer reduces the number of individual wire lines needed by instead going through
// a multiplexer that takes in a channel address through a small channel address bus.
// This allows a single pin access to much more than one single circuit/device. Pins that
// are multiplexed (or muxed) will enable their associated pin muxer (which deactivates
// all other muxers) prior to utilizing them. This muxer will set i/o pins to floating
// state as appropriate during channel switches, with optional usage of a chip enable.
class HelioPinMuxer {
public:
    HelioPinMuxer();
    HelioPinMuxer(HelioPin signalPin,
                  pintype_t *muxChannelPins, int8_t muxChannelBits,
                  HelioDigitalPin chipEnablePin = HelioDigitalPin());
    HelioPinMuxer(const HelioPinMuxerData *dataIn);

    void saveToData(HelioPinMuxerData *dataOut) const;

    void init();

    inline const HelioPin &getSignalPin() const { return _signal; }
    inline const HelioDigitalPin &getChipEnablePin() const { return _chipEnable; }
    inline uint8_t getChannelSelectBits() const { return _channelBits; }
    inline uint8_t getSelectedChannel() const { return _channelSelect; }

protected:
    HelioPin _signal;                                       // Muxed signal pin (unused channel #)
    HelioDigitalPin _chipEnable;                            // Muxing chip enable pin (optional)
    pintype_t _channelPins[5];                              // Channel select bus pins
    uint8_t _channelBits;                                   // Channel select bits (# of bus pins available)
    uint8_t _channelSelect;                                 // Channel select (active channel)

    void selectChannel(uint8_t channelNumber);
    void setIsActive(bool isActive);
    inline void activate() { setIsActive(true); }
    inline void deactivate() { setIsActive(false); } 

    friend class HelioPinHandlers;
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


// Pin Expander
// Instead of having an address bus, a pin expander expands the pins available
// by providing a set of virtualized pins that can be accessed over i2c. This
// class uses IoAbstractionRef objects for tcMenu-compatible pinset expansion.
// In case of analog pins, instead assume ref object is an AnalogDevice instance.
// Typically only one pin out direction is supported by devices, specified by passed
// signalPin. Signal pin #'s are virtualized and split into groups of 16 to quickly
// be able to identify expander index. The passed pin number is changed to the
// nearest previous divisible-by-16 offset from 100 (e.g. 100, 116, 132, etc.),
// and also stores the number of channel select bits in the unused pin channel #.
class HelioPinExpander {
public:
    HelioPinExpander();
    HelioPinExpander(HelioPin signalPin, IoAbstractionRef ioRef, uint8_t channelBits);
    HelioPinExpander(const HelioPinExpanderData *dataIn, IoAbstractionRef ioRef);

    void saveToData(HelioPinExpanderData *dataOut) const;

    inline const HelioPin &getSignalPin() const { return _signal; }
    inline IoAbstractionRef getIoAbstraction() { return _ioRef; }
    inline uint8_t getChannelSelectBits() const { return (uint8_t)_signal.channel; }

protected:
    HelioPin _signal;                                       // Expanded signal pin (pin = 100+ldiv16, channel # = bits)
    IoAbstractionRef _ioRef;                                // IoAbstraction instance

    bool syncChannel();

    friend class HelioPinHandlers;
    friend class HelioPin;
};

// Pin Expander Serialization Sub Data
struct HelioPinExpanderData : HelioSubData
{
    HelioPinData signal;                                    // Signal pin data
    uint8_t channelBits;                                    // Channel select bits

    HelioPinExpanderData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioPin_H

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Utilities
*/

#ifndef HelioUtils_H
#define HelioUtils_H

struct HelioBitResolution;
template<typename RTCType> class HelioRTCWrapper;
#ifdef HELIO_USE_MULTITASKING
template<typename ParameterType, int Slots> class SignalFireTask;
template<class ObjectType, typename ParameterType> class MethodSlotCallTask;
class ActuatorTimedEnableTask;
#endif

#include "Helioduino.h"
#include "HelioObject.h"
#ifdef HELIO_USE_MULTITASKING
#include "BasicInterruptAbstraction.h"
#endif

// Simple class for describing an analog bit resolution.
// This class is mainly used to calculate analog pin range boundary values.
struct HelioBitResolution {
    uint8_t bits;                                           // Bit resolution (# of bits)
    int maxVal;                                             // Maximum value (2 ^ (# of bits))

    HelioBitResolution(uint8_t bitResolution);              // Bit resolution (# of bits)

    // Transforms value from raw integer (or initial) value into normalized raw (or transformed) value.
    inline float transform(int intValue) const { return constrain(intValue / (float)maxVal, 0.0f, 1.0f); }

    // Inverse transforms value from normalized raw (or transformed) value back into raw integer (or initial) value.
    inline int inverseTransform(float rawValue) const { return constrain((int)((float)maxVal * rawValue), 0, maxVal); }
};


// Simple wrapper class for dealing with RTC modules.
// This class is mainly used to abstract which RTC module is used.
template<typename RTCType>
class HelioRTCWrapper : public HelioRTCInterface {
public:
    virtual bool begin(TwoWire *wireInstance) override { return _rtc.begin(wireInstance); }
    virtual void adjust(const DateTime &dt) override { _rtc.adjust(dt); }
    virtual bool lostPower(void) override { return _rtc.lostPower(); }
    virtual DateTime now() override { return _rtc.now(); }
protected:
    RTCType _rtc;
};

// Specialization for older DS1307 that doesn't have lost power tracking.
template<>
class HelioRTCWrapper<RTC_DS1307> : public HelioRTCInterface {
public:
    virtual bool begin(TwoWire *wireInstance) override;
    virtual void adjust(const DateTime &dt) override;
    virtual bool lostPower(void) override;
    virtual DateTime now() override;
protected:
    RTC_DS1307 _rtc;
};

// Scheduling

#ifdef HELIO_USE_MULTITASKING

// Standard interrupt abstraction
extern BasicArduinoInterruptAbstraction interruptImpl;


// This will schedule an actuator to enable on the next TaskManagerIO runloop using the given intensity and enable time millis.
// Actuator is captured. Returns taskId or TASKMGR_INVALIDID on error.
taskid_t scheduleActuatorTimedEnableOnce(SharedPtr<HelioActuator> actuator, float intensity, time_t duration);

// This will schedule an actuator to enable on the next TaskManagerIO runloop using the given enable time millis.
// Actuator is captured. Returns taskId or TASKMGR_INVALIDID on error.
taskid_t scheduleActuatorTimedEnableOnce(SharedPtr<HelioActuator> actuator, time_t duration);

// This will schedule a signal's fire method on the next TaskManagerIO runloop using the given call/fire parameter.
// Object is captured, if not nullptr. Returns taskId or TASKMGR_INVALIDID on error.
template<typename ParameterType, int Slots> taskid_t scheduleSignalFireOnce(SharedPtr<HelioObjInterface> object, Signal<ParameterType,Slots> &signal, ParameterType fireParam);

// This will schedule a signal's fire method on the next TaskManagerIO runloop using the given call/fire parameter, w/o capturing object.
// Returns taskId or TASKMGR_INVALIDID on error.
template<typename ParameterType, int Slots> taskid_t scheduleSignalFireOnce(Signal<ParameterType,Slots> &signal, ParameterType fireParam);

// This will schedule an object's method to be called on the next TaskManagerIO runloop using the given method slot and call parameter.
// Object is captured. Returns taskId or TASKMGR_INVALIDID on error.
template<class ObjectType, typename ParameterType> taskid_t scheduleObjectMethodCallOnce(SharedPtr<ObjectType> object, void (ObjectType::*method)(ParameterType), ParameterType callParam);

// This will schedule an object's method to be called on the next TaskManagerIO runloop using the given method slot and call parameter, w/o capturing object.
// Object is captured. Returns taskId or TASKMGR_INVALIDID on error.
template<class ObjectType, typename ParameterType> taskid_t scheduleObjectMethodCallOnce(ObjectType *object, void (ObjectType::*method)(ParameterType), ParameterType callParam);

// This will schedule an object's method to be called on the next TaskManagerIO runloop using the taskId that was created.
// Object is captured. Returns taskId or TASKMGR_INVALIDID on error.
template<class ObjectType> taskid_t scheduleObjectMethodCallWithTaskIdOnce(SharedPtr<ObjectType> object, void (ObjectType::*method)(taskid_t));

// This will schedule an object's method to be called on the next TaskManagerIO runloop using the taskId that was created, w/o capturing object.
// Returns taskId or TASKMGR_INVALIDID on error.
template<class ObjectType> taskid_t scheduleObjectMethodCallWithTaskIdOnce(ObjectType *object, void (ObjectType::*method)(taskid_t));


// Signal Fire Task
// This class holds onto the passed signal and parameter to pass it along to the signal's
// fire method upon task execution.
template<typename ParameterType, int Slots>
class SignalFireTask : public Executable {
public:
    taskid_t taskId;
    SignalFireTask(SharedPtr<HelioObjInterface> object,
                   Signal<ParameterType,Slots> &signal,
                   ParameterType &param)
        : taskId(TASKMGR_INVALIDID), _object(object), _signal(&signal), _param(param) { ; }

    virtual void exec() override { _signal->fire(_param); }
private:
    SharedPtr<HelioObjInterface> _object;
    Signal<ParameterType, Slots> *_signal;
    ParameterType _param;
};


// Method Slot Task
// This class holds onto a MethodSlot to call once executed.
template<class ObjectType, typename ParameterType>
class MethodSlotCallTask : public Executable {
public:
    typedef void (ObjectType::*FunctPtr)(ParameterType);
    taskid_t taskId;

    MethodSlotCallTask(SharedPtr<ObjectType> object, FunctPtr method, ParameterType callParam) : taskId(TASKMGR_INVALIDID), _object(object), _methodSlot(object.get(), method), _callParam(callParam) { ; }
    MethodSlotCallTask(ObjectType *object, FunctPtr method, ParameterType callParam) : taskId(TASKMGR_INVALIDID), _object(nullptr), _methodSlot(object, method), _callParam(callParam) { ; }

    virtual void exec() override { _methodSlot(_callParam); }
private:
    SharedPtr<ObjectType> _object;
    MethodSlot<ObjectType,ParameterType> _methodSlot;
    ParameterType _callParam;

    friend taskid_t scheduleObjectMethodCallWithTaskIdOnce<ObjectType>(SharedPtr<ObjectType> object, void (ObjectType::*method)(taskid_t));
    friend taskid_t scheduleObjectMethodCallWithTaskIdOnce<ObjectType>(ObjectType *object, void (ObjectType::*method)(taskid_t));
};


// Actuator Precise Timed Enable Task
// This task will enable an actuator for a period of time finely, and then deactivate it.
class ActuatorTimedEnableTask : public Executable {
public:
    taskid_t taskId;
    ActuatorTimedEnableTask(SharedPtr<HelioActuator> actuator,
                            float intensity,
                            millis_t duration);

    virtual void exec() override;
private:
    SharedPtr<HelioActuator> _actuator;
    float _intensity;
    millis_t _duration;
};


#endif // /ifdef HELIO_USE_MULTITASKING

// Assertions

#ifdef HELIO_USE_DEBUG_ASSERTIONS

// This softly asserts on a failed condition, sending message out to Serial output (if debugging enabled) and/or disk-based logging (if enabled), and then finally continuing program execution.
// See HELIO_SOFT_ASSERT() macro for usage.
extern void softAssert(bool cond, String msg, const char *file, const char *func, int line);

// This hard asserts on a failed condition, sending message out to Serial output (if debugging enabled) and/or disk-based logging (if enabled), then yielding (to allow comms), and then finally aborting program execution.
// See HELIO_HARD_ASSERT() macro for usage.
extern void hardAssert(bool cond, String msg, const char *file, const char *func, int line);

#endif // /ifdef HELIO_USE_DEBUG_ASSERTIONS

// Helpers & Misc

// Returns the active Helio instance. Not guaranteed to be non-null.
inline Helioduino *getHelioInstance();
// Returns the active scheduler instance. Not guaranteed to be non-null.
inline HelioScheduler *getSchedulerInstance();
// Returns the active logger instance. Not guaranteed to be non-null.
inline HelioLogger *getLoggerInstance();
// Returns the active publisher instance. Not guaranteed to be non-null.
inline HelioPublisher *getPublisherInstance();
#ifdef HELIO_USE_GUI
// Returns the active UI instance. Not guaranteed to be non-null.
inline HelioUIInterface *getUIInstance();
#endif

// Publishes latest data from sensor to Publisher output.
extern void publishData(HelioSensor *sensor);

// Returns current time, with proper time zone offset based on active Helio instance.
inline DateTime getCurrentTime();
// Returns the UTC seconds time that today started, accounting for time zone offset based on active Helio instance.
inline time_t getCurrentDayStartTime();
// Sets the global current time of the RTC, returning if update was successful, and additionally calls appropriate system time updaters.
extern bool setCurrentTime(DateTime currTime);

// Returns a proper filename for a storage monitoring file (log, data, etc) that uses YYMMDD as filename.
extern String getYYMMDDFilename(String prefix, String ext);
// Returns a proper filename for a storage library data file that uses ## as filename.
extern String getNNFilename(String prefix, unsigned int value, String ext);

// Creates intermediate folders given a filename. Currently only supports a single folder depth.
extern void createDirectoryFor(SDClass *sd, String filename);

// Computes a hash for a string using a fast and efficient (read as: good enough for our use) hashing algorithm.
extern hkey_t stringHash(String string);

// Returns properly formatted address "0xADDR" (size depending on void* size)
extern String addressToString(uintptr_t addr);

// Returns a string from char array with an exact max length.
// Null array or invalid length will abort function before encoding occurs, returning "null".
extern String charsToString(const char *charsIn, size_t length);
// Returns a string formatted to deal with variable time spans.
extern String timeSpanToString(const TimeSpan &span);
// Returns a string formatted to value and unit for dealing with measurements as value/units pair.
extern String measurementToString(float value, Helio_UnitsType units, unsigned int additionalDecPlaces = 0);
// Returns a string formatted to value and unit for dealing with measurements.
inline String measurementToString(const HelioSingleMeasurement &measurement, unsigned int additionalDecPlaces = 0) { return measurementToString(measurement.value, measurement.units, additionalDecPlaces); }

// Encodes a T-typed array to a comma-separated string.
// Null array or invalid length will abort function before encoding occurs, returning "null".
template<typename T> String commaStringFromArray(const T *arrayIn, size_t length);
// Decodes a comma-separated string back to a T-typed array.
// Last value read is repeated on through to last element, with no commas being treated as single value being applied to all elements.
// Empty string, string value of "null", null array, or invalid length will abort function before decoding.
template<typename T> void commaStringToArray(String stringIn, T *arrayOut, size_t length);
// Decodes a comma-separated JSON variant, if not null, object, or array, back to a T-typed array.
// Acts as a redirect for JSON variants so that they receive the additional checks before being converted to string.
template<typename T> void commaStringToArray(JsonVariantConst &variantIn, T *arrayOut, size_t length);

// Encodes a byte array to hexadecimal string.
extern String hexStringFromBytes(const uint8_t *bytesIn, size_t length);
// Decodes a hexadecimal string back to a byte array.
extern void hexStringToBytes(String stringIn, uint8_t *bytesOut, size_t length);
// Decodes a hexadecimal JSON variant, if not null, object, or array, back to a byte array.
extern void hexStringToBytes(JsonVariantConst &variantIn, uint8_t *bytesOut, size_t length);

// Returns # of occurrences of character in string.
int occurrencesInString(String string, char singleChar);
// Returns # of occurrences of substring in string.
int occurrencesInString(String string, String subString);
// Returns # of occurrences of character in string, ignoring case.
int occurrencesInStringIgnoreCase(String string, char singleChar);
// Returns # of occurrences of substring in string, ignoring case.
int occurrencesInStringIgnoreCase(String string, String subString);

// Returns whenever all elements of an array are equal to the specified value, or not.
template<typename T> bool arrayElementsEqual(const T *arrayIn, size_t length, T value);

// Similar to the standard map function, but does it on any type.
template<typename T> inline T mapValue(T value, T inMin, T inMax, T outMin, T outMax) { return ((value - inMin) * ((outMax - outMin) / (inMax - inMin))) + outMin; }

// Returns the amount of space between the stack and heap (ie free space left), else -1 if undeterminable.
extern unsigned int freeMemory();

// This delays a finely timed amount, with spin loop nearer to end. Used in finely timed dispensers.
extern void delayFine(millis_t time);

// This will query the active RTC sync device for the current time.
extern time_t rtcNow();

// This will return the time in unixtime (secs since 1970). Uses rtc if available, otherwise time since turned on.
inline time_t unixNow() { return rtcNow() ?: now() + SECONDS_FROM_1970_TO_2000; } // rtcNow returns 0 if not set

// This will return a non-zero millis time value, so that 0 time values can be reserved for other use.
inline millis_t nzMillis() { return millis() ?: 1; }

// This will handle interrupts for task manager.
extern void handleInterrupt(pintype_t pin);

// This is used to force debug statements through to serial monitor.
inline void flushYield() {
    #if defined(HELIO_ENABLE_DEBUG_OUTPUT) && HELIO_SYS_DEBUGOUT_FLUSH_YIELD
        Serial.flush(); yield();
    #else
        return;
    #endif
}

// Units & Conversion

// Tries to convert value from one unit to another (if supported), returning conversion success flag.
// Convert param used in certain unit conversions as external additional value (e.g. voltage for power/current conversion).
// This is the main conversion function that all others wrap around.
extern bool tryConvertUnits(float valueIn, Helio_UnitsType unitsIn, float *valueOut, Helio_UnitsType unitsOut, float convertParam = FLT_UNDEF);

// Attempts to convert value in-place from one unit to another, and if successful then assigns value back overtop of itself.
// Convert param used in certain unit conversions. Returns conversion success flag.
extern bool convertUnits(float *valueInOut, Helio_UnitsType *unitsInOut, Helio_UnitsType outUnits, float convertParam = FLT_UNDEF);
// Attempts to convert value from one unit to another, and if successful then assigns value, and optionally units, to output.
// Convert param used in certain unit conversions. Returns conversion success flag.
extern bool convertUnits(float valueIn, float *valueOut, Helio_UnitsType unitsIn, Helio_UnitsType outUnits, Helio_UnitsType *unitsOut = nullptr, float convertParam = FLT_UNDEF);
// Attempts to convert measurement in-place from one unit to another, and if successful then assigns value and units back overtop of itself.
// Convert param used in certain unit conversions. Returns conversion success flag.
inline bool convertUnits(HelioSingleMeasurement *measureInOut, Helio_UnitsType outUnits, float convertParam = FLT_UNDEF) { return convertUnits(&measureInOut->value, &measureInOut->units, outUnits, convertParam); }
// Attemps to convert measurement from one unit to another, and if successful then assigns value and units to output measurement.
// Convert param used in certain unit conversions. Returns conversion success flag.
inline bool convertUnits(const HelioSingleMeasurement *measureIn, HelioSingleMeasurement *measureOut, Helio_UnitsType outUnits, float convertParam = FLT_UNDEF) { return convertUnits(measureIn->value, &measureOut->value, measureIn->units, outUnits, &measureOut->units, convertParam); }

// Returns the base units from a rate unit (e.g. mm/s -> mm).
extern Helio_UnitsType baseUnitsFromRate(Helio_UnitsType units);

// Returns default temperature units to use based on measureMode (if undefined then uses active Helio instance's measurement mode, else default mode).
extern Helio_UnitsType defaultTemperatureUnits(Helio_MeasurementMode measureMode = Helio_MeasurementMode_Undefined);
// Returns default distance units to use based on measureMode (if undefined then uses active Helio instance's measurement mode, else default mode).
extern Helio_UnitsType defaultDistanceUnits(Helio_MeasurementMode measureMode = Helio_MeasurementMode_Undefined);
// Returns default speed units to use based on measureMode (if undefined then uses active Helio instance's measurement mode, else default mode).
extern Helio_UnitsType defaultSpeedUnits(Helio_MeasurementMode measureMode = Helio_MeasurementMode_Undefined);
// Returns default power units to use based on measureMode (if undefined then uses active Hydruino instance's measurement mode, else default mode).
extern Helio_UnitsType defaultPowerUnits(Helio_MeasurementMode measureMode = Helio_MeasurementMode_Undefined);
// Returns default decimal places rounded to based on measureMode (if undefined then uses active Helio instance's measurement mode, else default mode).
extern int defaultDecimalPlaces(Helio_MeasurementMode measureMode = Helio_MeasurementMode_Undefined);

// Rounds value according to default decimal places rounding, as typically used for data export, with optional additional decimal places.
inline float roundForExport(float value, unsigned int additionalDecPlaces = 0) { return roundToDecimalPlaces(value, defaultDecimalPlaces() + additionalDecPlaces); }
// Rounds value according to default decimal places rounding, as typically used for data export, to string with optional additional decimal places.
inline String roundToString(float value, unsigned int additionalDecPlaces = 0) { return String(roundToDecimalPlaces(value, defaultDecimalPlaces() + additionalDecPlaces), defaultDecimalPlaces() + additionalDecPlaces); }

// Linkages & Filtering

// Returns linkages list filtered down to just actuators.
template<size_t N = HELIO_OBJ_LINKSFILTER_DEFSIZE> Vector<HelioObject *, N> linksFilterActuators(Pair<uint8_t, Pair<HelioObject *, int8_t> *> links);

// Returns linkages list filtered down to just actuators of a certain type that operate on a specific panel.
template<size_t N = HELIO_OBJ_LINKSFILTER_DEFSIZE> Vector<HelioObject *, N> linksFilterActuatorsByPanelAndType(Pair<uint8_t, Pair<HelioObject *, int8_t> *> links, HelioPanel *srcPanel, Helio_ActuatorType actuatorType);

// Returns the # of actuators of a certain type that operate on a specific panel.
int linksCountActuatorsByPanelAndType(Pair<uint8_t, Pair<HelioObject *, int8_t> *> links, HelioPanel *srcPanel, Helio_ActuatorType actuatorType);

// Recombines filtered object list back into SharedPtr actuator list.
template<size_t N> void linksResolveActuatorsByType(Vector<HelioObject *, N> &actuatorsIn, Vector<HelioActuatorAttachment, N> &activationsOut, Helio_ActuatorType actuatorType);
// Recombines filtered object list back into SharedPtr actuator list paired with rate value.
template<size_t N> void linksResolveActuatorsWithRateByType(Vector<HelioObject *, N> &actuatorsIn, HelioObjInterface *parent, float rateMultiplier, Vector<HelioActuatorAttachment, N> &activationsOut, Helio_ActuatorType actuatorType);

// Pins & Checks

// Checks to see if the pin is an analog input pin.
extern bool checkPinIsAnalogInput(pintype_t pin);
// Checks to see if the pin is an analog output pin.
extern bool checkPinIsAnalogOutput(pintype_t pin);
// Checks to see if the pin is a standard digital (non-analog) pin.
inline bool checkPinIsDigital(pintype_t pin);
// Checks to see if the pin can produce a digital PWM output signal.
inline bool checkPinIsPWMOutput(pintype_t pin);
// Checks to see if the pin can be set up with an ISR to handle digital level changes.
inline bool checkPinCanInterrupt(pintype_t pin);

// Enums & Conversions

// Converts from pin mode enum to string, with optional exclude for special types (instead returning "").
extern String pinModeToString(Helio_PinMode pinMode, bool excludeSpecial = false);
// Converts back to pin mode enum from string.
extern Helio_PinMode pinModeFromString(String pinModeStr);

// Converts from system mode enum to string, with optional exclude for special types (instead returning "").
extern String systemModeToString(Helio_SystemMode systemMode, bool excludeSpecial = false);
// Converts back to system mode enum from string.
extern Helio_SystemMode systemModeFromString(String systemModeStr);

// Converts from measurement mode enum to string, with optional exclude for special types (instead returning "").
extern String measurementModeToString(Helio_MeasurementMode measurementMode, bool excludeSpecial = false);
// Converts back to measurement mode enum from string.
extern Helio_MeasurementMode measurementModeFromString(String measurementModeStr);

// Converts from display output mode enum to string, with optional exclude for special types (instead returning "").
extern String displayOutputModeToString(Helio_DisplayOutputMode displayOutMode, bool excludeSpecial = false);
// Converts back to display output mode enum from string.
extern Helio_DisplayOutputMode displayOutputModeFromString(String displayOutModeStr);

// Converts from control input mode enum to string, with optional exclude for special types (instead returning "").
extern String controlInputModeToString(Helio_ControlInputMode controlInMode, bool excludeSpecial = false);
// Converts back to control input mode enum from string.
extern Helio_ControlInputMode controlInputModeFromString(String controlInModeStr);

// Returns true for actuators that operate activation handles serially (as opposed to in-parallel) as derived from enabled mode enumeration.
inline bool getActuatorIsSerialFromMode(Helio_EnableMode actuatorMode) { return actuatorMode >= Helio_EnableMode_Serial; }

// Converts from actuator type enum to string, with optional exclude for special types (instead returning "").
extern String actuatorTypeToString(Helio_ActuatorType actuatorType, bool excludeSpecial = false);
// Converts back to actuator type enum from string.
extern Helio_ActuatorType actuatorTypeFromString(String actuatorTypeStr);

// Converts from sensor type enum to string, with optional exclude for special types (instead returning "").
extern String sensorTypeToString(Helio_SensorType sensorType, bool excludeSpecial = false);
// Converts back to sensor type enum from string.
extern Helio_SensorType sensorTypeFromString(String sensorTypeStr);

// Returns axis count as derived from panel type enumeration.
extern hposi_t getPanelAxisCountFromType(Helio_PanelType panelType);

// Converts from fluid panel enum to string, with optional exclude for special types (instead returning "").
extern String panelTypeToString(Helio_PanelType panelType, bool excludeSpecial = false);
// Converts back to fluid panel enum from string.
extern Helio_PanelType panelTypeFromString(String panelTypeStr);

// Returns nominal rail voltage as derived from rail type enumeration.
extern float getRailVoltageFromType(Helio_RailType railType);

// Converts from power rail enum to string, with optional exclude for special types (instead returning "").
extern String railTypeToString(Helio_RailType railType, bool excludeSpecial = false);
// Converts back to power rail enum from string.
extern Helio_RailType railTypeFromString(String railTypeStr);

// Converts from units category enum to string, with optional exclude for special types (instead returning "").
extern String unitsCategoryToString(Helio_UnitsCategory unitsCategory, bool excludeSpecial = false);
// Converts back to units category enum from string.
extern Helio_UnitsCategory unitsCategoryFromString(String unitsCategoryStr);

// Converts from units type enum to symbol string, with optional exclude for special types (instead returning "").
extern String unitsTypeToSymbol(Helio_UnitsType unitsType, bool excludeSpecial = false);
// Converts back to units type enum from symbol.
extern Helio_UnitsType unitsTypeFromSymbol(String unitsSymbolStr);

// Converts from position index to string, with optional exclude for special types (instead returning "").
extern String positionIndexToString(hposi_t positionIndex, bool excludeSpecial = false);
// Converts back to position index from string.
extern hposi_t positionIndexFromString(String positionIndexStr);

// Converts from boolean value to triggered/not-triggered trigger state.
inline Helio_TriggerState triggerStateFromBool(bool value) { return value ? Helio_TriggerState_Triggered : Helio_TriggerState_NotTriggered; }
// Converts from triggered/not-triggered trigger state back into boolean value.
inline bool triggerStateToBool(Helio_TriggerState state) { return state == Helio_TriggerState_Triggered; }

// Explicit Specializations

template<> String commaStringFromArray<float>(const float *arrayIn, size_t length);
template<> String commaStringFromArray<double>(const double *arrayIn, size_t length);
template<> void commaStringToArray<float>(String stringIn, float *arrayOut, size_t length);
template<> void commaStringToArray<double>(String stringIn, double *arrayOut, size_t length);
template<> bool arrayElementsEqual<float>(const float *arrayIn, size_t length, float value);
template<> bool arrayElementsEqual<double>(const double *arrayIn, size_t length, double value);

#endif // /ifndef HelioUtils_H

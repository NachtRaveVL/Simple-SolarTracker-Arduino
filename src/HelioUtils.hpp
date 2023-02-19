/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Utilities
*/

#ifndef HelioUtils_HPP
#define HelioUtils_HPP

#include "Helioduino.h"

inline HelioSingleMeasurement HelioSingleMeasurement::asUnits(Helio_UnitsType outUnits, float convertParam) const
{
    HelioSingleMeasurement out(*this);
    convertUnits(&out, outUnits, convertParam);
    return out;
}


#ifdef HELIO_USE_MULTITASKING

template<typename ParameterType, int Slots>
taskid_t scheduleSignalFireOnce(SharedPtr<HelioObjInterface> object, Signal<ParameterType,Slots> &signal, ParameterType fireParam)
{
    SignalFireTask<ParameterType,Slots> *fireTask = object ? new SignalFireTask<ParameterType,Slots>(object, signal, fireParam) : nullptr;
    HELIO_SOFT_ASSERT(!object || fireTask, SFP(HStr_Err_AllocationFailure));
    taskid_t retVal = fireTask ? taskManager.scheduleOnce(0, fireTask, TIME_MILLIS, true) : TASKMGR_INVALIDID;
    return (fireTask ? (fireTask->taskId = retVal) : retVal);
}

template<typename ParameterType, int Slots>
taskid_t scheduleSignalFireOnce(Signal<ParameterType,Slots> &signal, ParameterType fireParam)
{
    SignalFireTask<ParameterType,Slots> *fireTask = new SignalFireTask<ParameterType,Slots>(nullptr, signal, fireParam);
    HELIO_SOFT_ASSERT(fireTask, SFP(HStr_Err_AllocationFailure));
    taskid_t retVal = fireTask ? taskManager.scheduleOnce(0, fireTask, TIME_MILLIS, true) : TASKMGR_INVALIDID;
    return (fireTask ? (fireTask->taskId = retVal) : retVal);
}

template<class ObjectType, typename ParameterType>
taskid_t scheduleObjectMethodCallOnce(SharedPtr<ObjectType> object, void (ObjectType::*method)(ParameterType), ParameterType callParam)
{
    MethodSlotCallTask<ObjectType,ParameterType> *callTask = object ? new MethodSlotCallTask<ObjectType,ParameterType>(object, method, callParam) : nullptr;
    HELIO_SOFT_ASSERT(!object || callTask, SFP(HStr_Err_AllocationFailure));
    taskid_t retVal = callTask ? taskManager.scheduleOnce(0, callTask, TIME_MILLIS, true) : TASKMGR_INVALIDID;
    return (callTask ? (callTask->taskId = retVal) : retVal);
}

template<class ObjectType, typename ParameterType>
taskid_t scheduleObjectMethodCallOnce(ObjectType *object, void (ObjectType::*method)(ParameterType), ParameterType callParam)
{
    MethodSlotCallTask<ObjectType,ParameterType> *callTask = object ? new MethodSlotCallTask<ObjectType,ParameterType>(object, method, callParam) : nullptr;
    HELIO_SOFT_ASSERT(!object || callTask, SFP(HStr_Err_AllocationFailure));
    taskid_t retVal = callTask ? taskManager.scheduleOnce(0, callTask, TIME_MILLIS, true) : TASKMGR_INVALIDID;
    return (callTask ? (callTask->taskId = retVal) : retVal);
}

template<class ObjectType>
taskid_t scheduleObjectMethodCallWithTaskIdOnce(SharedPtr<ObjectType> object, void (ObjectType::*method)(taskid_t))
{
    MethodSlotCallTask<ObjectType,taskid_t> *callTask = object ? new MethodSlotCallTask<ObjectType,taskid_t>(object, method, (taskid_t)0) : nullptr;
    HELIO_SOFT_ASSERT(!object || callTask, SFP(HStr_Err_AllocationFailure));
    taskid_t retVal = callTask ? taskManager.scheduleOnce(0, callTask, TIME_MILLIS, true) : TASKMGR_INVALIDID;
    return (callTask ? (callTask->taskId = (callTask->_callParam = retVal)) : retVal);
}

template<class ObjectType>
taskid_t scheduleObjectMethodCallWithTaskIdOnce(ObjectType *object, void (ObjectType::*method)(taskid_t))
{
    MethodSlotCallTask<ObjectType,taskid_t> *callTask = object ? new MethodSlotCallTask<ObjectType,taskid_t>(object, method, (taskid_t)0) : nullptr;
    HELIO_SOFT_ASSERT(!object || callTask, SFP(HStr_Err_AllocationFailure));
    taskid_t retVal = callTask ? taskManager.scheduleOnce(0, callTask, TIME_MILLIS, true) : TASKMGR_INVALIDID;
    return (callTask ? (callTask->taskId = (callTask->_callParam = retVal)) : retVal);
}

#endif // /ifdef HELIO_USE_MULTITASKING


template<typename T>
String commaStringFromArray(const T *arrayIn, size_t length)
{
    if (!arrayIn || !length) { return String(SFP(HStr_null)); }
    String retVal; retVal.reserve(length << 1);
    for (size_t index = 0; index < length; ++index) {
        if (retVal.length()) { retVal.concat(','); }
        retVal += String(arrayIn[index]);
    }
    return retVal.length() ? retVal : String(SFP(HStr_null));
}

template<typename T>
void commaStringToArray(String stringIn, T *arrayOut, size_t length)
{
    if (!stringIn.length() || !length || stringIn.equalsIgnoreCase(SFP(HStr_null))) { return; }
    int lastSepPos = -1;
    for (size_t index = 0; index < length; ++index) {
        int nextSepPos = stringIn.indexOf(',', lastSepPos+1);
        if (nextSepPos == -1) { nextSepPos = stringIn.length(); }
        String subString = stringIn.substring(lastSepPos+1, nextSepPos);
        if (nextSepPos < stringIn.length()) { lastSepPos = nextSepPos; }

        arrayOut[index] = static_cast<T>(subString.toInt());
    }
}

template<typename T>
void commaStringToArray(JsonVariantConst &variantIn, T *arrayOut, size_t length)
{
    if (variantIn.isNull() || variantIn.is<JsonObjectConst>() || variantIn.is<JsonArrayConst>()) { return; }
    commaStringToArray<T>(variantIn.as<String>(), arrayOut, length);
}

template<typename T>
bool arrayElementsEqual(const T *arrayIn, size_t length, T value)
{
    for (size_t index = 0; index < length; ++index) {
        if (!(arrayIn[index] == value)) {
            return false;
        }
    }
    return true;
}


inline bool convertUnits(float *valueInOut, Helio_UnitsType *unitsInOut, Helio_UnitsType outUnits, float convertParam)
{
    if (tryConvertUnits(*valueInOut, *unitsInOut, valueInOut, outUnits, convertParam)) {
        *unitsInOut = outUnits;
        return true;
    }
    return false;
}

inline bool convertUnits(float valueIn, float *valueOut, Helio_UnitsType unitsIn, Helio_UnitsType outUnits, Helio_UnitsType *unitsOut, float convertParam)
{
    if (tryConvertUnits(valueIn, unitsIn, valueOut, outUnits, convertParam)) {
        if (unitsOut) { *unitsOut = outUnits; }
        return true;
    }
    return false;
}

inline bool convertUnits(HelioSingleMeasurement *measureInOut, Helio_UnitsType outUnits, float convertParam)
{
    return convertUnits(&measureInOut->value, &measureInOut->units, outUnits, convertParam);
}

inline bool convertUnits(const HelioSingleMeasurement *measureIn, HelioSingleMeasurement *measureOut, Helio_UnitsType outUnits, float convertParam)
{
    return convertUnits(measureIn->value, &measureOut->value, measureIn->units, outUnits, &measureOut->units, convertParam);
}


template<size_t N = HELIO_DEFAULT_MAXSIZE>
Vector<HelioObject *, N> linksFilterActuators(Pair<uint8_t, Pair<HelioObject *, int8_t> *> links)
{
    Vector<HelioObject *, N> retVal;

    for (hposi_t linksIndex = 0; linksIndex < links.first && links.second[linksIndex].first; ++linksIndex) {
        if (links.second[linksIndex].first->isActuatorType()) {
            retVal.push_back(links.second[linksIndex].first);
        }
    }

    return retVal;
}

template<size_t N = HELIO_DEFAULT_MAXSIZE>
Vector<HelioObject *, N> linksFilterActuatorsByPanelAndType(Pair<uint8_t, Pair<HelioObject *, int8_t> *> links, HelioPanel *srcPanel, Helio_ActuatorType actuatorType)
{
    Vector<HelioObject *, N> retVal;

    for (hposi_t linksIndex = 0; linksIndex < links.first && links.second[linksIndex].first; ++linksIndex) {
        if (links.second[linksIndex].first->isActuatorType()) {
            auto actuator = static_cast<HelioActuator *>(links.second[linksIndex].first);

            if (actuator->getActuatorType() == actuatorType && actuator->getParentPanel().get() == srcPanel) {
                retVal.push_back(links.second[linksIndex].first);
            }
        }
    }

    return retVal;
}

template<size_t N>
void linksResolveActuatorsByType(Vector<HelioObject *, N> &actuatorsIn, Vector<HelioActuatorAttachment, N> &activationsOut, Helio_ActuatorType actuatorType)
{
    for (auto actIter = actuatorsIn.begin(); actIter != actuatorsIn.end(); ++actIter) {
        auto actuator = getSharedPtr<HelioActuator>(*actIter);
        HELIO_HARD_ASSERT(actuator, SFP(HStr_Err_OperationFailure));
        if (actuator->getActuatorType() == actuatorType) {
            activationsOut.push_back(HelioActuatorAttachment(nullptr));
            activationsOut.back().setObject(actuator);
        }
    }
}

template<size_t N>
void linksResolveActuatorsToAttachmentsByRateAndType(Vector<HelioObject *, N> &actuatorsIn, HelioObjInterface *parent, float rateMultiplier, Vector<HelioActuatorAttachment, N> &activationsOut, Helio_ActuatorType actuatorType)
{
    for (auto actIter = actuatorsIn.begin(); actIter != actuatorsIn.end(); ++actIter) {
        auto actuator = getSharedPtr<HelioActuator>(*actIter);
        HELIO_HARD_ASSERT(actuator, SFP(HStr_Err_OperationFailure));
        if (actuator->getActuatorType() == actuatorType) {
            activationsOut.push_back(HelioActuatorAttachment(nullptr));
            activationsOut.back().setParent(parent);
            activationsOut.back().setRateMultiplier(rateMultiplier);
            activationsOut.back().setObject(actuator);
        }
    }
}


inline Helioduino *getController()
{
    return Helioduino::_activeInstance;
}

inline HelioScheduler *getScheduler()
{
    return Helioduino::_activeInstance ? &Helioduino::_activeInstance->scheduler : nullptr;
}

inline HelioLogger *getLogger()
{
    return Helioduino::_activeInstance ? &Helioduino::_activeInstance->logger : nullptr;
}

inline HelioPublisher *getPublisher()
{
    return Helioduino::_activeInstance ? &Helioduino::_activeInstance->publisher : nullptr;
}

#ifdef HELIO_USE_GUI

inline HelioUIInterface *getUI()
{
    return Helioduino::_activeInstance ? Helioduino::_activeInstance->_activeUIInstance : nullptr;
}

#endif


inline time_t unixTime(DateTime localTime)
{
    return localTime.unixtime() - (getController() ? getController()->getTimeZoneOffset() * SECS_PER_HOUR : 0L);
}

inline DateTime localTime(time_t unixTime)
{
    return DateTime((uint32_t)(unixTime + (getController() ? getController()->getTimeZoneOffset() * SECS_PER_HOUR : 0L)));
}

inline time_t unixDayStart(time_t unixTime)
{
    DateTime currTime = DateTime((uint32_t)unixTime);
    return DateTime(currTime.year(), currTime.month(), currTime.day()).unixtime();
}

inline DateTime localDayStart(time_t unixTime)
{
    DateTime currTime = localTime(unixTime);
    return DateTime(currTime.year(), currTime.month(), currTime.day());
}

extern bool setUnixTime(DateTime unixTime);

inline bool setUnixTime(time_t unixTime)
{
    return setUnixTime(DateTime((uint32_t)unixTime));
}

inline bool setLocalTime(DateTime localTime)
{
    return setUnixTime(unixTime(localTime));
}


inline bool checkPinIsDigital(pintype_t pin)
{
    #ifdef ESP32
        return true; // all digital pins are ADC capable
    #else // separate analog from digital pins
        return !checkPinIsAnalogInput(pin) && !checkPinIsAnalogOutput(pin);
    #endif
}

inline bool checkPinIsPWMOutput(pintype_t pin)
{
    #if defined(digitalPinHasPWM)
        return digitalPinHasPWM(pin);
    #else
        return checkPinIsDigital(pin); // all digital pins are PWM capable
    #endif
}

inline bool checkPinCanInterrupt(pintype_t pin)
{
    return isValidPin(digitalPinToInterrupt(pin));
}

#endif // /ifndef HelioUtils_HPP

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Utilities
*/

#include "Helioduino.h"
#include <pins_arduino.h>

bool HelioRTCWrapper<RTC_DS1307>::begin(TwoWire *wireInstance)
{
    return _rtc.begin(wireInstance);
}

void HelioRTCWrapper<RTC_DS1307>::adjust(const DateTime &dt)
{
    _rtc.adjust(dt);
}

bool HelioRTCWrapper<RTC_DS1307>::lostPower(void)
{
    return false; // not implemented
}

DateTime HelioRTCWrapper<RTC_DS1307>::now()
{
    return _rtc.now();
}


#ifdef HELIO_USE_MULTITASKING

BasicArduinoInterruptAbstraction interruptImpl;

#endif // /ifdef HELIO_USE_MULTITASKING


#ifdef HELIO_USE_DEBUG_ASSERTIONS

static String fileFromFullPath(String fullPath)
{
    int index = fullPath.lastIndexOf(HELIO_BLDPATH_SEPARATOR);
    return index != -1 ? fullPath.substring(index+1) : fullPath;
}

static String makeAssertMsg(const char *file, const char *func, int line)
{
    String retVal;

    retVal.concat(SFP(HStr_ColonSpace));
    retVal.concat(fileFromFullPath(String(file)));
    retVal.concat(':');
    retVal.concat(line);
    retVal.concat(F(" in "));
    retVal.concat(func);
    retVal.concat(SFP(HStr_ColonSpace));

    return retVal;
}

void softAssert(bool cond, String msg, const char *file, const char *func, int line)
{
    if (!cond) {
        if (getLogger()) {
            getLogger()->logWarning(SFP(HStr_Err_AssertionFailure), makeAssertMsg(file, func, line), msg);
            getLogger()->flush();
        }
        #ifdef HELIO_ENABLE_DEBUG_OUTPUT
            else if (Serial) {
                Serial.print(localNow().timestamp(DateTime::TIMESTAMP_FULL));
                Serial.print(' ');
                Serial.print(SFP(HStr_Log_Prefix_Warning));
                Serial.print(SFP(HStr_Err_AssertionFailure));
                Serial.print(makeAssertMsg(file, func, line));
                Serial.println(msg);
                Serial.flush(); yield();
            }
        #endif
    }
}

void hardAssert(bool cond, String msg, const char *file, const char *func, int line)
{
    if (!cond) {
        String assertFail = SFP(HStr_Err_AssertionFailure);
        assertFail.concat(F(" HARD"));
        if (getLogger()) {
            getLogger()->logError(assertFail, makeAssertMsg(file, func, line), msg);
            getLogger()->flush();
        }
        #ifdef HELIO_ENABLE_DEBUG_OUTPUT
            else if (Serial) {
                Serial.print(localNow().timestamp(DateTime::TIMESTAMP_FULL));
                Serial.print(' ');
                Serial.print(SFP(HStr_Log_Prefix_Error));
                Serial.print(assertFail);
                Serial.print(makeAssertMsg(file, func, line));
                Serial.println(msg);
                Serial.flush(); yield();
            }
        #endif

        if (getController()) { getController()->suspend(); }
        yield(); delay(10);
        abort();
    }
}

#endif // /ifdef HELIO_USE_DEBUG_ASSERTIONS


void publishData(HelioSensor *sensor)
{
    HELIO_HARD_ASSERT(sensor, SFP(HStr_Err_InvalidParameter));

    if (getPublisher()) {
        auto measurement = sensor->getMeasurement();
        hposi_t rows = getMeasurementRowCount(measurement);
        hposi_t columnIndexStart = getPublisher()->getColumnIndexStart(sensor->getKey());

        if (columnIndexStart >= 0) {
            for (uint8_t measurementRow = 0; measurementRow < rows; ++measurementRow) {
                getPublisher()->publishData(columnIndexStart + measurementRow, getAsSingleMeasurement(measurement, measurementRow));
            }
        }
    }
}

bool setUnixTime(DateTime unixTime)
{
    auto rtc = getController() ? getController()->getRTC() : nullptr;
    if (rtc) {
        rtc->adjust(unixTime);
        getController()->notifyRTCTimeUpdated();
        return true;
    }
    return false;
}

String getYYMMDDFilename(String prefix, String ext)
{
    DateTime currTime = localNow();
    uint8_t yy = currTime.year() % 100;
    uint8_t mm = currTime.month();
    uint8_t dd = currTime.day();

    String retVal; retVal.reserve(prefix.length() + 11);

    retVal.concat(prefix);
    if (yy < 10) { retVal.concat('0'); }
    retVal.concat(yy);
    if (mm < 10) { retVal.concat('0'); }
    retVal.concat(mm);
    if (dd < 10) { retVal.concat('0'); }
    retVal.concat(dd);
    retVal.concat('.');
    retVal.concat(ext);

    return retVal;
}

String getNNFilename(String prefix, unsigned int value, String ext)
{
    String retVal; retVal.reserve(prefix.length() + 7);

    retVal.concat(prefix);
    if (value < 10) { retVal.concat('0'); }
    retVal.concat(value);
    retVal.concat('.');
    retVal.concat(ext);

    return retVal;
}

void createDirectoryFor(SDClass *sd, String filename)
{
    auto slashIndex = filename.indexOf(HELIO_FSPATH_SEPARATOR);
    String directory = slashIndex != -1 ? filename.substring(0, slashIndex) : String();
    String dirWithSep = directory + String(HELIO_FSPATH_SEPARATOR);
    if (directory.length() && !sd->exists(dirWithSep.c_str())) {
        sd->mkdir(directory.c_str());
    }
}

hkey_t stringHash(String string)
{
    hkey_t hash = 5381;
    for(int index = 0; index < string.length(); ++index) {
        hash = ((hash << 5) + hash) + (hkey_t)string[index]; // Good 'ol DJB2
    }
    return hash != hkey_none ? hash : 5381;
}

String addressToString(uintptr_t addr)
{
    String retVal; retVal.reserve((2 * sizeof(void*)) + 3);
    if (addr == (uintptr_t)-1) { addr = 0; }
    retVal.concat('0'); retVal.concat('x');

    if (sizeof(void*) >= 4) {
        if (addr < 0x10000000) { retVal.concat('0'); }
        if (addr <  0x1000000) { retVal.concat('0'); }
        if (addr <   0x100000) { retVal.concat('0'); }
        if (addr <    0x10000) { retVal.concat('0'); }
    }
    if (sizeof(void*) >= 2) {
        if (addr <     0x1000) { retVal.concat('0'); }
        if (addr <      0x100) { retVal.concat('0'); }
    }
    if (sizeof(void*) >= 1) {
        if (addr <       0x10) { retVal.concat('0'); }
    }

    retVal.concat(String((unsigned long)addr, 16));

    return retVal;
}

String charsToString(const char *charsIn, size_t length)
{
    if (!charsIn || !length) { return String(SFP(HStr_null)); }
    String retVal; retVal.reserve(length + 1);
    for (size_t index = 0; index < length && charsIn[index] != '\0'; ++index) {
        retVal.concat(charsIn[index]);
    }
    return retVal.length() ? retVal : String(SFP(HStr_null));
}

String timeSpanToString(const TimeSpan &span)
{
    String retVal; retVal.reserve(12);

    if (span.days()) {
        retVal.concat(span.days());
        retVal.concat('d');
    }
    if (span.hours()) {
        if (retVal.length()) { retVal.concat(' '); }
        retVal.concat(span.hours());
        retVal.concat('h');
    }
    if (span.minutes()) {
        if (retVal.length()) { retVal.concat(' '); }
        retVal.concat(span.minutes());
        retVal.concat('m');
    }
    if (span.seconds()) {
        if (retVal.length()) { retVal.concat(' '); }
        retVal.concat(span.seconds());
        retVal.concat('s');
    }

    return retVal;
}

extern String measurementToString(float value, Helio_UnitsType units, unsigned int additionalDecPlaces)
{
    String retVal; retVal.reserve(12);
    retVal.concat(roundToString(value, additionalDecPlaces));

    String unitsSym = unitsTypeToSymbol(units, true); // also excludes dimensionless, e.g. pH
    if (unitsSym.length()) {
        retVal.concat(' ');
        retVal.concat(unitsSym);
    }

    return retVal;
}

template<>
String commaStringFromArray<float>(const float *arrayIn, size_t length)
{
    if (!arrayIn || !length) { return String(SFP(HStr_null)); }
    String retVal; retVal.reserve(length << 1);
    for (size_t index = 0; index < length; ++index) {
        if (retVal.length()) { retVal.concat(','); }

        String floatString = String(arrayIn[index], 6);
        int trimIndex = floatString.length() - 1;

        while (floatString[trimIndex] == '0' && trimIndex > 0) { trimIndex--; }
        if (floatString[trimIndex] == '.' && trimIndex > 0) { trimIndex--; }
        if (trimIndex < floatString.length() - 1) {
            floatString = floatString.substring(0, trimIndex+1);
        }

        retVal += floatString;
    }
    return retVal.length() ? retVal : String(SFP(HStr_null));
}

template<>
String commaStringFromArray<double>(const double *arrayIn, size_t length)
{
    if (!arrayIn || !length) { return String(SFP(HStr_null)); }
    String retVal; retVal.reserve(length << 1);
    for (size_t index = 0; index < length; ++index) {
        if (retVal.length()) { retVal.concat(','); }

        String doubleString = String(arrayIn[index], 14);
        int trimIndex = doubleString.length() - 1;

        while (doubleString[trimIndex] == '0' && trimIndex > 0) { trimIndex--; }
        if (doubleString[trimIndex] == '.' && trimIndex > 0) { trimIndex--; }
        if (trimIndex < doubleString.length() - 1) {
            doubleString = doubleString.substring(0, trimIndex+1);
        }

        retVal += doubleString;
    }
    return retVal.length() ? retVal : String(SFP(HStr_null));
}

template<>
void commaStringToArray<float>(String stringIn, float *arrayOut, size_t length)
{
    if (!stringIn.length() || !length || stringIn.equalsIgnoreCase(SFP(HStr_null))) { return; }
    int lastSepPos = -1;
    for (size_t index = 0; index < length; ++index) {
        int nextSepPos = stringIn.indexOf(',', lastSepPos+1);
        if (nextSepPos == -1) { nextSepPos = stringIn.length(); }
        String subString = stringIn.substring(lastSepPos+1, nextSepPos);
        if (nextSepPos < stringIn.length()) { lastSepPos = nextSepPos; }

        arrayOut[index] = subString.toFloat();
    }
}

template<>
void commaStringToArray<double>(String stringIn, double *arrayOut, size_t length)
{
    if (!stringIn.length() || !length || stringIn.equalsIgnoreCase(SFP(HStr_null))) { return; }
    int lastSepPos = -1;
    for (size_t index = 0; index < length; ++index) {
        int nextSepPos = stringIn.indexOf(',', lastSepPos+1);
        if (nextSepPos == -1) { nextSepPos = stringIn.length(); }
        String subString = stringIn.substring(lastSepPos+1, nextSepPos);
        if (nextSepPos < stringIn.length()) { lastSepPos = nextSepPos; }

        #if !defined(CORE_TEENSY)
            arrayOut[index] = subString.toDouble();
        #else
            arrayOut[index] = subString.toFloat();
        #endif
    }
}

template<>
void commaStringToArray<String>(String stringIn, String *arrayOut, size_t length)
{
    if (!stringIn.length() || !length || stringIn.equalsIgnoreCase(SFP(HStr_null))) { return; }
    int lastSepPos = -1;
    for (size_t index = 0; index < length; ++index) {
        int nextSepPos = stringIn.indexOf(',', lastSepPos+1);
        if (nextSepPos == -1) { nextSepPos = stringIn.length(); }
        String subString = stringIn.substring(lastSepPos+1, nextSepPos);
        if (nextSepPos < stringIn.length()) { lastSepPos = nextSepPos; }
        arrayOut[index] = subString;
    }
}

String hexStringFromBytes(const uint8_t *bytesIn, size_t length)
{
    if (!bytesIn || !length) { return String(SFP(HStr_null)); }
    String retVal; retVal.reserve((length << 1) + 1);
    for (size_t index = 0; index < length; ++index) {
        String valStr = String(bytesIn[index], 16);
        if (valStr.length() == 1) { valStr = String('0') + valStr; }

        retVal += valStr;
    }
    return retVal.length() ? retVal : String(SFP(HStr_null));
}

void hexStringToBytes(String stringIn, uint8_t *bytesOut, size_t length)
{
    if (!stringIn.length() || !length || stringIn.equalsIgnoreCase(SFP(HStr_null))) { return; }
    for (size_t index = 0; index < length; ++index) {
        String valStr = stringIn.substring(index << 1,(index+1) << 1);
        if (valStr.length() == 2) { bytesOut[index] = strtoul(valStr.c_str(), nullptr, 16); }
        else { bytesOut[index] = 0; }
    }
}

void hexStringToBytes(JsonVariantConst &variantIn, uint8_t *bytesOut, size_t length)
{
    if (variantIn.isNull() || variantIn.is<JsonObjectConst>() || variantIn.is<JsonArrayConst>()) { return; }
    hexStringToBytes(variantIn.as<String>(), bytesOut, length);
}

int occurrencesInString(String string, char singleChar)
{
    int retVal = 0;
    int posIndex = string.indexOf(singleChar);
    while (posIndex != -1) {
        retVal++;
        posIndex = string.indexOf(singleChar, posIndex+1);
    }
    return retVal;
}

int occurrencesInString(String string, String subString)
{
    int retVal = 0;
    int posIndex = string.indexOf(subString[0]);
    while (posIndex != -1) {
        if (subString.equals(string.substring(posIndex, posIndex + subString.length()))) {
            retVal++;
            posIndex += subString.length();
        }
        posIndex = string.indexOf(subString[0], posIndex+1);
    }
    return retVal;
}

int occurrencesInStringIgnoreCase(String string, char singleChar)
{
    int retVal = 0;
    int posIndex = min(string.indexOf(tolower(singleChar)), string.indexOf(toupper(singleChar)));
    while (posIndex != -1) {
        retVal++;
        posIndex = min(string.indexOf(tolower(singleChar), posIndex+1), string.indexOf(toupper(singleChar), posIndex+1));
    }
    return retVal;
}

int occurrencesInStringIgnoreCase(String string, String subString)
{
    int retVal = 0;
    int posIndex = min(string.indexOf(tolower(subString[0])), string.indexOf(toupper(subString[0])));
    while (posIndex != -1) {
        if (subString.equalsIgnoreCase(string.substring(posIndex, posIndex + subString.length()))) {
            retVal++;
            posIndex += subString.length();
        }
        posIndex = min(string.indexOf(tolower(subString[0]), posIndex+1), string.indexOf(toupper(subString[0]), posIndex+1));
    }
    return retVal;
}

template<>
bool arrayElementsEqual<float>(const float *arrayIn, size_t length, float value)
{
    for (size_t index = 0; index < length; ++index) {
        if (!isFPEqual(arrayIn[index], value)) {
            return false;
        }
    }
    return true;
}

template<>
bool arrayElementsEqual<double>(const double *arrayIn, size_t length, double value)
{
    for (size_t index = 0; index < length; ++index) {
        if (!isFPEqual(arrayIn[index], value)) {
            return false;
        }
    }
    return true;
}

// See: https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#elif !defined(ESP_PLATFORM)
extern char *__brkval;
#elif defined(ESP8266)
extern "C" {
#include "user_interface.h"
}
#endif

unsigned int freeMemory() {
    #if defined(ESP32)
        return esp_get_free_heap_size();
    #elif defined(ESP8266)
        return system_get_free_heap_size();
    #else
        char top;
        #ifdef __arm__
            return &top - reinterpret_cast<char*>(sbrk(0));
        #elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
            return &top - __brkval;
        #else
            return __brkval ? &top - __brkval : &top - __malloc_heap_start;
        #endif
        return 0;
    #endif
}

void delayFine(millis_t duration) {
    millis_t start = millis();
    millis_t end = start + duration;

    {   millis_t left = max(0, duration - HELIO_SYS_DELAYFINE_SPINMILLIS);
        if (left > 0) { delay(left); }
    }

    {   millis_t time = millis();
        while ((end >= start && (time < end)) ||
               (end < start && (time >= start || time < end))) {
            time = millis();
        }
    }
}

bool tryConvertUnits(float valueIn, Helio_UnitsType unitsIn, float *valueOut, Helio_UnitsType unitsOut, float convertParam)
{
    if (!valueOut || unitsOut == Helio_UnitsType_Undefined || unitsIn == unitsOut) return false;

    switch (unitsIn) {
        case Helio_UnitsType_Raw_1:
            switch (unitsOut) {
                // Known extents

                case Helio_UnitsType_Percentile_100:
                    *valueOut = valueIn * 100.0;
                    return true;

                case Helio_UnitsType_Angle_Degrees_360:
                    *valueOut = wrapBy360(valueIn * 360.0);
                    return true;

                case Helio_UnitsType_Angle_Radians_2pi:
                    *valueOut = wrapBy2Pi(valueIn * TWO_PI);
                    return true;

                case Helio_UnitsType_Angle_Minutes_24hr:
                    *valueOut = wrapBy24Hr(valueIn * MIN_PER_DAY);
                    return true;

                default:
                    if (convertParam != FLT_UNDEF) {
                        *valueOut = valueIn * convertParam;
                        return true;
                    }
                    break;
            }
            break;

        case Helio_UnitsType_Percentile_100:
            switch (unitsOut) {
                case Helio_UnitsType_Raw_1:
                    *valueOut = valueIn / 100.0;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Angle_Degrees_360:
            switch (unitsOut) {
                case Helio_UnitsType_Angle_Radians_2pi:
                    *valueOut = wrapBy2Pi(valueIn * (TWO_PI / 360.0));
                    return true;

                case Helio_UnitsType_Angle_Minutes_24hr:
                    *valueOut = wrapBy24Hr(valueIn * (MIN_PER_DAY / 360.0));
                    return true;

                case Helio_UnitsType_Raw_1:
                    *valueOut = valueIn / 360.0;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Angle_Radians_2pi:
            switch (unitsOut) {
                case Helio_UnitsType_Angle_Degrees_360:
                    *valueOut = wrapBy360(valueIn * (360.0 / TWO_PI));
                    return true;

                case Helio_UnitsType_Angle_Minutes_24hr:
                    *valueOut = wrapBy24Hr(valueIn * (MIN_PER_DAY / TWO_PI));
                    return true;

                case Helio_UnitsType_Raw_1:
                    *valueOut = valueIn / TWO_PI;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Angle_Minutes_24hr:
            switch (unitsOut) {
                case Helio_UnitsType_Angle_Degrees_360:
                    *valueOut = wrapBy360(valueIn * (360.0 / MIN_PER_DAY));
                    return true;

                case Helio_UnitsType_Angle_Radians_2pi:
                    *valueOut = wrapBy2Pi(valueIn * (TWO_PI / MIN_PER_DAY));
                    return true;

                case Helio_UnitsType_Raw_1:
                    *valueOut = valueIn / TWO_PI;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Distance_Feet:
            switch (unitsOut) {
                case Helio_UnitsType_Distance_Meters:
                    *valueOut = valueIn * 0.3048;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Distance_Meters:
            switch (unitsOut) {
                case Helio_UnitsType_Distance_Feet:
                    *valueOut = valueIn * 3.28084;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Power_Amperage:
            switch (unitsOut) {
                case Helio_UnitsType_Power_Wattage:
                    if (convertParam != FLT_UNDEF) { // convertParam = rail voltage
                        *valueOut = valueIn * convertParam;
                        return true;
                    }
                break;
            }
            break;

        case Helio_UnitsType_Power_Wattage:
            switch (unitsOut) {
                case Helio_UnitsType_Power_Amperage:
                    if (convertParam != FLT_UNDEF) { // convertParam = rail voltage
                        *valueOut = valueIn / convertParam;
                        return true;
                    }
                break;
            }
            break;

        case Helio_UnitsType_Speed_FeetPerMin:
            switch (unitsOut) {
                case Helio_UnitsType_Speed_MetersPerMin:
                    *valueOut = valueIn * 0.3048;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Speed_MetersPerMin:
            switch (unitsOut) {
                case Helio_UnitsType_Speed_FeetPerMin:
                    *valueOut = valueIn * 3.28084;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Temperature_Celsius:
            switch (unitsOut) {
                case Helio_UnitsType_Temperature_Fahrenheit:
                    *valueOut = valueIn * 1.8 + 32.0;
                    return true;

                case Helio_UnitsType_Temperature_Kelvin:
                    *valueOut = valueIn + 273.15;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Temperature_Fahrenheit:
            switch (unitsOut) {
                case Helio_UnitsType_Temperature_Celsius:
                    *valueOut = (valueIn - 32.0) / 1.8;
                    return true;

                case Helio_UnitsType_Temperature_Kelvin:
                    *valueOut = ((valueIn + 459.67) * 5.0) / 9.0;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Temperature_Kelvin:
            switch (unitsOut) {
                case Helio_UnitsType_Temperature_Celsius:
                    *valueOut = valueIn - 273.15;
                    return true;

                case Helio_UnitsType_Temperature_Fahrenheit:
                    *valueOut = ((valueIn * 9.0) / 5.0) - 459.67;
                    return true;

                default:
                    break;
            }
            break;

        case Helio_UnitsType_Undefined:
            *valueOut = valueIn;
            return true;

        default:
            break;
    }

    return false;
}

Helio_UnitsType baseUnits(Helio_UnitsType units)
{
    switch (units) {
        case Helio_UnitsType_Speed_MetersPerMin:
            return Helio_UnitsType_Distance_Meters;
        case Helio_UnitsType_Speed_FeetPerMin:
            return Helio_UnitsType_Distance_Feet;
        default:
            return Helio_UnitsType_Undefined;
    }
}

Helio_UnitsType rateUnits(Helio_UnitsType units)
{
    switch (units) {
        case Helio_UnitsType_Distance_Meters:
            return Helio_UnitsType_Speed_MetersPerMin;
        case Helio_UnitsType_Distance_Feet:
            return Helio_UnitsType_Speed_FeetPerMin;
        default:
            return Helio_UnitsType_Undefined;
    }
}

Helio_UnitsType defaultUnits(Helio_UnitsCategory unitsCategory, Helio_MeasurementMode measureMode)
{
    measureMode = (measureMode == Helio_MeasurementMode_Undefined && getController() ? getController()->getMeasurementMode() : measureMode);

    switch (unitsCategory) {
        case Helio_UnitsCategory_Angle:
            switch (measureMode) {
                case Helio_MeasurementMode_Imperial:
                case Helio_MeasurementMode_Metric:
                    return Helio_UnitsType_Angle_Degrees_360;
                case Helio_MeasurementMode_Scientific:
                    return Helio_UnitsType_Angle_Radians_2pi;
                default:
                    return Helio_UnitsType_Undefined;
            }

        case Helio_UnitsCategory_Distance:
            switch (measureMode) {
                case Helio_MeasurementMode_Imperial:
                    return Helio_UnitsType_Distance_Feet;
                case Helio_MeasurementMode_Metric:
                case Helio_MeasurementMode_Scientific:
                    return Helio_UnitsType_Distance_Meters;
                default:
                    return Helio_UnitsType_Undefined;
            }

        case Helio_UnitsCategory_Percentile:
            return Helio_UnitsType_Percentile_100;

        case Helio_UnitsCategory_Power:
            return Helio_UnitsType_Power_Wattage;

        case Helio_UnitsCategory_Speed:
            switch (measureMode) {
                case Helio_MeasurementMode_Imperial:
                    return Helio_UnitsType_Speed_FeetPerMin;
                case Helio_MeasurementMode_Metric:
                case Helio_MeasurementMode_Scientific:
                    return Helio_UnitsType_Speed_MetersPerMin;
                default:
                    return Helio_UnitsType_Undefined;
            }

        case Helio_UnitsCategory_Temperature:
            switch (measureMode) {
                case Helio_MeasurementMode_Imperial:
                    return Helio_UnitsType_Temperature_Fahrenheit;
                case Helio_MeasurementMode_Metric:
                    return Helio_UnitsType_Temperature_Celsius;
                case Helio_MeasurementMode_Scientific:
                    return Helio_UnitsType_Temperature_Kelvin;
                default:
                    return Helio_UnitsType_Undefined;
            }

        case Helio_UnitsCategory_Count:
            switch (measureMode) {
                case Helio_MeasurementMode_Scientific:
                    return (Helio_UnitsType)2;
                default:
                    return (Helio_UnitsType)1;
            }

        case Helio_UnitsCategory_Undefined:
            return Helio_UnitsType_Undefined;
    }
    return Helio_UnitsType_Undefined;
}

int linksCountTravelActuators(Pair<uint8_t, Pair<HelioObject *, int8_t> *> links)
{
    int retVal = 0;

    for (hposi_t linksIndex = 0; linksIndex < links.first && links.second[linksIndex].first; ++linksIndex) {
        if (links.second[linksIndex].first->isActuatorType()) {
            auto actuator = static_cast<HelioActuator *>(links.second[linksIndex].first);

            if (actuator->isTravelType()) {
                retVal++;
            }
        }
    }

    return retVal;
}

int linksCountActuatorsByPanelAndType(Pair<uint8_t, Pair<HelioObject *, int8_t> *> links, HelioPanel *panel, Helio_ActuatorType actuatorType)
{
    int retVal = 0;

    for (hposi_t linksIndex = 0; linksIndex < links.first && links.second[linksIndex].first; ++linksIndex) {
        if (links.second[linksIndex].first->isActuatorType()) {
            auto actuator = static_cast<HelioActuator *>(links.second[linksIndex].first);

            if (actuator->getActuatorType() == actuatorType && actuator->getParentPanel().get() == panel) {
                retVal++;
            }
        }
    }

    return retVal;
}


bool checkPinIsAnalogInput(pintype_t pin)
{
    #if !defined(NUM_ANALOG_INPUTS) || NUM_ANALOG_INPUTS == 0
        return false;
    #elif defined(ESP32)
        return checkPinIsDigital(pin); // all digital pins are ADC capable
    #else
        switch (pin) {
            #if NUM_ANALOG_INPUTS > 0
                case (pintype_t)A0:
            #endif
            #if NUM_ANALOG_INPUTS > 1 && !(defined(PIN_A0) && !defined(PIN_A1))
                case (pintype_t)A1:
            #endif
            #if NUM_ANALOG_INPUTS > 2 && !(defined(PIN_A0) && !defined(PIN_A2))
                case (pintype_t)A2:
            #endif
            #if NUM_ANALOG_INPUTS > 3 && !(defined(PIN_A0) && !defined(PIN_A3))
                case (pintype_t)A3:
            #endif
            #if NUM_ANALOG_INPUTS > 4 && !(defined(PIN_A0) && !defined(PIN_A4))
                case (pintype_t)A4:
            #endif
            #if NUM_ANALOG_INPUTS > 5 && !(defined(PIN_A0) && !defined(PIN_A5))
                case (pintype_t)A5:
            #endif
            #if NUM_ANALOG_INPUTS > 6 && !(defined(PIN_A0) && !defined(PIN_A6))
                case (pintype_t)A6:
            #endif
            #if NUM_ANALOG_INPUTS > 7 && !(defined(PIN_A0) && !defined(PIN_A7))
                case (pintype_t)A7:
            #endif
            #if NUM_ANALOG_INPUTS > 8 && !(defined(PIN_A0) && !defined(PIN_A8))
                case (pintype_t)A8:
            #endif
            #if NUM_ANALOG_INPUTS > 9 && !(defined(PIN_A0) && !defined(PIN_A9))
                case (pintype_t)A9:
            #endif
            #if NUM_ANALOG_INPUTS > 10 && !(defined(PIN_A0) && !defined(PIN_A10))
                case (pintype_t)A10:
            #endif
            #if NUM_ANALOG_INPUTS > 11 && !(defined(PIN_A0) && !defined(PIN_A11))
                case (pintype_t)A11:
            #endif
            #if NUM_ANALOG_INPUTS > 12 && !(defined(PIN_A0) && !defined(PIN_A12))
                case (pintype_t)A12:
            #endif
            #if NUM_ANALOG_INPUTS > 13 && !(defined(PIN_A0) && !defined(PIN_A13))
                case (pintype_t)A13:
            #endif
            #if NUM_ANALOG_INPUTS > 14 && !(defined(PIN_A0) && !defined(PIN_A14))
                case (pintype_t)A14:
            #endif
            #if NUM_ANALOG_INPUTS > 15 && !(defined(PIN_A0) && !defined(PIN_A15))
                case (pintype_t)A15:
            #endif
            #if NUM_ANALOG_INPUTS > 16 && !(defined(PIN_A0) && !defined(PIN_A16))
                case (pintype_t)A16:
            #endif
            #if NUM_ANALOG_INPUTS > 17 && !(defined(PIN_A0) && !defined(PIN_A17))
                case (pintype_t)A17:
            #endif
            #if NUM_ANALOG_INPUTS > 18 && !(defined(PIN_A0) && !defined(PIN_A18))
                case (pintype_t)A18:
            #endif
            #if NUM_ANALOG_INPUTS > 19 && !(defined(PIN_A0) && !defined(PIN_A19))
                case (pintype_t)A19:
            #endif
            #if NUM_ANALOG_INPUTS > 20 && !(defined(PIN_A0) && !defined(PIN_A20))
                case (pintype_t)A20:
            #endif
            #if NUM_ANALOG_INPUTS > 21 && !(defined(PIN_A0) && !defined(PIN_A21))
                case (pintype_t)A21:
            #endif
                return true;

            default:
                return false;
        }
    #endif
}

bool checkPinIsAnalogOutput(pintype_t pin)
{
    #if !defined(NUM_ANALOG_OUTPUTS) || NUM_ANALOG_OUTPUTS == 0
        return false;
    #elif defined(ESP32)
        return checkPinIsDigital(pin); // all digital pins are ADC capable
    #else
        switch (pin) {
            #if NUM_ANALOG_OUTPUTS > 0
                #ifndef PIN_DAC0
                    case (pintype_t)A0:
                #else
                    case (pintype_t)DAC0:
                #endif
            #endif
            #if NUM_ANALOG_OUTPUTS > 1
                #ifndef PIN_DAC1
                    case (pintype_t)A1:
                #else
                    case (pintype_t)DAC1:
                #endif
            #endif
            #if NUM_ANALOG_OUTPUTS > 2
                #ifndef PIN_DAC2
                    case (pintype_t)A2:
                #else
                    case (pintype_t)DAC2:
                #endif
            #endif
            #if NUM_ANALOG_OUTPUTS > 3
                #ifndef PIN_DAC3
                    case (pintype_t)A3:
                #else
                    case (pintype_t)DAC3:
                #endif
            #endif
            #if NUM_ANALOG_OUTPUTS > 4
                #ifndef PIN_DAC4
                    case (pintype_t)A4:
                #else
                    case (pintype_t)DAC4:
                #endif
            #endif
            #if NUM_ANALOG_OUTPUTS > 5
                #ifndef PIN_DAC5
                    case (pintype_t)A5:
                #else
                    case (pintype_t)DAC5:
                #endif
            #endif
            #if NUM_ANALOG_OUTPUTS > 6
                #ifndef PIN_DAC6
                    case (pintype_t)A6:
                #else
                    case (pintype_t)DAC6:
                #endif
            #endif
            #if NUM_ANALOG_OUTPUTS > 7
                #ifndef PIN_DAC7
                    case (pintype_t)A7:
                #else
                    case (pintype_t)DAC7:
                #endif
            #endif
                return true;

            default:
                return false;
        }
    #endif
}


String systemModeToString(Helio_SystemMode systemMode, bool excludeSpecial)
{
    switch (systemMode) {
        case Helio_SystemMode_Tracking:
            return SFP(HStr_Enum_Tracking);
        case Helio_SystemMode_Balancing:
            return SFP(HStr_Enum_Balancing);
        case Helio_SystemMode_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_SystemMode_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String measurementModeToString(Helio_MeasurementMode measurementMode, bool excludeSpecial)
{
    switch (measurementMode) {
        case Helio_MeasurementMode_Imperial:
            return SFP(HStr_Enum_Imperial);
        case Helio_MeasurementMode_Metric:
            return SFP(HStr_Enum_Metric);
        case Helio_MeasurementMode_Scientific:
            return SFP(HStr_Enum_Scientific);
        case Helio_MeasurementMode_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_MeasurementMode_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String displayOutputModeToString(Helio_DisplayOutputMode displayOutMode, bool excludeSpecial)
{
    switch (displayOutMode) {
        case Helio_DisplayOutputMode_Disabled:
            return SFP(HStr_Disabled);
        case Helio_DisplayOutputMode_LCD16x2:
            return SFP(HStr_Enum_LCD16x2);
        case Helio_DisplayOutputMode_LCD16x2_Swapped: {
            String retVal(SFP(HStr_Enum_LCD16x2));
            String concat(SFP(HStr_Enum_Swapped));
            retVal.reserve(retVal.length() + concat.length());
            retVal.concat(concat);
            return retVal;
        }
        case Helio_DisplayOutputMode_LCD20x4:
            return SFP(HStr_Enum_LCD20x4);
        case Helio_DisplayOutputMode_LCD20x4_Swapped: {
            String retVal(SFP(HStr_Enum_LCD20x4));
            String concat(SFP(HStr_Enum_Swapped));
            retVal.reserve(retVal.length() + concat.length());
            retVal.concat(concat);
            return retVal;
        }
        case Helio_DisplayOutputMode_SSD1305:
            return SFP(HStr_Enum_SSD1305);
        case Helio_DisplayOutputMode_SSD1305_x32Ada:
            return SFP(HStr_Enum_SSD1305x32Ada);
        case Helio_DisplayOutputMode_SSD1305_x64Ada:
            return SFP(HStr_Enum_SSD1305x64Ada);
        case Helio_DisplayOutputMode_SSD1306:
            return SFP(HStr_Enum_SSD1306);
        case Helio_DisplayOutputMode_SH1106:
            return SFP(HStr_Enum_SH1106);
        case Helio_DisplayOutputMode_SSD1607_GD: {
            String retVal(SFP(HStr_Enum_SSD1607));
            retVal.reserve(retVal.length() + 2);
            retVal.concat('G'); retVal.concat('D');
            return retVal;
        }
        case Helio_DisplayOutputMode_SSD1607_WS: {
            String retVal(SFP(HStr_Enum_SSD1607));
            retVal.reserve(retVal.length() + 2);
            retVal.concat('W'); retVal.concat('S');
            return retVal;
        }
        case Helio_DisplayOutputMode_IL3820:
            return SFP(HStr_Enum_IL3820);
        case Helio_DisplayOutputMode_IL3820_V2:
            return SFP(HStr_Enum_IL3820V2);
        case Helio_DisplayOutputMode_ST7735:
            return SFP(HStr_Enum_ST7735);
        case Helio_DisplayOutputMode_ST7789:
            return SFP(HStr_Enum_ST7789);
        case Helio_DisplayOutputMode_ILI9341:
            return SFP(HStr_Enum_ILI9341);
        case Helio_DisplayOutputMode_PCD8544:
            return SFP(HStr_Enum_PCD8544);
        case Helio_DisplayOutputMode_TFT: {
            String retVal; retVal.reserve(3);
            retVal.concat('T'); retVal.concat('F'); retVal.concat('T');
            return retVal;
        }
        case Helio_DisplayOutputMode_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_DisplayOutputMode_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String controlInputModeToString(Helio_ControlInputMode controlInMode, bool excludeSpecial)
{
    switch (controlInMode) {
        case Helio_ControlInputMode_Disabled:
            return SFP(HStr_Disabled);
        case Helio_ControlInputMode_RotaryEncoderOk: {
            String retVal(SFP(HStr_Enum_RotaryEncoder));
            retVal.reserve(retVal.length() + 2);
            retVal.concat('O'); retVal.concat('k');
            return retVal;
        }
        case Helio_ControlInputMode_RotaryEncoderOk_LR: {
            String retVal(SFP(HStr_Enum_RotaryEncoder));
            retVal.reserve(retVal.length() + 4);
            retVal.concat('O'); retVal.concat('k');
            retVal.concat('L'); retVal.concat('R');
            return retVal;
        }
        case Helio_ControlInputMode_UpDownOkButtons: {
            String retVal(SFP(HStr_Enum_UpDownButtons));
            retVal.reserve(retVal.length() + 2);
            retVal.concat('O'); retVal.concat('k');
            return retVal;
        }
        case Helio_ControlInputMode_UpDownOkButtons_LR: {
            String retVal(SFP(HStr_Enum_UpDownButtons));
            retVal.reserve(retVal.length() + 4);
            retVal.concat('O'); retVal.concat('k');
            retVal.concat('L'); retVal.concat('R');
            return retVal;
        }
        case Helio_ControlInputMode_AnalogJoystickOk: {
            String retVal(SFP(HStr_Enum_AnalogJoystick));
            retVal.reserve(retVal.length() + 2);
            retVal.concat('O'); retVal.concat('k');
            return retVal;
        }
        case Helio_ControlInputMode_Matrix3x4Keyboard_OptRotEncOk: {
            String retVal(SFP(HStr_Enum_Matrix3x4));
            retVal.reserve(retVal.length() + 2);
            retVal.concat('O'); retVal.concat('k');
            return retVal;
        }
        case Helio_ControlInputMode_Matrix3x4Keyboard_OptRotEncOkLR: {
            String retVal(SFP(HStr_Enum_Matrix3x4));
            retVal.reserve(retVal.length() + 4);
            retVal.concat('O'); retVal.concat('k');
            retVal.concat('L'); retVal.concat('R');
            return retVal;
        }
        case Helio_ControlInputMode_Matrix4x4Keyboard_OptRotEncOk: {
            String retVal(SFP(HStr_Enum_Matrix4x4));
            retVal.reserve(retVal.length() + 2);
            retVal.concat('O'); retVal.concat('k');
            return retVal;
        }
        case Helio_ControlInputMode_Matrix4x4Keyboard_OptRotEncOkLR: {
            String retVal(SFP(HStr_Enum_Matrix4x4));
            retVal.reserve(retVal.length() + 4);
            retVal.concat('O'); retVal.concat('k');
            retVal.concat('L'); retVal.concat('R');
            return retVal;
        }
        case Helio_ControlInputMode_ResistiveTouch:
            return SFP(HStr_Enum_ResistiveTouch);
        case Helio_ControlInputMode_TouchScreen:
            return SFP(HStr_Enum_TouchScreen);
        case Helio_ControlInputMode_TFTTouch:
            return SFP(HStr_Enum_TFTTouch);
        case Helio_ControlInputMode_RemoteControl:
            return SFP(HStr_Enum_RemoteControl);
        case Helio_ControlInputMode_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_ControlInputMode_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String actuatorTypeToString(Helio_ActuatorType actuatorType, bool excludeSpecial)
{
    switch (actuatorType) {
        case Helio_ActuatorType_ContinuousServo:
            return SFP(HStr_Enum_ContinuousServo);
        case Helio_ActuatorType_LinearActuator:
            return SFP(HStr_Enum_LinearActuator);
        case Helio_ActuatorType_PanelBrake:
            return SFP(HStr_Enum_PanelBrake);
        case Helio_ActuatorType_PanelCover:
            return SFP(HStr_Enum_PanelCover);
        case Helio_ActuatorType_PanelHeater:
            return SFP(HStr_Enum_PanelHeater);
        case Helio_ActuatorType_PanelSprayer:
            return SFP(HStr_Enum_PanelSprayer);
        case Helio_ActuatorType_PositionalServo:
            return SFP(HStr_Enum_PositionalServo);
        case Helio_ActuatorType_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_ActuatorType_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String sensorTypeToString(Helio_SensorType sensorType, bool excludeSpecial)
{
    switch (sensorType) {
        case Helio_SensorType_IceDetector:
            return SFP(HStr_Enum_IceDetector);
        case Helio_SensorType_LightIntensity:
            return SFP(HStr_Enum_LightIntensity);
        case Helio_SensorType_PowerProduction:
            return SFP(HStr_Enum_PowerProduction);
        case Helio_SensorType_PowerUsage:
            return SFP(HStr_Enum_PowerUsage);
        case Helio_SensorType_TravelPosition:
            return SFP(HStr_Enum_TravelPosition);
        case Helio_SensorType_TemperatureHumidity:
            return SFP(HStr_Enum_Temperature);
        case Helio_SensorType_TiltAngle:
            return SFP(HStr_Enum_TiltAngle);
        case Helio_SensorType_WindSpeed:
            return SFP(HStr_Enum_WindSpeed);
        case Helio_SensorType_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_SensorType_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

hposi_t getPanelAxisCountFromType(Helio_PanelType panelType)
{
    switch (panelType) {
        case Helio_PanelType_Horizontal:
        case Helio_PanelType_Vertical:
            return 1;
        case Helio_PanelType_Gimballed:
        case Helio_PanelType_Equatorial:
            return 2;
        default:
            return 0;
    }
}

String panelTypeToString(Helio_PanelType panelType, bool excludeSpecial)
{
    switch (panelType) {
        case Helio_PanelType_Horizontal:
            return SFP(HStr_Enum_Horizontal);
        case Helio_PanelType_Vertical:
            return SFP(HStr_Enum_Vertical);
        case Helio_PanelType_Gimballed:
            return SFP(HStr_Enum_Gimballed);
        case Helio_PanelType_Equatorial:
            return SFP(HStr_Enum_Equatorial);
        case Helio_PanelType_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_PanelType_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

float getRailVoltageFromType(Helio_RailType railType)
{
    switch (railType) {
        case Helio_RailType_AC110V:
            return 110.0f;
        case Helio_RailType_AC220V:
            return 220.0f;
        case Helio_RailType_DC3V3:
            return 3.3f;
        case Helio_RailType_DC5V:
            return 5.0f;
        case Helio_RailType_DC12V:
            return 12.0f;
        case Helio_RailType_DC24V:
            return 24.0f;
        case Helio_RailType_DC48V:
            return 48.0f;
        default:
            return 0.0f;
    }
}

String railTypeToString(Helio_RailType railType, bool excludeSpecial)
{
    switch (railType) {
        case Helio_RailType_AC110V:
            return SFP(HStr_Enum_AC110V);
        case Helio_RailType_AC220V:
            return SFP(HStr_Enum_AC220V);
        case Helio_RailType_DC3V3:
            return SFP(HStr_Enum_DC3V3);
        case Helio_RailType_DC5V:
            return SFP(HStr_Enum_DC5V);
        case Helio_RailType_DC12V:
            return SFP(HStr_Enum_DC12V);
        case Helio_RailType_DC24V:
            return SFP(HStr_Enum_DC24V);
        case Helio_RailType_DC48V:
            return SFP(HStr_Enum_DC48V);
        case Helio_RailType_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_RailType_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String pinModeToString(Helio_PinMode pinMode, bool excludeSpecial)
{
    switch (pinMode) {
        case Helio_PinMode_Digital_Input:
            return SFP(HStr_Enum_DigitalInput);
        case Helio_PinMode_Digital_Input_PullUp:
            return SFP(HStr_Enum_DigitalInputPullUp);
        case Helio_PinMode_Digital_Input_PullDown:
            return SFP(HStr_Enum_DigitalInputPullDown);
        case Helio_PinMode_Digital_Output:
            return SFP(HStr_Enum_DigitalOutput);
        case Helio_PinMode_Digital_Output_PushPull:
            return SFP(HStr_Enum_DigitalOutputPushPull);
        case Helio_PinMode_Analog_Input:
            return SFP(HStr_Enum_AnalogInput);
        case Helio_PinMode_Analog_Output:
            return SFP(HStr_Enum_AnalogOutput);
        case Helio_PinMode_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_PinMode_Undefined:
            break;
        default:
            return String((int)pinMode);
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String enableModeToString(Helio_EnableMode enableMode, bool excludeSpecial)
{
    switch (enableMode) {
        case Helio_EnableMode_Highest:
            return SFP(HStr_Enum_Highest);
        case Helio_EnableMode_Lowest:
            return SFP(HStr_Enum_Lowest);
        case Helio_EnableMode_Average:
            return SFP(HStr_Enum_Average);
        case Helio_EnableMode_Multiply:
            return SFP(HStr_Enum_Multiply);
        case Helio_EnableMode_InOrder:
            return SFP(HStr_Enum_InOrder);
        case Helio_EnableMode_RevOrder:
            return SFP(HStr_Enum_RevOrder);
        case Helio_EnableMode_DescOrder:
            return SFP(HStr_Enum_DescOrder);
        case Helio_EnableMode_AscOrder:
            return SFP(HStr_Enum_AscOrder);
        case Helio_EnableMode_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_EnableMode_Undefined:
            break;
        default:
            return String((int)enableMode);
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String unitsCategoryToString(Helio_UnitsCategory unitsCategory, bool excludeSpecial)
{
    switch (unitsCategory) {
        case Helio_UnitsCategory_Angle:
            return SFP(HStr_Enum_Angle);
        case Helio_UnitsCategory_Distance:
            return SFP(HStr_Enum_Distance);
        case Helio_UnitsCategory_Percentile:
            return SFP(HStr_Enum_Percentile);
        case Helio_UnitsCategory_Power:
            return SFP(HStr_Enum_Power);
        case Helio_UnitsCategory_Speed:
            return SFP(HStr_Enum_Speed);
        case Helio_UnitsCategory_Temperature:
            return SFP(HStr_Enum_Temperature);
        case Helio_UnitsCategory_Count:
            return !excludeSpecial ? SFP(HStr_Count) : String();
        case Helio_UnitsCategory_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Undefined) : String();
}

String unitsTypeToSymbol(Helio_UnitsType unitsType, bool excludeSpecial)
{
    switch (unitsType) {
        case Helio_UnitsType_Raw_1:
            return SFP(HStr_raw);
        case Helio_UnitsType_Percentile_100:
            return String('%');
        case Helio_UnitsType_Angle_Degrees_360:
            return SFP(HStr_Unit_Degree);
        case Helio_UnitsType_Angle_Radians_2pi: {
            String retVal(SFP(HStr_Unit_Degree));
            String concat(SFP(HStr_Unit_Radians));
            retVal.reserve(retVal.length() + concat.length());
            retVal.concat(concat);
            return retVal;
        }
        case Helio_UnitsType_Angle_Minutes_24hr:
            return SFP(HStr_Unit_Minutes);
        case Helio_UnitsType_Distance_Feet:
            return SFP(HStr_Unit_Feet);
        case Helio_UnitsType_Distance_Meters:
            return String('m');
        case Helio_UnitsType_Power_Amperage:
            return String('A');
        case Helio_UnitsType_Power_Wattage:
            return String('W');
        case Helio_UnitsType_Speed_FeetPerMin: {
            String retVal(SFP(HStr_Unit_Feet));
            String concat(SFP(HStr_Unit_PerMinute));
            retVal.reserve(retVal.length() + concat.length());
            retVal.concat(concat);
            return retVal;
        }
        case Helio_UnitsType_Speed_MetersPerMin: {
            String retVal('m');
            String concat(SFP(HStr_Unit_PerMinute));
            retVal.reserve(retVal.length() + concat.length());
            retVal.concat(concat);
            return retVal;
        }
        case Helio_UnitsType_Temperature_Celsius: {
            String retVal(SFP(HStr_Unit_Degree));
            retVal.reserve(retVal.length() + 1);
            retVal.concat('C');
            return retVal;
        }
        case Helio_UnitsType_Temperature_Fahrenheit: {
            String retVal(SFP(HStr_Unit_Degree));
            retVal.reserve(retVal.length() + 1);
            retVal.concat('F');
            return retVal;
        }
        case Helio_UnitsType_Temperature_Kelvin: {
            String retVal(SFP(HStr_Unit_Degree));
            retVal.reserve(retVal.length() + 1);
            retVal.concat('K');
            return retVal;
        }
        case Helio_UnitsType_Count:
            return !excludeSpecial ? SFP(HStr_Unit_Count) : String();
        case Helio_UnitsType_Undefined:
            break;
    }
    return !excludeSpecial ? SFP(HStr_Unit_Undefined) : String();
}

String positionIndexToString(hposi_t positionIndex, bool excludeSpecial)
{
    if (positionIndex >= 0 && positionIndex < HELIO_POS_MAXSIZE) {
        return String(positionIndex + HELIO_POS_EXPORT_BEGFROM);
    } else if (!excludeSpecial) {
        if (positionIndex == HELIO_POS_MAXSIZE) {
            return SFP(HStr_Count);
        } else {
            return SFP(HStr_Undefined);
        }
    }
    return String();
}

hposi_t positionIndexFromString(String positionIndexStr)
{
    if (positionIndexStr == positionIndexToString(HELIO_POS_MAXSIZE)) {
        return HELIO_POS_MAXSIZE;
    } else if (positionIndexStr == positionIndexToString(-1)) {
        return -1;
    } else {
        int8_t decode = positionIndexStr.toInt();
        return decode >= 0 && decode < HELIO_POS_MAXSIZE ? decode : -1;
    }
}


// All remaining methods generated from minimum spanning trie

Helio_SystemMode systemModeFromString(String systemModeStr)
{
    switch (systemModeStr.length() >= 1 ? systemModeStr[0] : '\0') {
        case 'U':
            return (Helio_SystemMode)-1;
        case 'T':
            return (Helio_SystemMode)0;
        case 'B':
            return (Helio_SystemMode)1;
        case 'C':
            return (Helio_SystemMode)2;
    }
    return Helio_SystemMode_Undefined;
}

Helio_MeasurementMode measurementModeFromString(String measurementModeStr)
{
    switch (measurementModeStr.length() >= 1 ? measurementModeStr[0] : '\0') {
        case 'C':
            return (Helio_MeasurementMode)3;
        case 'I':
            return (Helio_MeasurementMode)0;
        case 'M':
            return (Helio_MeasurementMode)1;
        case 'S':
            return (Helio_MeasurementMode)2;
        case 'U':
            return (Helio_MeasurementMode)-1;
    }
    return Helio_MeasurementMode_Undefined;
}

Helio_DisplayOutputMode displayOutputModeFromString(String displayOutModeStr)
{
    switch (displayOutModeStr.length() >= 1 ? displayOutModeStr[0] : '\0') {
        case 'U':
            return (Helio_DisplayOutputMode)-1;
        case 'D':
            return (Helio_DisplayOutputMode)0;
        case 'L':
            switch (displayOutModeStr.length() >= 4 ? displayOutModeStr[3] : '\0') {
                case '1':
                    switch (displayOutModeStr.length() >= 8 ? displayOutModeStr[7] : '\0') {
                        case '\0':
                            return (Helio_DisplayOutputMode)1;
                        case 'S':
                            return (Helio_DisplayOutputMode)2;
                    }
                    break;
                case '2':
                    switch (displayOutModeStr.length() >= 8 ? displayOutModeStr[7] : '\0') {
                        case '\0':
                            return (Helio_DisplayOutputMode)3;
                        case 'S':
                            return (Helio_DisplayOutputMode)4;
                    }
                    break;
            }
            break;
        case 'S':
            switch (displayOutModeStr.length() >= 2 ? displayOutModeStr[1] : '\0') {
                case 'S':
                    switch (displayOutModeStr.length() >= 5 ? displayOutModeStr[4] : '\0') {
                        case '3':
                            switch (displayOutModeStr.length() >= 7 ? displayOutModeStr[6] : '\0') {
                                case '5':
                                    switch (displayOutModeStr.length() >= 8 ? displayOutModeStr[7] : '\0') {
                                        case '\0':
                                            return (Helio_DisplayOutputMode)5;
                                        case 'x':
                                            switch (displayOutModeStr.length() >= 9 ? displayOutModeStr[8] : '\0') {
                                                case '3':
                                                    return (Helio_DisplayOutputMode)6;
                                                case '6':
                                                    return (Helio_DisplayOutputMode)7;
                                            }
                                            break;
                                    }
                                    break;
                                case '6':
                                    return (Helio_DisplayOutputMode)8;
                            }
                            break;
                        case '6':
                            switch (displayOutModeStr.length() >= 8 ? displayOutModeStr[7] : '\0') {
                                case 'G':
                                    return (Helio_DisplayOutputMode)10;
                                case 'W':
                                    return (Helio_DisplayOutputMode)11;
                            }
                            break;
                    }
                    break;
                case 'H':
                    return (Helio_DisplayOutputMode)9;
                case 'T':
                    switch (displayOutModeStr.length() >= 5 ? displayOutModeStr[4] : '\0') {
                        case '3':
                            return (Helio_DisplayOutputMode)14;
                        case '8':
                            return (Helio_DisplayOutputMode)15;
                    }
                    break;
            }
            break;
        case 'I':
            switch (displayOutModeStr.length() >= 3 ? displayOutModeStr[2] : '\0') {
                case '3':
                    switch (displayOutModeStr.length() >= 7 ? displayOutModeStr[6] : '\0') {
                        case '\0':
                            return (Helio_DisplayOutputMode)12;
                        case 'V':
                            return (Helio_DisplayOutputMode)13;
                    }
                    break;
                case 'I':
                    return (Helio_DisplayOutputMode)16;
            }
            break;
        case 'P':
            return (Helio_DisplayOutputMode)17;
        case 'T':
            return (Helio_DisplayOutputMode)18;
        case 'C':
            return (Helio_DisplayOutputMode)19;
    }
    return Helio_DisplayOutputMode_Undefined;
}

Helio_ControlInputMode controlInputModeFromString(String controlInModeStr)
{
    switch (controlInModeStr.length() >= 1 ? controlInModeStr[0] : '\0') {
        case 'U':
            switch (controlInModeStr.length() >= 2 ? controlInModeStr[1] : '\0') {
                case 'n':
                    return (Helio_ControlInputMode)-1;
                case 'p':
                    switch (controlInModeStr.length() >= 16 ? controlInModeStr[15] : '\0') {
                        case '\0':
                            return (Helio_ControlInputMode)3;
                        case 'L':
                            return (Helio_ControlInputMode)4;
                    }
                    break;
            }
            break;
        case 'D':
            return (Helio_ControlInputMode)0;
        case 'R':
            switch (controlInModeStr.length() >= 2 ? controlInModeStr[1] : '\0') {
                case 'o':
                    switch (controlInModeStr.length() >= 16 ? controlInModeStr[15] : '\0') {
                        case '\0':
                            return (Helio_ControlInputMode)1;
                        case 'L':
                            return (Helio_ControlInputMode)2;
                    }
                    break;
                case 'e':
                    switch (controlInModeStr.length() >= 3 ? controlInModeStr[2] : '\0') {
                        case 's':
                            return (Helio_ControlInputMode)10;
                        case 'm':
                            return (Helio_ControlInputMode)13;
                    }
                    break;
            }
            break;
        case 'A':
            return (Helio_ControlInputMode)5;
        case '3':
            switch (controlInModeStr.length() >= 12 ? controlInModeStr[11] : '\0') {
                case '\0':
                    return (Helio_ControlInputMode)6;
                case 'L':
                    return (Helio_ControlInputMode)7;
            }
            break;
        case '4':
            switch (controlInModeStr.length() >= 12 ? controlInModeStr[11] : '\0') {
                case '\0':
                    return (Helio_ControlInputMode)8;
                case 'L':
                    return (Helio_ControlInputMode)9;
            }
            break;
        case 'T':
            switch (controlInModeStr.length() >= 2 ? controlInModeStr[1] : '\0') {
                case 'o':
                    return (Helio_ControlInputMode)11;
                case 'F':
                    return (Helio_ControlInputMode)12;
            }
            break;
        case 'C':
            return (Helio_ControlInputMode)14;
    }
    return Helio_ControlInputMode_Undefined;
}

Helio_ActuatorType actuatorTypeFromString(String actuatorTypeStr)
{
    switch (actuatorTypeStr.length() >= 1 ? actuatorTypeStr[0] : '\0') {
        case 'C':
            switch (actuatorTypeStr.length() >= 3 ? actuatorTypeStr[2] : '\0') {
                case 'n':
                    return (Helio_ActuatorType)0;
                case 'u':
                    return (Helio_ActuatorType)7;
            }
            break;
        case 'L':
            return (Helio_ActuatorType)1;
        case 'P':
            switch (actuatorTypeStr.length() >= 2 ? actuatorTypeStr[1] : '\0') {
                case 'a':
                    switch (actuatorTypeStr.length() >= 6 ? actuatorTypeStr[5] : '\0') {
                        case 'B':
                            return (Helio_ActuatorType)2;
                        case 'C':
                            return (Helio_ActuatorType)3;
                        case 'H':
                            return (Helio_ActuatorType)4;
                        case 'S':
                            return (Helio_ActuatorType)5;
                    }
                    break;
                case 'o':
                    return (Helio_ActuatorType)6;
            }
            break;
        case 'U':
            return (Helio_ActuatorType)-1;
    }
    return Helio_ActuatorType_Undefined;
}

Helio_SensorType sensorTypeFromString(String sensorTypeStr)
{
        switch (sensorTypeStr.length() >= 1 ? sensorTypeStr[0] : '\0') {
        case 'C':
            return (Helio_SensorType)8;
        case 'I':
            return (Helio_SensorType)0;
        case 'L':
            return (Helio_SensorType)1;
        case 'P':
            switch (sensorTypeStr.length() >= 6 ? sensorTypeStr[5] : '\0') {
                case 'P':
                    return (Helio_SensorType)2;
                case 'U':
                    return (Helio_SensorType)3;
            }
            break;
        case 'T':
            switch (sensorTypeStr.length() >= 2 ? sensorTypeStr[1] : '\0') {
                case 'e':
                    return (Helio_SensorType)4;
                case 'i':
                    return (Helio_SensorType)5;
                case 'r':
                    return (Helio_SensorType)6;
            }
            break;
        case 'U':
            return (Helio_SensorType)-1;
        case 'W':
            return (Helio_SensorType)7;
    }
    return Helio_SensorType_Undefined;
}

Helio_PanelType panelTypeFromString(String panelTypeStr)
{
    switch (panelTypeStr.length() >= 1 ? panelTypeStr[0] : '\0') {
        case 'C':
            return (Helio_PanelType)4;
        case 'E':
            return (Helio_PanelType)3;
        case 'G':
            return (Helio_PanelType)2;
        case 'H':
            return (Helio_PanelType)0;
        case 'U':
            return (Helio_PanelType)-1;
        case 'V':
            return (Helio_PanelType)1;
    }
    return Helio_PanelType_Undefined;
}

Helio_RailType railTypeFromString(String railTypeStr) {
        switch (railTypeStr.length() >= 1 ? railTypeStr[0] : '\0') {
        case 'A':
            switch (railTypeStr.length() >= 3 ? railTypeStr[2] : '\0') {
                case '1':
                    return (Helio_RailType)0;
                case '2':
                    return (Helio_RailType)1;
            }
            break;
        case 'C':
            return (Helio_RailType)7;
        case 'D':
            switch (railTypeStr.length() >= 3 ? railTypeStr[2] : '\0') {
                case '1':
                    return (Helio_RailType)4;
                case '2':
                    return (Helio_RailType)5;
                case '3':
                    return (Helio_RailType)2;
                case '4':
                    return (Helio_RailType)6;
                case '5':
                    return (Helio_RailType)3;
            }
            break;
        case 'U':
            return (Helio_RailType)-1;
    }
    return Helio_RailType_Undefined;
}

Helio_PinMode pinModeFromString(String pinModeStr)
{
        switch (pinModeStr.length() >= 1 ? pinModeStr[0] : '\0') {
        case 'A':
            switch (pinModeStr.length() >= 7 ? pinModeStr[6] : '\0') {
                case 'I':
                    return (Helio_PinMode)5;
                case 'O':
                    return (Helio_PinMode)6;
            }
            break;
        case 'C':
            return (Helio_PinMode)7;
        case 'D':
            switch (pinModeStr.length() >= 8 ? pinModeStr[7] : '\0') {
                case 'I':
                    switch (pinModeStr.length() >= 13 ? pinModeStr[12] : '\0') {
                        case '\0':
                            return (Helio_PinMode)0;
                        case 'P':
                            switch (pinModeStr.length() >= 17 ? pinModeStr[16] : '\0') {
                                case 'D':
                                    return (Helio_PinMode)2;
                                case 'U':
                                    return (Helio_PinMode)1;
                            }
                            break;
                    }
                    break;
                case 'O':
                    switch (pinModeStr.length() >= 14 ? pinModeStr[13] : '\0') {
                        case '\0':
                            return (Helio_PinMode)3;
                        case 'P':
                            return (Helio_PinMode)4;
                    }
                    break;
            }
            break;
        case 'U':
            return (Helio_PinMode)-1;
    }
    return Helio_PinMode_Undefined;
}

Helio_EnableMode enableModeFromString(String enableModeStr)
{
    switch (enableModeStr.length() >= 1 ? enableModeStr[0] : '\0') {
        case 'A':
            switch (enableModeStr.length() >= 2 ? enableModeStr[1] : '\0') {
                case 's':
                    return (Helio_EnableMode)7;
                case 'v':
                    return (Helio_EnableMode)2;
            }
            break;
        case 'C':
            return (Helio_EnableMode)8;
        case 'D':
            return (Helio_EnableMode)6;
        case 'H':
            return (Helio_EnableMode)0;
        case 'I':
            return (Helio_EnableMode)4;
        case 'L':
            return (Helio_EnableMode)1;
        case 'M':
            return (Helio_EnableMode)3;
        case 'R':
            return (Helio_EnableMode)5;
        case 'U':
            return (Helio_EnableMode)-1;
    }
    return Helio_EnableMode_Undefined;
}

Helio_UnitsCategory unitsCategoryFromString(String unitsCategoryStr)
{
    switch (unitsCategoryStr.length() >= 1 ? unitsCategoryStr[0] : '\0') {
        case 'A':
            return (Helio_UnitsCategory)0;
        case 'C':
            return (Helio_UnitsCategory)6;
        case 'D':
            return (Helio_UnitsCategory)1;
        case 'P':
            switch (unitsCategoryStr.length() >= 2 ? unitsCategoryStr[1] : '\0') {
                case 'e':
                    return (Helio_UnitsCategory)2;
                case 'o':
                    return (Helio_UnitsCategory)3;
            }
            break;
        case 'S':
            return (Helio_UnitsCategory)4;
        case 'T':
            return (Helio_UnitsCategory)5;
        case 'U':
            return (Helio_UnitsCategory)-1;
    }
    return Helio_UnitsCategory_Undefined;
}

Helio_UnitsType unitsTypeFromSymbol(String unitsSymbolStr)
{
    switch (unitsSymbolStr.length() >= 1 ? unitsSymbolStr[0] : '\0') {
        case '%':
            return (Helio_UnitsType)1;
        case 'A':
            return (Helio_UnitsType)7;
        case 'J':
            return (Helio_UnitsType)8;
        case 'W':
            return (Helio_UnitsType)8;
        case '[':
            switch (unitsSymbolStr.length() >= 2 ? unitsSymbolStr[1] : '\0') {
                case 'q':
                    return (Helio_UnitsType)14;
                case 'u':
                    return (Helio_UnitsType)-1;
            }
            break;
        case 'f':
            switch (unitsSymbolStr.length() >= 3 ? unitsSymbolStr[2] : '\0') {
                case '\0':
                    return (Helio_UnitsType)5;
                case '/':
                    return (Helio_UnitsType)9;
            }
            break;
        case 'm':
            switch (unitsSymbolStr.length() >= 2 ? unitsSymbolStr[1] : '\0') {
                case '\0':
                    return (Helio_UnitsType)6;
                case '/':
                    return (Helio_UnitsType)10;
                case 'i':
                    return (Helio_UnitsType)4;
            }
            break;
        case 'r':
            return (Helio_UnitsType)0;
        default:
            switch (unitsSymbolStr.length() >= 3 ? unitsSymbolStr[2] : '\0') {
                case '\0':
                    return (Helio_UnitsType)2;
                case 'C':
                    return (Helio_UnitsType)11;
                case 'F':
                    return (Helio_UnitsType)12;
                case 'K':
                    return (Helio_UnitsType)13;
                case 'r':
                    return (Helio_UnitsType)3;
            }
            break;
    }
    return Helio_UnitsType_Undefined;
}

/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino System
*/

#include "Helioduino.h"

static HelioRTCInterface *_rtcSyncProvider = nullptr;
time_t rtcNow() {
    return _rtcSyncProvider ? _rtcSyncProvider->now().unixtime() : 0;
}

void handleInterrupt(pintype_t pin)
{
    if (Helioduino::_activeInstance) {
        for (auto iter = Helioduino::_activeInstance->_objects.begin(); iter != Helioduino::_activeInstance->_objects.end(); ++iter) {
            if (iter->second->isSensorType()) {
                auto sensor = static_pointer_cast<HelioSensor>(iter->second);
                if (sensor->isBinaryClass()) {
                    auto binarySensor = static_pointer_cast<HelioBinarySensor>(sensor);
                    if (binarySensor && binarySensor->getInputPin().pin == pin) { binarySensor->notifyISRTriggered(); }
                }
            }
        }
    }
}


Helioduino *Helioduino::_activeInstance = nullptr;

Helioduino::Helioduino(pintype_t piezoBuzzerPin,
                       Helio_EEPROMType eepromType, DeviceSetup eepromSetup,
                       Helio_RTCType rtcType, DeviceSetup rtcSetup,
                       DeviceSetup sdSetup,
                       DeviceSetup netSetup,
                       DeviceSetup gpsSetup,
                       pintype_t *ctrlInputPins,
                       DeviceSetup lcdSetup)
    : _piezoBuzzerPin(piezoBuzzerPin),
      _eepromType(eepromType), _eepromSetup(eepromSetup), _eeprom(nullptr), _eepromBegan(false),
      _rtcType(rtcType), _rtcSetup(rtcSetup), _rtc(nullptr), _rtcBegan(false), _rtcBattFail(false),
      _sdSetup(sdSetup), _sd(nullptr), _sdBegan(false), _sdOut(0),
#ifdef HELIO_USE_NET
      _netSetup(netSetup), _netBegan(false),
#endif
#ifdef HELIO_USE_GPS
      _gpsSetup(gpsSetup), _gps(nullptr), _gpsBegan(false),
#endif
#ifdef HELIO_USE_GUI
      _activeUIInstance(nullptr), _ctrlInputPins(ctrlInputPins), _lcdSetup(lcdSetup),
#endif
#ifdef HELIO_USE_MULTITASKING
      _controlTaskId(TASKMGR_INVALIDID), _dataTaskId(TASKMGR_INVALIDID), _miscTaskId(TASKMGR_INVALIDID),
#endif
      _systemData(nullptr), _suspend(true), _pollingFrame(0), _lastSpaceCheck(0), _lastAutosave(0),
      _sysConfigFilename(SFP(HStr_Default_ConfigFilename)), _sysDataAddress(-1)
{
    _activeInstance = this;
}

Helioduino::~Helioduino()
{
    suspend();
#ifdef HELIO_USE_GUI
    if (_activeUIInstance) { delete _activeUIInstance; _activeUIInstance = nullptr; }
#endif
    deselectPinMuxers();
    while (_objects.size()) { _objects.erase(_objects.begin()); }
    while (_oneWires.size()) { dropOneWireForPin(_oneWires.begin()->first); }
    while (_pinMuxers.size()) { _pinMuxers.erase(_pinMuxers.begin()); }
    deallocateEEPROM();
    deallocateRTC();
    deallocateSD();
#ifdef HELIO_USE_GPS
    deallocateGPS();
#endif
    if (this == _activeInstance) { _activeInstance = nullptr; }
    if (_systemData) { delete _systemData; _systemData = nullptr; }
}

void Helioduino::allocateEEPROM()
{
    if (!_eeprom && _eepromType != Helio_EEPROMType_None && _eepromSetup.cfgType == DeviceSetup::I2CSetup) {
        _eeprom = new I2C_eeprom(_eepromSetup.cfgAs.i2c.address | HELIO_SYS_I2CEEPROM_BASEADDR,
                                 getEEPROMSize(), _eepromSetup.cfgAs.i2c.wire);
        _eepromBegan = false;
        HELIO_SOFT_ASSERT(_eeprom, SFP(HStr_Err_AllocationFailure));
    }
}

void Helioduino::deallocateEEPROM()
{
    if (_eeprom) {
        delete _eeprom; _eeprom = nullptr;
        _eepromBegan = false;
    }
}

void Helioduino::allocateRTC()
{
    if (!_rtc && _rtcType != Helio_RTCType_None && _rtcSetup.cfgType == DeviceSetup::I2CSetup) {
        switch (_rtcType) {
            case Helio_RTCType_DS1307:
                _rtc = new HelioRTCWrapper<RTC_DS1307>();
                break;
            case Helio_RTCType_DS3231:
                _rtc = new HelioRTCWrapper<RTC_DS3231>();
                break;
            case Helio_RTCType_PCF8523:
                _rtc = new HelioRTCWrapper<RTC_PCF8523>();
                break;
            case Helio_RTCType_PCF8563:
                _rtc = new HelioRTCWrapper<RTC_PCF8563>();
                break;
            default: break;
        }
        _rtcBegan = false;
        HELIO_SOFT_ASSERT(_rtc, SFP(HStr_Err_AllocationFailure));
        HELIO_HARD_ASSERT(_rtcSetup.cfgAs.i2c.address == B000, F("RTClib does not support i2c multi-addressing, only i2c address B000 may be used"));
    }
}

void Helioduino::deallocateRTC()
{
    if (_rtc) {
        if (_rtcSyncProvider == _rtc) { setSyncProvider(nullptr); _rtcSyncProvider = nullptr; }
        delete _rtc; _rtc = nullptr;
        _rtcBegan = false;
    }
}

void Helioduino::allocateSD()
{
    if (!_sd && _sdSetup.cfgType == DeviceSetup::SPISetup) {
        #if !(defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SD))
            _sd = &SD;
        #else
            _sd = new SDClass();
        #endif
        _sdBegan = false;
        HELIO_SOFT_ASSERT(_sd, SFP(HStr_Err_AllocationFailure));
    }
}

void Helioduino::deallocateSD()
{
    if (_sd) {
        #if !(defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SD))
            _sd = nullptr;
        #else
            delete _sd; _sd = nullptr;
        #endif
        _sdBegan = false;
    }
}

#ifdef HELIO_USE_GPS

void Helioduino::allocateGPS()
{
    if (!_gps && _gpsSetup.cfgType != DeviceSetup::None) {
        switch (_gpsSetup.cfgType) {
            case DeviceSetup::UARTSetup:
                _gps = new GPSClass(_gpsSetup.cfgAs.uart.serial);
                break;
            case DeviceSetup::I2CSetup:
                _gps = new GPSClass(_gpsSetup.cfgAs.i2c.wire);
                break;
            case DeviceSetup::SPISetup:
                _gps = new GPSClass(_gpsSetup.cfgAs.spi.spi, _gpsSetup.cfgAs.spi.cs);
                break;
            default: break;
        }
        _gpsBegan = false;
        HELIO_SOFT_ASSERT(_gps, SFP(HStr_Err_AllocationFailure));
    }
}

void Helioduino::deallocateGPS()
{
    if (_gps) {
        delete _gps; _gps = nullptr;
        _gpsBegan = false;
    }
}

#endif

void Helioduino::init(Helio_SystemMode systemMode,
                      Helio_MeasurementMode measureMode,
                      Helio_DisplayOutputMode dispOutMode,
                      Helio_ControlInputMode ctrlInMode)
{
    HELIO_HARD_ASSERT(!_systemData, SFP(HStr_Err_AlreadyInitialized));

    if (!_systemData) {
        commonPreInit();

        HELIO_SOFT_ASSERT((int)systemMode >= 0 && systemMode < Helio_SystemMode_Count, SFP(HStr_Err_InvalidParameter));
        HELIO_SOFT_ASSERT((int)measureMode >= 0 && measureMode < Helio_MeasurementMode_Count, SFP(HStr_Err_InvalidParameter));
        #ifdef HELIO_USE_GUI
            HELIO_SOFT_ASSERT((int)dispOutMode >= 0 && dispOutMode < Helio_DisplayOutputMode_Count, SFP(HStr_Err_InvalidParameter));
            HELIO_SOFT_ASSERT((int)ctrlInMode >= 0 && ctrlInMode < Helio_ControlInputMode_Count, SFP(HStr_Err_InvalidParameter));
        #endif

        _systemData = new HelioSystemData();
        HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_AllocationFailure));

        if (_systemData) {
            _systemData->systemMode = systemMode;
            _systemData->measureMode = measureMode;
            #ifdef HELIO_USE_GUI
                _systemData->dispOutMode = dispOutMode;
                _systemData->ctrlInMode = ctrlInMode;
            #else
                _systemData->dispOutMode = Helio_DisplayOutputMode_Disabled;
                _systemData->ctrlInMode = Helio_ControlInputMode_Disabled;
            #endif

            commonPostInit();
        }
    }  
}

bool Helioduino::initFromEEPROM(bool jsonFormat)
{
    HELIO_HARD_ASSERT(!_systemData, SFP(HStr_Err_AlreadyInitialized));

    if (!_systemData) {
        commonPreInit();

        if (getEEPROM() && _eepromBegan && _sysDataAddress != -1) {
            HelioEEPROMStream eepromStream(_sysDataAddress, getEEPROMSize() - _sysDataAddress);
            return jsonFormat ? initFromJSONStream(&eepromStream) : initFromBinaryStream(&eepromStream);
        }
    }

    return false;
}

bool Helioduino::saveToEEPROM(bool jsonFormat)
{
    HELIO_HARD_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));

    if (_systemData) {
        if (getEEPROM() && _eepromBegan && _sysDataAddress != -1) {
            HelioEEPROMStream eepromStream(_sysDataAddress, getEEPROMSize() - _sysDataAddress);
            return jsonFormat ? saveToJSONStream(&eepromStream) : saveToBinaryStream(&eepromStream);
        }
    }

    return false;
}

bool Helioduino::initFromSDCard(bool jsonFormat)
{
    HELIO_HARD_ASSERT(!_systemData, SFP(HStr_Err_AlreadyInitialized));

    if (!_systemData) {
        commonPreInit();
        auto sd = getSDCard();

        if (sd) {
            bool retVal = false;
            auto configFile = sd->open(_sysConfigFilename.c_str(), FILE_READ);

            if (configFile) {
                retVal = jsonFormat ? initFromJSONStream(&configFile) : initFromBinaryStream(&configFile);

                configFile.close();
            }

            endSDCard(sd);
            return retVal;
        }
    }

    return false;
}

bool Helioduino::saveToSDCard(bool jsonFormat)
{
    HELIO_HARD_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));

    if (!_systemData) {
        auto sd = getSDCard();

        if (sd) {
            bool retVal = false;
            auto configFile = sd->open(_sysConfigFilename.c_str(), FILE_READ);

            if (configFile) {
                retVal = jsonFormat ? saveToJSONStream(&configFile, false) : saveToBinaryStream(&configFile);

                configFile.flush();
                configFile.close();
            }

            endSDCard(sd);
            return retVal;
        }
    }

    return false;
}

#ifdef HELIO_USE_WIFI_STORAGE

bool Helioduino::initFromWiFiStorage(bool jsonFormat)
{
    HELIO_HARD_ASSERT(!_systemData, SFP(HStr_Err_AlreadyInitialized));

    if (!_systemData) {
        commonPreInit();

        auto configFile = WiFiStorage.open(_sysConfigFilename.c_str());

        if (configFile) {
            auto configFileStream = HelioWiFiStorageFileStream(configFile);
            return jsonFormat ? initFromJSONStream(&configFileStream) : initFromBinaryStream(&configFileStream);

            configFile.close();
        }
    }

    return false;
}

bool Helioduino::saveToWiFiStorage(bool jsonFormat)
{
    HELIO_HARD_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));

    if (_systemData) {
        if (WiFiStorage.exists(_sysConfigFilename.c_str())) {
            WiFiStorage.remove(_sysConfigFilename.c_str());
        }
        auto configFile = WiFiStorage.open(_sysConfigFilename.c_str());

        if (configFile) {
            auto configFileStream = HelioWiFiStorageFileStream(configFile);
            return jsonFormat ? saveToJSONStream(&configFileStream, false) : saveToBinaryStream(&configFileStream);

            configFileStream.flush();
            configFile.close();
        }
    }

    return false;
}

#endif

bool Helioduino::initFromJSONStream(Stream *streamIn)
{
    HELIO_HARD_ASSERT(!_systemData, SFP(HStr_Err_AlreadyInitialized));
    HELIO_SOFT_ASSERT(streamIn && streamIn->available(), SFP(HStr_Err_InvalidParameter));

    if (!_systemData && streamIn && streamIn->available()) {
        commonPreInit();

        {   StaticJsonDocument<HELIO_JSON_DOC_SYSSIZE> doc;
            deserializeJson(doc, *streamIn);
            JsonObjectConst systemDataObj = doc.as<JsonObjectConst>();
            HelioSystemData *systemData = (HelioSystemData *)newDataFromJSONObject(systemDataObj);

            HELIO_SOFT_ASSERT(systemData && systemData->isSystemData(), SFP(HStr_Err_ImportFailure));
            if (systemData && systemData->isSystemData()) {
                _systemData = systemData;
            } else if (systemData) {
                delete systemData;
            }
        }

        if (_systemData) {
            while (streamIn->available()) {
                StaticJsonDocument<HELIO_JSON_DOC_DEFSIZE> doc;
                deserializeJson(doc, *streamIn);
                JsonObjectConst dataObj = doc.as<JsonObjectConst>();
                HelioData *data = newDataFromJSONObject(dataObj);

                HELIO_SOFT_ASSERT(data && (data->isStandardData() || data->isObjectData()), SFP(HStr_Err_ImportFailure));
                if (data && data->isStandardData()) {
                    if (data->isCalibrationData()) {
                        setUserCalibrationData((HelioCalibrationData *)data);
                    }
                    delete data; data = nullptr;
                } else if (data && data->isObjectData()) {
                    HelioObject *obj = newObjectFromData(data);
                    delete data; data = nullptr;

                    if (obj && !obj->isUnknownType()) {
                        _objects[obj->getKey()] = SharedPtr<HelioObject>(obj);
                    } else {
                        HELIO_SOFT_ASSERT(false, SFP(HStr_Err_ImportFailure));
                        if (obj) { delete obj; }
                        delete _systemData; _systemData = nullptr;
                        break;
                    }
                } else {
                    if (data) { delete data; data = nullptr; }
                    delete _systemData; _systemData = nullptr;
                    break;
                }
            }
        }

        HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_InitializationFailure));
        if (_systemData) { commonPostInit(); }
        return _systemData;
    }

    return false;
}

bool Helioduino::saveToJSONStream(Stream *streamOut, bool compact)
{
    HELIO_HARD_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(streamOut, SFP(HStr_Err_InvalidParameter));

    if (_systemData && streamOut) {
        {   StaticJsonDocument<HELIO_JSON_DOC_SYSSIZE> doc;

            JsonObject systemDataObj = doc.to<JsonObject>();
            _systemData->toJSONObject(systemDataObj);

            if (!(compact ? serializeJson(doc, *streamOut) : serializeJsonPretty(doc, *streamOut))) {
                HELIO_SOFT_ASSERT(false, SFP(HStr_Err_ExportFailure));
                return false;
            }
        }

        if (hasUserCalibrations()) {
            for (auto iter = _calibrationData.begin(); iter != _calibrationData.end(); ++iter) {
                StaticJsonDocument<HELIO_JSON_DOC_DEFSIZE> doc;

                JsonObject calibDataObj = doc.to<JsonObject>();
                iter->second->toJSONObject(calibDataObj);

                if (!(compact ? serializeJson(doc, *streamOut) : serializeJsonPretty(doc, *streamOut))) {
                    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_ExportFailure));
                    return false;
                }
            }
        }

        if (_objects.size()) {
            for (auto iter = _objects.begin(); iter != _objects.end(); ++iter) {
                HelioData *data = iter->second->newSaveData();

                HELIO_SOFT_ASSERT(data && data->isObjectData(), SFP(HStr_Err_AllocationFailure));
                if (data && data->isObjectData()) {
                    StaticJsonDocument<HELIO_JSON_DOC_DEFSIZE> doc;

                    JsonObject objectDataObj = doc.to<JsonObject>();
                    data->toJSONObject(objectDataObj);
                    delete data; data = nullptr;

                    if (!(compact ? serializeJson(doc, *streamOut) : serializeJsonPretty(doc, *streamOut))) {
                        HELIO_SOFT_ASSERT(false, SFP(HStr_Err_ExportFailure));
                        return false;
                    }
                } else {
                    if (data) { delete data; data = nullptr; }
                    return false;
                }
            }
        }

        commonPostSave();
        return true;
    }

    return false;
}

bool Helioduino::initFromBinaryStream(Stream *streamIn)
{
    HELIO_HARD_ASSERT(!_systemData, SFP(HStr_Err_AlreadyInitialized));
    HELIO_SOFT_ASSERT(streamIn && streamIn->available(), SFP(HStr_Err_InvalidParameter));

    if (!_systemData && streamIn && streamIn->available()) {
        commonPreInit();

        {   HelioSystemData *systemData = (HelioSystemData *)newDataFromBinaryStream(streamIn);

            HELIO_SOFT_ASSERT(systemData && systemData->isSystemData(), SFP(HStr_Err_ImportFailure));
            if (systemData && systemData->isSystemData()) {
                _systemData = systemData;
            } else if (systemData) {
                delete systemData;
            }
        }

        if (_systemData) {
            while (streamIn->available()) {
                HelioData *data = newDataFromBinaryStream(streamIn);

                HELIO_SOFT_ASSERT(data && (data->isStandardData() || data->isObjectData()), SFP(HStr_Err_AllocationFailure));
                if (data && data->isStandardData()) {
                    if (data->isCalibrationData()) {
                        setUserCalibrationData((HelioCalibrationData *)data);
                    }
                    delete data; data = nullptr;
                } else if (data && data->isObjectData()) {
                    HelioObject *obj = newObjectFromData(data);
                    delete data; data = nullptr;

                    if (obj && !obj->isUnknownType()) {
                        _objects[obj->getKey()] = SharedPtr<HelioObject>(obj);
                    } else {
                        HELIO_SOFT_ASSERT(false, SFP(HStr_Err_ImportFailure));
                        if (obj) { delete obj; }
                        delete _systemData; _systemData = nullptr;
                        break;
                    }
                } else {
                    if (data) { delete data; data = nullptr; }    
                    delete _systemData; _systemData = nullptr;
                    break;
                }
            }
        }

        HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_InitializationFailure));
        if (_systemData) { commonPostInit(); }
        return _systemData;
    }

    return false;
}

bool Helioduino::saveToBinaryStream(Stream *streamOut)
{
    HELIO_HARD_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    HELIO_SOFT_ASSERT(streamOut, SFP(HStr_Err_InvalidParameter));

    if (_systemData && streamOut) {
        {   size_t bytesWritten = serializeDataToBinaryStream(_systemData, streamOut);

            HELIO_SOFT_ASSERT(!bytesWritten, SFP(HStr_Err_ExportFailure));
            if (!bytesWritten) { return false; }
        }

        if (hasUserCalibrations()) {
            size_t bytesWritten = 0;

            for (auto iter = _calibrationData.begin(); iter != _calibrationData.end(); ++iter) {
                bytesWritten += serializeDataToBinaryStream(iter->second, streamOut);
            }

            HELIO_SOFT_ASSERT(bytesWritten, SFP(HStr_Err_ExportFailure));
            if (!bytesWritten) { return false; }
        }

        if (_objects.size()) {
            for (auto iter = _objects.begin(); iter != _objects.end(); ++iter) {
                HelioData *data = iter->second->newSaveData();

                HELIO_SOFT_ASSERT(data && data->isObjectData(), SFP(HStr_Err_AllocationFailure));
                if (data && data->isObjectData()) {
                    size_t bytesWritten = serializeDataToBinaryStream(data, streamOut);
                    delete data; data = nullptr;

                    HELIO_SOFT_ASSERT(bytesWritten, SFP(HStr_Err_ExportFailure));
                    if (!bytesWritten) { return false; }
                } else {
                    if (data) { delete data; data = nullptr; }
                    return false;
                }
            }
        }

        commonPostSave();
        return true;
    }

    return false;
}

void Helioduino::commonPreInit()
{
    Map<uintptr_t,uint32_t> began;

    if (isValidPin(_piezoBuzzerPin)) {
        pinMode(_piezoBuzzerPin, OUTPUT);
        #ifdef ESP32
            ledcSetup(0, 0, 10);
            ledcAttachPin(_piezoBuzzerPin, 0);
        #elif !defined(ARDUINO_SAM_DUE)
            noTone(_piezoBuzzerPin);
        #else
            digitalWrite(_piezoBuzzerPin, 0);
        #endif
    }
    if (_eepromSetup.cfgType == DeviceSetup::I2CSetup) {
        if (began.find((uintptr_t)_eepromSetup.cfgAs.i2c.wire) == began.end() || _eepromSetup.cfgAs.i2c.speed < began[(uintptr_t)_eepromSetup.cfgAs.i2c.wire]) {
            _eepromSetup.cfgAs.i2c.wire->begin();
            _eepromSetup.cfgAs.i2c.wire->setClock((began[(uintptr_t)_eepromSetup.cfgAs.i2c.wire] = _eepromSetup.cfgAs.i2c.speed));
        }
    }
    if (_rtcSetup.cfgType == DeviceSetup::I2CSetup) {
        if (began.find((uintptr_t)_rtcSetup.cfgAs.i2c.wire) == began.end() || _rtcSetup.cfgAs.i2c.speed < began[(uintptr_t)_rtcSetup.cfgAs.i2c.wire]) {
            _rtcSetup.cfgAs.i2c.wire->begin();
            _rtcSetup.cfgAs.i2c.wire->setClock((began[(uintptr_t)_rtcSetup.cfgAs.i2c.wire] = _rtcSetup.cfgAs.i2c.speed));
        }
    }
    #ifdef HELIO_USE_GUI
        if (_lcdSetup.cfgType == DeviceSetup::I2CSetup) {
            if (began.find((uintptr_t)_lcdSetup.cfgAs.i2c.wire) == began.end() || _lcdSetup.cfgAs.i2c.speed < began[(uintptr_t)_lcdSetup.cfgAs.i2c.wire]) {
                _lcdSetup.cfgAs.i2c.wire->begin();
                _lcdSetup.cfgAs.i2c.wire->setClock((began[(uintptr_t)_lcdSetup.cfgAs.i2c.wire] = _lcdSetup.cfgAs.i2c.speed));
            }
        }
    #endif
    if (_sdSetup.cfgType == DeviceSetup::SPISetup) {
        if (began.find((uintptr_t)_rtcSetup.cfgAs.spi.spi) == began.end()) {
            _sdSetup.cfgAs.spi.spi->begin();
            began[(uintptr_t)_rtcSetup.cfgAs.spi.spi] = 0;
        }
        if (isValidPin(_sdSetup.cfgAs.spi.cs)) {
            pinMode(_sdSetup.cfgAs.spi.cs, OUTPUT);
            digitalWrite(_sdSetup.cfgAs.spi.cs, HIGH);
        }
    }
    #ifdef HELIO_USE_NET
        if (_netSetup.cfgType == DeviceSetup::SPISetup) {
            if (began.find((uintptr_t)_netSetup.cfgAs.spi.spi) == began.end()) {
                _netSetup.cfgAs.spi.spi->begin();
                began[(uintptr_t)_netSetup.cfgAs.spi.spi] = 0;
            }
            if (isValidPin(_netSetup.cfgAs.spi.cs)) {
                pinMode(_netSetup.cfgAs.spi.cs, OUTPUT);
                digitalWrite(_netSetup.cfgAs.spi.cs, HIGH);
            }
            #ifdef HELIO_USE_ETHERNET
                Ethernet.init(_netSetup.cfgAs.spi.cs);
            #endif
        } else if (_netSetup.cfgType == DeviceSetup::UARTSetup) {
            if (began.find((uintptr_t)_netSetup.cfgAs.uart.serial) == began.end() || _netSetup.cfgAs.uart.baud < began[(uintptr_t)_netSetup.cfgAs.uart.serial]) {
                _netSetup.cfgAs.uart.serial->begin((began[(uintptr_t)_netSetup.cfgAs.uart.serial] = _netSetup.cfgAs.uart.baud), (uartmode_t)HELIO_SYS_ATWIFI_SERIALMODE);
            }
            #ifdef HELIO_USE_AT_WIFI
                WiFi.init(_netSetup.cfgAs.uart.serial);
            #endif
        }
    #endif
    #ifdef HELIO_USE_WIFI_STORAGE
        //WiFiStorage.begin();
    #endif
    #ifdef HELIO_USE_MULTITASKING
        taskManager.setInterruptCallback(&handleInterrupt);
    #endif
}

#ifdef HELIO_USE_VERBOSE_OUTPUT
static void printDeviceSetup(String prefix, const DeviceSetup &devSetup)
{
    switch(devSetup.cfgType) {
        case DeviceSetup::I2CSetup:
            Serial.print(','); Serial.print(' '); Serial.print(prefix); Serial.print(F("I2CAddress: 0x"));
            Serial.print(devSetup.cfgAs.i2c.address, HEX);
            Serial.print(','); Serial.print(' '); Serial.print(prefix); Serial.print(F("I2CSpeed: "));
            Serial.print(roundf(devSetup.cfgAs.i2c.speed / 1000.0f)); Serial.print(F("kHz"));
            break;

        case DeviceSetup::SPISetup:
            Serial.print(','); Serial.print(' '); Serial.print(prefix); Serial.print(F("SPICSPin: "));
            if (isValidPin(devSetup.cfgAs.spi.cs)) { Serial.print(devSetup.cfgAs.spi.cs); }
            else { Serial.print(SFP(HStr_Disabled)); }
            Serial.print(','); Serial.print(' '); Serial.print(prefix); Serial.print(F("SPISpeed: "));
            Serial.print(roundf(devSetup.cfgAs.spi.speed / 1000000.0f)); Serial.print(F("MHz"));
            break;

        case DeviceSetup::UARTSetup:
            Serial.print(','); Serial.print(' '); Serial.print(prefix); Serial.print(F("UARTBaud: "));
            Serial.print(devSetup.cfgAs.uart.baud); Serial.print(F("bps"));
            break;

        default:
            Serial.print(','); Serial.print(' '); Serial.print(prefix); Serial.print(':'); Serial.print(' ');
            Serial.print(SFP(HStr_Disabled));
            break;
    }
}
#endif

void Helioduino::commonPostInit()
{
    if ((_rtcSyncProvider = getRTC())) {
        setSyncProvider(rtcNow);
    }

    scheduler.updateDayTracking(); // also calls setNeedsScheduling
    logger.updateInitTracking();
    publisher.setNeedsTabulation();

    #ifdef HELIO_USE_WIFI
        if (!_systemData->wifiPasswordSeed && _systemData->wifiPassword[0]) {
            setWiFiConnection(getWiFiSSID(), getWiFiPassword()); // sets seed and encrypts
        }
    #endif

    #ifdef HELIO_USE_VERBOSE_OUTPUT
        #if 1 // set to 0 if you just want this gone
            Serial.print(F("Helioduino::commonPostInit piezoBuzzerPin: "));
            if (isValidPin(_piezoBuzzerPin)) { Serial.print(_piezoBuzzerPin); }
            else { Serial.print(SFP(HStr_Disabled)); }
            Serial.print(F(", eepromSize: "));
            if (getEEPROMSize()) { Serial.print(getEEPROMSize()); }
            else { Serial.print(SFP(HStr_Disabled)); }
            printDeviceSetup(F("eeprom"), _eepromSetup);
            Serial.print(F(", rtcType: "));
            if (_rtcType != Helio_RTCType_None) { Serial.print(_rtcType); }
            else { Serial.print(SFP(HStr_Disabled)); }
            printDeviceSetup(F("rtc"), _rtcSetup);
            printDeviceSetup(F("sd"), _sdSetup);
            #ifdef HELIO_USE_NET
                printDeviceSetup(F("net"), _netSetup);
            #endif
            #ifdef HELIO_USE_GUI
                Serial.print(F(", controlInputPins: "));
                if (getControlInputPins() && _ctrlInputPins && isValidPin(_ctrlInputPins[0])) {
                    Serial.print('{');
                    for (int i = 0; i < getControlInputPins(); ++i) {
                        if (i) { Serial.print(','); }
                        Serial.print(_ctrlInputPins[i]);
                    }
                    Serial.print('}');
                }
                else { Serial.print(SFP(HStr_Disabled)); }
                printDeviceSetup(F("lcd"), _lcdSetup);
            #endif
            Serial.print(F(", systemMode: "));
            Serial.print(systemModeToString(getSystemMode()));
            Serial.print(F(", measureMode: "));
            Serial.print(measurementModeToString(getMeasurementMode()));
            Serial.print(F(", dispOutMode: "));
            Serial.print(displayOutputModeToString(getDisplayOutputMode()));
            Serial.print(F(", ctrlInMode: "));
            Serial.print(controlInputModeToString(getControlInputMode()));
            Serial.println(); flushYield();
        #endif
    #endif // /ifdef HELIO_USE_VERBOSE_OUTPUT
}

void Helioduino::commonPostSave()
{
    logger.logSystemSave();

    if (hasUserCalibrations()) {
        for (auto iter = _calibrationData.begin(); iter != _calibrationData.end(); ++iter) {
            iter->second->_unsetModded();
        }
    }
}

void controlLoop()
{
    if (Helioduino::_activeInstance && !Helioduino::_activeInstance->_suspend) {
        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("controlLoop")); flushYield();
        #endif

        for (auto iter = Helioduino::_activeInstance->_objects.begin(); iter != Helioduino::_activeInstance->_objects.end(); ++iter) {
            iter->second->update();
        }

        Helioduino::_activeInstance->scheduler.update();

        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("~controlLoop")); flushYield();
        #endif
    }

    yield();
}

void dataLoop()
{
    if (Helioduino::_activeInstance && !Helioduino::_activeInstance->_suspend) {
        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("dataLoop")); flushYield();
        #endif

        Helioduino::_activeInstance->publisher.advancePollingFrame();

        for (auto iter = Helioduino::_activeInstance->_objects.begin(); iter != Helioduino::_activeInstance->_objects.end(); ++iter) {
            if (iter->second->isSensorType()) {
                auto sensor = static_pointer_cast<HelioSensor>(iter->second);
                if (sensor->getNeedsPolling()) {
                    sensor->takeMeasurement(); // no force if already current for this frame #, we're just ensuring data for publisher
                }
            }
        }

        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("~dataLoop")); flushYield();
        #endif
    }

    yield();
}

void miscLoop()
{
    if (Helioduino::_activeInstance && !Helioduino::_activeInstance->_suspend) {
        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("miscLoop")); flushYield();
        #endif

        Helioduino::_activeInstance->checkFreeMemory();
        Helioduino::_activeInstance->checkFreeSpace();
        Helioduino::_activeInstance->checkAutosave();

        Helioduino::_activeInstance->publisher.update();

        #ifdef HELIO_USE_GPS // FIXME: This may get removed if it doesn't work right, but it's probably close.
            if (Helioduino::_gps && Helioduino::_gps->newNMEAreceived()) {
                Helioduino::_gps->parse(Helioduino::_gps->lastNMEA());
                // TODO: Update lat/long of controller (trigger possible event change, possibly back behind a frequency timer).
            }
        #endif

        #if HELIO_SYS_MEM_LOGGING_ENABLE
        {   static time_t _lastMemLog = unixNow();
            if (unixNow() >= _lastMemLog + 15) {
                _lastMemLog = unixNow();
                Helioduino::_activeInstance->logger.logMessage(String(F("Free memory: ")), String(freeMemory()));
            }
        }
        #endif

        #ifdef HELIO_USE_VERBOSE_OUTPUT
            Serial.println(F("~miscLoop")); flushYield();
        #endif
    }

    yield();
}

void Helioduino::launch()
{
    // Forces all sensors to get a new measurement
    publisher.advancePollingFrame();

    // Create/enable main runloops
    _suspend = false;
    #ifdef HELIO_USE_MULTITASKING
        if (!isValidTask(_controlTaskId)) {
            _controlTaskId = taskManager.scheduleFixedRate(HELIO_CONTROL_LOOP_INTERVAL, controlLoop);
        } else {
            taskManager.setTaskEnabled(_controlTaskId, true);
        }
        if (!isValidTask(_dataTaskId)) {
            _dataTaskId = taskManager.scheduleFixedRate(getPollingInterval(), dataLoop);
        } else {
            taskManager.setTaskEnabled(_dataTaskId, true);
        }
        if (!isValidTask(_miscTaskId)) {
            _miscTaskId = taskManager.scheduleFixedRate(HELIO_MISC_LOOP_INTERVAL, miscLoop);
        } else {
            taskManager.setTaskEnabled(_miscTaskId, true);
        }
    #endif

    #ifdef HELIO_USE_VERBOSE_OUTPUT
        Serial.println(F("Helioduino::launch System launched!")); flushYield();
    #endif
}

void Helioduino::suspend()
{
    _suspend = true;
    #ifdef HELIO_USE_MULTITASKING
        if (isValidTask(_controlTaskId)) {
            taskManager.setTaskEnabled(_controlTaskId, false);
        }
        if (isValidTask(_dataTaskId)) {
            taskManager.setTaskEnabled(_dataTaskId, false);
        }
        if (isValidTask(_miscTaskId)) {
            taskManager.setTaskEnabled(_miscTaskId, false);
        }
    #endif

    #ifdef HELIO_USE_VERBOSE_OUTPUT
        Serial.println(F("Helioduino::suspend System suspended!")); flushYield();
    #endif
}

void Helioduino::update()
{
    #ifdef HELIO_USE_MULTITASKING
        taskManager.runLoop(); // tcMenu also uses this system to run its UI
    #else
        controlLoop();
        dataLoop();
        miscLoop();
    #endif

    #ifdef HELIO_USE_MQTT
        if (publisher._mqttClient) { publisher._mqttClient->loop(); }
    #endif
    #ifdef HELIO_USE_GPS // FIXME: This may get removed if it doesn't work right.
        if (_gps) { while(_gps->available()) { _gps->read(); } }
    #endif
}

bool Helioduino::registerObject(SharedPtr<HelioObject> obj)
{
    HELIO_SOFT_ASSERT(obj->getId().posIndex >= 0 && obj->getId().posIndex < HELIO_POS_MAXSIZE, SFP(HStr_Err_InvalidParameter));
    if (obj && _objects.find(obj->getKey()) == _objects.end()) {
        _objects[obj->getKey()] = obj;

        if (obj->isActuatorType() || obj->isPanelType()) {
            scheduler.setNeedsScheduling();
        }
        if (obj->isSensorType()) {
            publisher.setNeedsTabulation();
        }

        return true;
    }
    return false;
}

bool Helioduino::unregisterObject(SharedPtr<HelioObject> obj)
{
    auto iter = _objects.find(obj->getKey());
    if (iter != _objects.end()) {
        _objects.erase(iter);

        if (obj->isActuatorType() || obj->isPanelType()) {
            scheduler.setNeedsScheduling();
        }
        if (obj->isSensorType()) {
            publisher.setNeedsTabulation();
        }

        return true;
    }
    return false;
}

SharedPtr<HelioObject> Helioduino::objectById(HelioIdentity id) const
{
    if (id.posIndex == HELIO_POS_SEARCH_FROMBEG) {
        while (++id.posIndex < HELIO_POS_MAXSIZE) {
            auto iter = _objects.find(id.regenKey());
            if (iter != _objects.end()) {
                if (id.keyString == iter->second->getKeyString()) {
                    return iter->second;
                } else {
                    objectById_Col(id);
                }
            }
        }
    } else if (id.posIndex == HELIO_POS_SEARCH_FROMEND) {
        while (--id.posIndex >= 0) {
            auto iter = _objects.find(id.regenKey());
            if (iter != _objects.end()) {
                if (id.keyString == iter->second->getKeyString()) {
                    return iter->second;
                } else {
                    objectById_Col(id);
                }
            }
        }
    } else {
        auto iter = _objects.find(id.key);
        if (iter != _objects.end()) {
            if (id.keyString == iter->second->getKeyString()) {
                return iter->second;
            } else {
                objectById_Col(id);
            }
        }
    }

    return nullptr;
}

SharedPtr<HelioObject> Helioduino::objectById_Col(const HelioIdentity &id) const
{
    HELIO_SOFT_ASSERT(false, F("Hashing collision")); // exhaustive search must be performed, publishing may miss values

    for (auto iter = _objects.begin(); iter != _objects.end(); ++iter) {
        if (id.keyString == iter->second->getKeyString()) {
            return iter->second;
        }
    }

    return nullptr;
}

hposi_t Helioduino::firstPosition(HelioIdentity id, bool taken)
{
    if (id.posIndex != HELIO_POS_SEARCH_FROMEND) {
        id.posIndex = HELIO_POS_SEARCH_FROMBEG;
        while (++id.posIndex < HELIO_POS_MAXSIZE) {
            auto iter = _objects.find(id.regenKey());
            if (taken == (iter != _objects.end())) {
                return id.posIndex;
            }
        }
    } else {
        id.posIndex = HELIO_POS_SEARCH_FROMEND;
        while (--id.posIndex >= 0) {
            auto iter = _objects.find(id.regenKey());
            if (taken == (iter != _objects.end())) {
                return id.posIndex;
            }
        }
    }

    return -1;
}

bool Helioduino::tryGetPinLock(pintype_t pin, millis_t wait)
{
    millis_t start = millis();
    while (1) {
        auto iter = _pinLocks.find(pin);
        if (iter == _pinLocks.end()) {
            _pinLocks[pin] = true;
            return (_pinLocks.find(pin) != _pinLocks.end());
        }
        else if (millis() - start >= wait) { return false; }
        else { yield(); }
    }
}

void Helioduino::deselectPinMuxers()
{
    for (auto iter = Helioduino::_activeInstance->_pinMuxers.begin(); iter != Helioduino::_activeInstance->_pinMuxers.end(); ++iter) {
        iter->second->deselect();
    }
}

OneWire *Helioduino::getOneWireForPin(pintype_t pin)
{
    auto wireIter = _oneWires.find(pin);
    if (wireIter != _oneWires.end()) {
        return wireIter->second;
    } else {
        OneWire *oneWire = new OneWire(pin);
        if (oneWire) {
            _oneWires[pin] = oneWire;
            if (_oneWires.find(pin) != _oneWires.end()) { return oneWire; }
            else if (oneWire) { delete oneWire; }
        } else if (oneWire) { delete oneWire; }
    }
    return nullptr;
}

void Helioduino::dropOneWireForPin(pintype_t pin)
{
    auto wireIter = _oneWires.find(pin);
    if (wireIter != _oneWires.end()) {
        if (wireIter->second) {
            wireIter->second->depower();
            delete wireIter->second;
        }
        _oneWires.erase(wireIter);
    }
}

void Helioduino::setSystemName(String systemName)
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    if (_systemData && !systemName.equals(getSystemName())) {
        _systemData->_bumpRevIfNotAlreadyModded();
        strncpy(_systemData->systemName, systemName.c_str(), HELIO_NAME_MAXSIZE);
        if (_activeUIInstance) { _activeUIInstance->setNeedsLayout(); }
    }
}

void Helioduino::setTimeZoneOffset(int8_t timeZoneOffset)
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    if (_systemData && _systemData->timeZoneOffset != timeZoneOffset) {
        _systemData->_bumpRevIfNotAlreadyModded();
        _systemData->timeZoneOffset = timeZoneOffset;
        scheduler.setNeedsScheduling();
        if (_activeUIInstance) { _activeUIInstance->setNeedsLayout(); }
    }
}

void Helioduino::setPollingInterval(uint16_t pollingInterval)
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    if (_systemData && _systemData->pollingInterval != pollingInterval) {
        _systemData->_bumpRevIfNotAlreadyModded();
        _systemData->pollingInterval = pollingInterval;

        #ifdef HELIO_USE_MULTITASKING
            if (isValidTask(_dataTaskId)) {
                auto dataTask = taskManager.getTask(_dataTaskId);
                if (dataTask) {
                    bool enabled = dataTask->isEnabled();
                    auto next = dataTask->getNext();
                    dataTask->handleScheduling(getPollingInterval(), TIME_MILLIS, true);
                    dataTask->setNext(next);
                    dataTask->setEnabled(enabled);
                }
            }
        #endif
    }
}

void Helioduino::setAutosaveEnabled(Helio_Autosave autosaveEnabled, Helio_Autosave autosaveFallback, uint16_t autosaveInterval)
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    if (_systemData && (_systemData->autosaveEnabled != autosaveEnabled || _systemData->autosaveFallback != autosaveFallback || _systemData->autosaveInterval != autosaveInterval)) {
        _systemData->_bumpRevIfNotAlreadyModded();
        _systemData->autosaveEnabled = autosaveEnabled;
        _systemData->autosaveFallback = autosaveFallback;
        _systemData->autosaveInterval = autosaveInterval;
    }
}

void Helioduino::setRTCTime(DateTime time)
{
    auto rtc = getRTC();
    if (rtc) {
        rtc->adjust(DateTime((uint32_t)(time.unixtime() + (-getTimeZoneOffset() * SECS_PER_HOUR))));
        notifyRTCTimeUpdated();
    }
}

#ifdef HELIO_USE_WIFI

void Helioduino::setWiFiConnection(String ssid, String pass)
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    if (_systemData) {
        bool ssidChanged = ssid.equals(getWiFiSSID());
        bool passChanged = pass.equals(getWiFiPassword());

        if (ssidChanged || passChanged || (pass.length() && !_systemData->wifiPasswordSeed)) {
            _systemData->_bumpRevIfNotAlreadyModded();

            if (ssid.length()) {
                strncpy(_systemData->wifiSSID, ssid.c_str(), HELIO_NAME_MAXSIZE);
            } else {
                memset(_systemData->wifiSSID, '\0', HELIO_NAME_MAXSIZE);
            }

            if (pass.length()) {
                randomSeed(unixNow());
                _systemData->wifiPasswordSeed = random(1, RANDOM_MAX);

                randomSeed(_systemData->wifiPasswordSeed);
                for (int charIndex = 0; charIndex < HELIO_NAME_MAXSIZE; ++charIndex) {
                    _systemData->wifiPassword[charIndex] = (uint8_t)(charIndex < pass.length() ? pass[charIndex] : '\0') ^ (uint8_t)random(256);
                }
            } else {
                _systemData->wifiPasswordSeed = 0;
                memset(_systemData->wifiPassword, '\0', HELIO_NAME_MAXSIZE);
            }

            if (_netBegan && (ssidChanged || passChanged)) { WiFi.disconnect(); _netBegan = false; } // forces re-connect on next getWiFi
        }
    }
}

#endif
#ifdef HELIO_USE_ETHERNET

void Helioduino::setEthernetConnection(const uint8_t *macAddress)
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    if (_systemData) {
        bool macChanged = memcmp(macAddress, getMACAddress(), sizeof(uint8_t[6])) != 0;

        if (macChanged) {
            _systemData->_bumpRevIfNotAlreadyModded();

            memcpy(_systemData->macAddress, macAddress, sizeof(uint8_t[6]));

            if (_netBegan) { Ethernet.setMACAddress(macAddress); }
        }
    }
}

#endif
#ifdef HELIO_USE_GUI

int Helioduino::getControlInputPins() const
{
    switch (getControlInputMode()) {
        case Helio_ControlInputMode_2x2Matrix:
        case Helio_ControlInputMode_4xButton:
            return 4;
        case Helio_ControlInputMode_6xButton:
            return 6;
        case Helio_ControlInputMode_RotaryEncoder:
            return 5;
        default:
            return 0;
    }
}

pintype_t Helioduino::getControlInputPin(int ribbonPinIndex) const
{
    int ctrlInPinCount = getControlInputPins();
    HELIO_SOFT_ASSERT(ctrlInPinCount > 0, SFP(HStr_Err_UnsupportedOperation));
    HELIO_SOFT_ASSERT(ctrlInPinCount <= 0 || (ribbonPinIndex >= 0 && ribbonPinIndex < ctrlInPinCount), SFP(HStr_Err_InvalidParameter));

    return ctrlInPinCount && ribbonPinIndex >= 0 && ribbonPinIndex < ctrlInPinCount ? _ctrlInputPins[ribbonPinIndex] : -1;
}

#endif

I2C_eeprom *Helioduino::getEEPROM(bool begin)
{
    if (!_eeprom) { allocateEEPROM(); }

    if (_eeprom && begin && !_eepromBegan) {
        _eepromBegan = _eeprom->begin();

        if (!_eepromBegan) { deallocateEEPROM(); }
    }

    return (!begin || _eepromBegan) ? _eeprom : nullptr;
}

HelioRTCInterface *Helioduino::getRTC(bool begin)
{
    if (!_rtc) { allocateRTC(); }

    if (_rtc && begin && !_rtcBegan) {
        _rtcBegan = _rtc->begin(_rtcSetup.cfgAs.i2c.wire);

        if (_rtcBegan) {
            bool rtcBattFailBefore = _rtcBattFail;
            _rtcBattFail = _rtc->lostPower();
            if (_rtcBattFail && !rtcBattFailBefore) {
                logger.logWarning(SFP(HStr_Log_RTCBatteryFailure));
            }
        } else { deallocateRTC(); }
    }

    return (!begin || _rtcBegan) ? _rtc : nullptr;
}

SDClass *Helioduino::getSDCard(bool begin)
{
    if (!_sd) { allocateSD(); }

    if (_sd && begin) {
        if (!_sdBegan) {
            #if defined(ESP32)
                _sdBegan = _sd->begin(_sdSetup.cfgAs.spi.cs, _sdSetup.cfgAs.spi.spi, _sdSetup.cfgAs.spi.speed);
            #elif defined(CORE_TEENSY)
                _sdBegan = _sd->begin(_sdSetup.cfgAs.spi.cs); // card speed not possible to set on teensy
            #else
                _sdBegan = _sd->begin(_sdSetup.cfgAs.spi.speed, _sdSetup.cfgAs.spi.cs);
            #endif
        }

        if (!_sdBegan && _sdOut == 0) { deallocateSD(); }

        if (_sd && _sdBegan) { _sdOut++; }
    }

    return (!begin || _sdBegan) ? _sd : nullptr;
}

void Helioduino::endSDCard(SDClass *sd)
{
    #if defined(CORE_TEENSY)
        --_sdOut; // no delayed write on teensy's SD impl
    #else
        if (--_sdOut == 0 && _sd) {
            _sd->end();
        }
    #endif
}

#ifdef HELIO_USE_WIFI

WiFiClass *Helioduino::getWiFi(String ssid, String pass, bool begin)
{
    int status = WiFi.status();

    if (begin && (!_netBegan || status != WL_CONNECTED)) {
        if (status == WL_CONNECTED) {
            _netBegan = true;
        } else if (status == WL_NO_SHIELD) {
            _netBegan = false;
        } else { // attempt connection
            #ifdef HELIO_USE_AT_WIFI
                status = WiFi.begin(ssid.c_str(), pass.c_str());
            #else
                status = pass.length() ? WiFi.begin(const_cast<char *>(ssid.c_str()), pass.c_str())
                                       : WiFi.begin(const_cast<char *>(ssid.c_str()));
            #endif

            _netBegan = (status == WL_CONNECTED);
        }
    }

    return (!begin || _netBegan) ? &WiFi : nullptr;
}

#endif
#ifdef HELIO_USE_ETHERNET

EthernetClass *Helioduino::getEthernet(const uint8_t *macAddress, bool begin)
{
    int status = Ethernet.linkStatus();
    
    if (begin && (!_netBegan || status != LinkON)) {
        if (status == LinkON) {
            _netBegan = true;
        } else if (Ethernet.hardwareStatus() == EthernetNoHardware) {
            _netBegan = false;
        } else { // attempt connection
            status = Ethernet.begin(const_cast<uint8_t *>(getMACAddress()));

            _netBegan = (status == LinkON);
        }
    }

    return (!begin || _netBegan) ? &Ethernet : nullptr;
}

#endif
#ifdef HELIO_USE_GPS

GPSClass *Helioduino::getGPS(bool begin)
{
    if (!_gps) { allocateGPS(); }

    if (_gps && begin && !_gpsBegan) {
        switch (_gpsSetup.cfgType) {
            case DeviceSetup::UARTSetup:
                _gpsBegan = _gps->begin(_gpsSetup.cfgAs.uart.baud);
            case DeviceSetup::I2CSetup:
                _gpsBegan = _gps->begin(_gpsSetup.cfgAs.i2c.speed);
            case DeviceSetup::SPISetup:
                _gpsBegan = _gps->begin(_gpsSetup.cfgAs.spi.speed);
            default: break;
        }
        if (!_gpsBegan) { deallocateGPS(); }
    }

    return (!begin || _gpsBegan) ? _gps : nullptr;
}

#endif

Helio_SystemMode Helioduino::getSystemMode() const
{
    return _systemData ? _systemData->systemMode : Helio_SystemMode_Undefined;
}

Helio_MeasurementMode Helioduino::getMeasurementMode() const
{
    return _systemData ? _systemData->measureMode : Helio_MeasurementMode_Undefined;
}

Helio_DisplayOutputMode Helioduino::getDisplayOutputMode() const
{
    return _systemData ? _systemData->dispOutMode : Helio_DisplayOutputMode_Undefined;
}

Helio_ControlInputMode Helioduino::getControlInputMode() const
{
    return _systemData ? _systemData->ctrlInMode : Helio_ControlInputMode_Undefined;
}

String Helioduino::getSystemName() const
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    return _systemData ? String(_systemData->systemName) : String();
}

int8_t Helioduino::getTimeZoneOffset() const
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    return _systemData ? _systemData->timeZoneOffset : 0;
}

uint16_t Helioduino::getPollingInterval() const
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    return _systemData ? _systemData->pollingInterval : 0;
}

bool Helioduino::isPollingFrameOld(hframe_t frame, hframe_t allowance) const
{
    return _pollingFrame - frame > allowance;
}

bool Helioduino::isAutosaveEnabled() const
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    return _systemData ? _systemData->autosaveEnabled != Helio_Autosave_Disabled : false;
}

bool Helioduino::isAutosaveFallbackEnabled() const
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    return _systemData ? _systemData->autosaveFallback != Helio_Autosave_Disabled : false;
}

#ifdef HELIO_USE_WIFI

String Helioduino::getWiFiSSID() const
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    return _systemData ? String(_systemData->wifiSSID) : String();
}

String Helioduino::getWiFiPassword() const
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    if (_systemData) {
        char wifiPassword[HELIO_NAME_MAXSIZE] = {0};

        if (_systemData->wifiPasswordSeed) {
            randomSeed(_systemData->wifiPasswordSeed);
            for (int charIndex = 0; charIndex < HELIO_NAME_MAXSIZE; ++charIndex) {
                wifiPassword[charIndex] = (char)(_systemData->wifiPassword[charIndex] ^ (uint8_t)random(256));
            }
        } else {
            strncpy(wifiPassword, (const char *)(_systemData->wifiPassword), HELIO_NAME_MAXSIZE);
        }

        return String(wifiPassword);
    }
    return String();
}

#endif
#ifdef HELIO_USE_ETHERNET

const uint8_t *Helioduino::getMACAddress() const
{
    HELIO_SOFT_ASSERT(_systemData, SFP(HStr_Err_NotYetInitialized));
    return _systemData ? &_systemData->macAddress[0] : nullptr;
}

#endif

void Helioduino::notifyRTCTimeUpdated()
{
    _rtcBattFail = false;
    _lastAutosave = 0;
    logger.updateInitTracking();
    scheduler.broadcastDayChange();
}

void Helioduino::notifyDayChanged()
{
    for (auto iter = _objects.begin(); iter != _objects.end(); ++iter) {
        if (iter->second->isPanelType()) {
            auto panel = static_pointer_cast<HelioPanel>(iter->second);

            // todo
            // if (panel && panel->isTrackClass()) {
            //     auto panel = static_pointer_cast<HelioPanel>(iter->second);
            //     if (panel) {panel->notifyDayChanged(); }
            // }
        }
    }
}

void Helioduino::checkFreeMemory()
{
    auto memLeft = freeMemory();
    if (memLeft != -1 && memLeft < HELIO_SYS_FREERAM_LOWBYTES) {
        broadcastLowMemory();
    }
}

void Helioduino::broadcastLowMemory()
{
    for (auto iter = _objects.begin(); iter != _objects.end(); ++iter) {
        iter->second->handleLowMemory();
    }
}

static uint64_t getSDCardFreeSpace()
{
    uint64_t retVal = HELIO_SYS_FREESPACE_LOWSPACE;
    #if defined(CORE_TEENSY)
        auto sd = getHelioInstance()->getSDCard();
        if (sd) {
            retVal = sd->totalSize() - sd->usedSize();
            getHelioInstance()->endSDCard(sd);
        }
    #endif
    return retVal;
}

void Helioduino::checkFreeSpace()
{
    if ((logger.isLoggingEnabled() || publisher.isPublishingEnabled()) &&
        (!_lastSpaceCheck || unixNow() >= _lastSpaceCheck + (HELIO_SYS_FREESPACE_INTERVAL * SECS_PER_MIN))) {
        if (logger.isLoggingToSDCard() || publisher.isPublishingToSDCard()) {
            uint32_t freeKB = getSDCardFreeSpace() >> 10;
            while (freeKB < HELIO_SYS_FREESPACE_LOWSPACE) {
                logger.cleanupOldestLogs(true);
                publisher.cleanupOldestData(true);
                freeKB = getSDCardFreeSpace();
            }
        }
        // TODO: URL free space
        _lastSpaceCheck = unixNow();
    }
}

void Helioduino::checkAutosave()
{
    if (isAutosaveEnabled() && unixNow() >= _lastAutosave + (_systemData->autosaveInterval * SECS_PER_MIN)) {
        for (int index = 0; index < 2; ++index) {
            switch (index == 0 ? _systemData->autosaveEnabled : _systemData->autosaveFallback) {
                case Helio_Autosave_EnabledToSDCardJson:
                    saveToSDCard(JSON);
                    break;
                case Helio_Autosave_EnabledToSDCardRaw:
                    saveToSDCard(RAW);
                    break;
                case Helio_Autosave_EnabledToEEPROMJson:
                    saveToEEPROM(JSON);
                    break;
                case Helio_Autosave_EnabledToEEPROMRaw:
                    saveToEEPROM(RAW);
                    break;
                case Helio_Autosave_EnabledToWiFiStorageJson:
                    #ifdef HELIO_USE_WIFI_STORAGE
                        saveToWiFiStorage(JSON);
                    #endif
                    break;
                case Helio_Autosave_EnabledToWiFiStorageRaw:
                    #ifdef HELIO_USE_WIFI_STORAGE
                        saveToWiFiStorage(RAW);
                    #endif
                case Helio_Autosave_Disabled:
                    break;
            }
        }
        _lastAutosave = unixNow();
    }
}

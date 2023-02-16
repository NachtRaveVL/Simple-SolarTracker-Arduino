/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Strings
*/

#include "Helioduino.h"

#ifndef HELIO_DISABLE_BUILTIN_DATA
String stringFromPGMAddr(const char *flashStr);
const char *pgmAddrForStr(Helio_String strNum);
#endif

static uint16_t _strDataAddress((uint16_t)-1);
void beginStringsFromEEPROM(uint16_t dataAddress)
{
    _strDataAddress = dataAddress;
}

static String _strDataFilePrefix;
void beginStringsFromSDCard(String dataFilePrefix)
{
    _strDataFilePrefix = dataFilePrefix;
}

inline String getStringsFilename()
{
    String filename; filename.reserve(_strDataFilePrefix.length() + 12);
    filename.concat(_strDataFilePrefix);
    filename.concat('s'); // Cannot use SFP here so have to do it the long way
    filename.concat('t');
    filename.concat('r');
    filename.concat('i');
    filename.concat('n');
    filename.concat('g');
    filename.concat('s');
    filename.concat('.');
    filename.concat('d');
    filename.concat('a');
    filename.concat('t');
    return filename;
}

String stringFromPGM(Helio_String strNum)
{    
    static Helio_String _lookupStrNum = HStr_Count; // Simple LRU cache reduces a lot of lookup access
    static String _lookupCachedRes;
    if (strNum == _lookupStrNum && _lookupCachedRes.length()) { return _lookupCachedRes; }
    else { _lookupStrNum = strNum; } // _lookupCachedRes set below

    if (_strDataAddress != (uint16_t)-1) {
        auto eeprom = getController()->getEEPROM();

        if (eeprom) {
            uint16_t lookupOffset = 0;
            eeprom->readBlock(_strDataAddress + (sizeof(uint16_t) * ((int)strNum + 1)), // +1 for initial total size word
                              (uint8_t *)&lookupOffset, sizeof(lookupOffset));

            {   String retVal;
                char buffer[HELIO_STRING_BUFFER_SIZE] = {0};
                uint16_t bytesRead = eeprom->readBlock(lookupOffset, (uint8_t *)&buffer[0], HELIO_STRING_BUFFER_SIZE);
                retVal.concat(charsToString(buffer, bytesRead));

                while (strnlen(buffer, HELIO_STRING_BUFFER_SIZE) == HELIO_STRING_BUFFER_SIZE) {
                    lookupOffset += HELIO_STRING_BUFFER_SIZE;
                    bytesRead = eeprom->readBlock(lookupOffset, (uint8_t *)&buffer[0], HELIO_STRING_BUFFER_SIZE);
                    if (bytesRead) { retVal.concat(charsToString(buffer, bytesRead)); }
                }

                if (retVal.length()) {
                    return (_lookupCachedRes = retVal);
                }
            }
        }
    }

    #ifndef HELIO_DISABLE_BUILTIN_DATA
        return (_lookupCachedRes = stringFromPGMAddr(pgmAddrForStr(strNum)));
    #endif

    if (_strDataFilePrefix.length()) {
        #if HELIO_SYS_LEAVE_FILES_OPEN
            static
        #endif
        auto sd = getController()->getSDCard();

        if (sd) {
            String retVal;
            #if HELIO_SYS_LEAVE_FILES_OPEN
                static
            #endif
            auto file = sd->open(getStringsFilename().c_str(), FILE_READ);

            if (file) {
                uint16_t lookupOffset = 0;
                file.seek(sizeof(uint16_t) * (int)strNum);
                #if defined(ARDUINO_ARCH_RP2040) || defined(ESP_PLATFORM)
                    file.readBytes((char *)&lookupOffset, sizeof(lookupOffset));
                #else
                    file.readBytes((uint8_t *)&lookupOffset, sizeof(lookupOffset));
                #endif

                {   char buffer[HELIO_STRING_BUFFER_SIZE];
                    file.seek(lookupOffset);
                    auto bytesRead = file.readBytesUntil('\0', buffer, HELIO_STRING_BUFFER_SIZE);
                    retVal.concat(charsToString(buffer, bytesRead));

                    while (strnlen(buffer, HELIO_STRING_BUFFER_SIZE) == HELIO_STRING_BUFFER_SIZE) {
                        bytesRead = file.readBytesUntil('\0', buffer, HELIO_STRING_BUFFER_SIZE);
                        if (bytesRead) { retVal.concat(charsToString(buffer, bytesRead)); }
                    }
                }

                #if !HELIO_SYS_LEAVE_FILES_OPEN
                    file.close();
                #endif
            }

            #if !HELIO_SYS_LEAVE_FILES_OPEN
                getController()->endSDCard(sd);
            #endif
            if (retVal.length()) {
                return (_lookupCachedRes = retVal);
            }
        }
    }

    return (_lookupCachedRes = String());
}

String stringFromPGMAddr(const char *flashStr) {
    String retVal; retVal.reserve(strlen_P(flashStr) + 1);
    char buffer[HELIO_STRING_BUFFER_SIZE] = {0};
    strncpy_P(buffer, flashStr, HELIO_STRING_BUFFER_SIZE);
    retVal.concat(charsToString(buffer, HELIO_STRING_BUFFER_SIZE));

    while (strnlen(buffer, HELIO_STRING_BUFFER_SIZE) == HELIO_STRING_BUFFER_SIZE) {
        flashStr += HELIO_STRING_BUFFER_SIZE;
        strncpy_P(buffer, flashStr, HELIO_STRING_BUFFER_SIZE);
        if (buffer[0]) { retVal.concat(charsToString(buffer, HELIO_STRING_BUFFER_SIZE)); }
    }

    return retVal;
}

#ifndef HELIO_DISABLE_BUILTIN_DATA

const char *pgmAddrForStr(Helio_String strNum)
{
    switch(strNum) {
        case HStr_ColonSpace: {
            static const char flashStr_ColonSpace[] PROGMEM = {": "};
            return flashStr_ColonSpace;
        } break;
        case HStr_DoubleSpace: {
            static const char flashStr_DoubleSpace[] PROGMEM = {"  "};
            return flashStr_DoubleSpace;
        } break;
        case HStr_Count: {
            static const char flashStr_Count[] PROGMEM = {"Count"};
            return flashStr_Count;
        } break;
        case HStr_csv: {
            static const char flashStr_csv[] PROGMEM = {"csv"};
            return flashStr_csv;
        } break;
        case HStr_dat: {
            static const char flashStr_dat[] PROGMEM = {"dat"};
            return flashStr_dat;
        } break;
        case HStr_Disabled: {
            static const char flashStr_Disabled[] PROGMEM = {"Disabled"};
            return flashStr_Disabled;
        } break;
        case HStr_raw: {
            static const char flashStr_raw[] PROGMEM = {"raw"};
            return flashStr_raw;
        } break;
        case HStr_txt: {
            static const char flashStr_txt[] PROGMEM = {"txt"};
            return flashStr_txt;
        } break;
        case HStr_Undefined: {
            static const char flashStr_Undefined[] PROGMEM = {"Undefined"};
            return flashStr_Undefined;
        } break;
        case HStr_null: {
            static const char flashStr_null[] PROGMEM = {"null"};
            return flashStr_null;
        } break;

        case HStr_Default_SystemName: {
            static const char flashStr_Default_SystemName[] PROGMEM = {"Helioduino"};
            return flashStr_Default_SystemName;
        } break;
        case HStr_Default_ConfigFilename: {
            static const char flashStr_Default_ConfigFilename[] PROGMEM = {"Helioduino.cfg"};
            return flashStr_Default_ConfigFilename;
        } break;

        case HStr_Err_AllocationFailure: {
            static const char flashStr_Err_AllocationFailure[] PROGMEM = {"Allocation failure"};
            return flashStr_Err_AllocationFailure;
        } break;
        case HStr_Err_AlreadyInitialized: {
            static const char flashStr_Err_AlreadyInitialized[] PROGMEM = {"Already initialized"};
            return flashStr_Err_AlreadyInitialized;
        } break;
        case HStr_Err_AssertionFailure: {
            static const char flashStr_Err_AssertionFailure[] PROGMEM = {"Assertion failure"};
            return flashStr_Err_AssertionFailure;
        } break;
        case HStr_Err_ExportFailure: {
            static const char flashStr_Err_ExportFailure[] PROGMEM = {"Export failure"};
            return flashStr_Err_ExportFailure;
        } break;
        case HStr_Err_ImportFailure: {
            static const char flashStr_Err_ImportFailure[] PROGMEM = {"Import failure"};
            return flashStr_Err_ImportFailure;
        } break;
        case HStr_Err_InitializationFailure: {
            static const char flashStr_Err_InitializationFailure[] PROGMEM = {"Initialization failure"};
            return flashStr_Err_InitializationFailure;
        } break;
        case HStr_Err_InvalidParameter: {
            static const char flashStr_Err_InvalidParameter[] PROGMEM = {"Invalid parameter"};
            return flashStr_Err_InvalidParameter;
        } break;
        case HStr_Err_InvalidPinOrType: {
            static const char flashStr_Err_InvalidPinOrType[] PROGMEM = {"Invalid pin or type"};
            return flashStr_Err_InvalidPinOrType;
        } break;
        case HStr_Err_MeasurementFailure: {
            static const char flashStr_Err_MeasurementFailure[] PROGMEM = {"Measurement failure"};
            return flashStr_Err_MeasurementFailure;
        } break;
        case HStr_Err_MissingLinkage: {
            static const char flashStr_Err_MissingLinkage[] PROGMEM = {"Missing or no linkage"};
            return flashStr_Err_MissingLinkage;
        } break;
        case HStr_Err_NoPositionsAvailable: {
            static const char flashStr_Err_NoPositionsAvailable[] PROGMEM = {"No positions available"};
            return flashStr_Err_NoPositionsAvailable;
        } break;
        case HStr_Err_NotYetInitialized: {
            static const char flashStr_Err_NotYetInitialized[] PROGMEM = {"Not yet initialized"};
            return flashStr_Err_NotYetInitialized;
        } break;
        case HStr_Err_OperationFailure: {
            static const char flashStr_Err_OperationFailure[] PROGMEM = {"Operation failure"};
            return flashStr_Err_OperationFailure;
        } break;
        case HStr_Err_UnsupportedOperation: {
            static const char flashStr_Err_UnsupportedOperation[] PROGMEM = {"Unsupported operation"};
            return flashStr_Err_UnsupportedOperation;
        } break;

        case HStr_Log_AirReport: {
            static const char flashStr_Log_AirReport[] PROGMEM = {" air report:"};
            return flashStr_Log_AirReport;
        } break;
        case HStr_Log_CalculatedTravel: {
            static const char flashStr_Log_CalculatedTravel[] PROGMEM = {" travel request:"};
            return flashStr_Log_CalculatedTravel;
        } break;
        case HStr_Log_HasBegan: {
            static const char flashStr_Log_HasBegan[] PROGMEM = {" has began"};
            return flashStr_Log_HasBegan;
        } break;
        case HStr_Log_HasDisabled: {
            static const char flashStr_Log_HasDisabled[] PROGMEM = {" has disabled"};
            return flashStr_Log_HasDisabled;
        } break;
        case HStr_Log_HasEnabled: {
            static const char flashStr_Log_HasEnabled[] PROGMEM = {" has enabled"};
            return flashStr_Log_HasEnabled;
        } break;
        case HStr_Log_HasEnded: {
            static const char flashStr_Log_HasEnded[] PROGMEM = {" has ended"};
            return flashStr_Log_HasEnded;
        } break;
        case HStr_Log_LightingSequence: {
            static const char flashStr_Log_LightingSequence[] PROGMEM = {" lighting sequence"};
            return flashStr_Log_LightingSequence;
        } break;
        case HStr_Log_MeasuredTravel: {
            static const char flashStr_Log_LightingSequence[] PROGMEM = {" travel result:"};
            return flashStr_Log_LightingSequence;
        } break;
        case HStr_Log_RTCBatteryFailure: {
            static const char flashStr_Log_RTCBatteryFailure[] PROGMEM = {"RTC battery failure, time needs reset."};
            return flashStr_Log_RTCBatteryFailure;
        } break;
        case HStr_Log_SystemDataSaved: {
            static const char flashStr_Log_SystemDataSaved[] PROGMEM = {"System data saved"};
            return flashStr_Log_SystemDataSaved;
        } break;
        case HStr_Log_SystemUptime: {
            static const char flashStr_Log_SystemUptime[] PROGMEM = {"System uptime: "};
            return flashStr_Log_SystemUptime;
        } break;

        case HStr_Log_Prefix_Info: {
            static const char flashStr_Prefix_Info[] PROGMEM = {"[INFO] "};
            return flashStr_Prefix_Info;
        } break;
        case HStr_Log_Prefix_Warning: {
            static const char flashStr_Log_Prefix_Warning[] PROGMEM = {"[WARN] "};
            return flashStr_Log_Prefix_Warning;
        } break;
        case HStr_Log_Prefix_Error: {
            static const char flashStr_Log_Prefix_Error[] PROGMEM = {"[FAIL] "};
            return flashStr_Log_Prefix_Error;
        } break;

        case HStr_Log_Field_Light_Duration: {
            static const char flashStr_Log_Field_Light_Duration[] PROGMEM = {"  Daylight hours: "};
            return flashStr_Log_Field_Light_Duration;
        } break;
        case HStr_Log_Field_Panel: {
            static const char flashStr_Log_Field_Panel[] PROGMEM = {"  Solar panel: "};
            return flashStr_Log_Field_Panel;
        } break;
        case HStr_Log_Field_Sprayer_Duration: {
            static const char flashStr_Log_Field_Sprayer_Duration[] PROGMEM = {"  Sprayer run time: "};
            return flashStr_Log_Field_Sprayer_Duration;
        } break;
        case HStr_Log_Field_Temp_Measured: {
            static const char flashStr_Log_Field_Temp_Measured[] PROGMEM = {"  Temp sensor: "};
            return flashStr_Log_Field_Temp_Measured;
        } break;
        case HStr_Log_Field_Temp_Setpoint: {
            static const char flashStr_Log_Field_Temp_Setpoint[] PROGMEM = {"  Temp setpoint: "};
            return flashStr_Log_Field_Temp_Setpoint;
        } break;
        case HStr_Log_Field_Time_Calculated: {
            static const char flashStr_Log_Field_Time_Calculated[] PROGMEM = {"  Motor run time: "};
            return flashStr_Log_Field_Time_Calculated;
        } break;
        case HStr_Log_Field_Time_Finish: {
            static const char flashStr_Log_Field_Time_Finish[] PROGMEM = {"  Finish time: "};
            return flashStr_Log_Field_Time_Finish;
        } break;
        case HStr_Log_Field_Time_Measured: {
            static const char flashStr_Log_Field_Time_Measured[] PROGMEM = {"  Elapsed time: "};
            return flashStr_Log_Field_Time_Measured;
        } break;
        case HStr_Log_Field_Time_Start: {
            static const char flashStr_Log_Field_Time_Start[] PROGMEM = {"  Start time: "};
            return flashStr_Log_Field_Time_Start;
        } break;
        case HStr_Log_Field_Travel_Calculated: {
            static const char flashStr_Log_Field_Travel_Calculated[] PROGMEM = {"  Est. travel dis.: "};
            return flashStr_Log_Field_Travel_Calculated;
        } break;
        case HStr_Log_Field_Travel_Measured: {
            static const char flashStr_Log_Field_Travel_Measured[] PROGMEM = {"  Act. travel dis.: "};
            return flashStr_Log_Field_Travel_Measured;
        } break;

        case HStr_Key_ActiveLow: {
            static const char flashStr_Key_ActiveLow[] PROGMEM = {"activeLow"};
            return flashStr_Key_ActiveLow;
        } break;
        case HStr_Key_AirReportInterval: {
            static const char flashStr_Key_AirReportInterval[] PROGMEM = {"airReportInterval"};
            return flashStr_Key_AirReportInterval;
        } break;
        case HStr_Key_Altitude: {
            static const char flashStr_Key_Altitude[] PROGMEM = {"altitude"};
            return flashStr_Key_Altitude;
        } break;
        case HStr_Key_AutosaveEnabled: {
            static const char flashStr_Key_AutosaveEnabled[] PROGMEM = {"autosaveEnabled"};
            return flashStr_Key_AutosaveEnabled;
        } break;
        case HStr_Key_AutosaveFallback: {
            static const char flashStr_Key_AutosaveFallback[] PROGMEM = {"autosaveFallback"};
            return flashStr_Key_AutosaveFallback;
        } break;
        case HStr_Key_AutosaveInterval: {
            static const char flashStr_Key_AutosaveInterval[] PROGMEM = {"autosaveInterval"};
            return flashStr_Key_AutosaveInterval;
        } break;
        case HStr_Key_BitRes: {
            static const char flashStr_Key_BitRes[] PROGMEM = {"bitRes"};
            return flashStr_Key_BitRes;
        } break;
        case HStr_Key_CalibrationUnits: {
            static const char flashStr_Key_CalibrationUnits[] PROGMEM = {"calibrationUnits"};
            return flashStr_Key_CalibrationUnits;
        } break;
        case HStr_Key_Channel: {
            static const char flashStr_Key_Channel[] PROGMEM = {"channel"};
            return flashStr_Key_Channel;
        } break;
        case HStr_Key_ChannelPins: {
            static const char flashStr_Key_ChannelPins[] PROGMEM = {"channelPins"};
            return flashStr_Key_ChannelPins;
        } break;
        case HStr_Key_ChipEnablePin: {
            static const char flashStr_Key_ChipEnablePin[] PROGMEM = {"chipEnablePin"};
            return flashStr_Key_ChipEnablePin;
        } break;
        case HStr_Key_ComputeHeatIndex: {
            static const char flashStr_Key_ComputeHeatIndex[] PROGMEM = {"computeHeatIndex"};
            return flashStr_Key_ComputeHeatIndex;
        } break;
        case HStr_Key_ContinuousPowerUsage: {
            static const char flashStr_Key_ContinuousPowerUsage[] PROGMEM = {"contPowerUsage"};
            return flashStr_Key_ContinuousPowerUsage;
        } break;
        case HStr_Key_ContinuousSpeed: {
            static const char flashStr_Key_ContinuousSpeed[] PROGMEM = {"contSpeed"};
            return flashStr_Key_ContinuousSpeed;
        } break;
        case HStr_Key_CtrlInMode: {
            static const char flashStr_Key_CtrlInMode[] PROGMEM = {"ctrlInMode"};
            return flashStr_Key_CtrlInMode;
        } break;
        case HStr_Key_DailyLightHours: {
            static const char flashStr_Key_DailyLightHours[] PROGMEM = {"dailyLightHours"};
            return flashStr_Key_DailyLightHours;
        } break;
        case HStr_Key_DataFilePrefix: {
            static const char flashStr_Key_DataFilePrefix[] PROGMEM = {"dataFilePrefix"};
            return flashStr_Key_DataFilePrefix;
        } break;
        case HStr_Key_DetriggerTol: {
            static const char flashStr_Key_DetriggerTol[] PROGMEM = {"detriggerTol"};
            return flashStr_Key_DetriggerTol;
        } break;
        case HStr_Key_DHTType: {
            static const char flashStr_Key_DHTType[] PROGMEM = {"dhtType"};
            return flashStr_Key_DHTType;
        } break;
        case HStr_Key_DispOutMode: {
            static const char flashStr_Key_DispOutMode[] PROGMEM = {"dispOutMode"};
            return flashStr_Key_DispOutMode;
        } break;
        case HStr_Key_DistUnits: {
            static const char flashStr_Key_DistUnits[] PROGMEM = {"distanceUnits"};
            return flashStr_Key_DistUnits;
        } break;
        case HStr_Key_EnableMode: {
            static const char flashStr_Key_EnableMode[] PROGMEM = {"enableMode"};
            return flashStr_Key_EnableMode;
        } break;
        case HStr_Key_Flags: {
            static const char flashStr_Key_Flags[] PROGMEM = {"flags"};
            return flashStr_Key_Flags;
        } break;
        case HStr_Key_Id: {
            static const char flashStr_Key_Id[] PROGMEM = {"id"};
            return flashStr_Key_Id;
        } break;
        case HStr_Key_InputInversion: {
            static const char flashStr_Key_InputInversion[] PROGMEM = {"inputInversion"};
            return flashStr_Key_InputInversion;
        } break;
        case HStr_Key_InputPin: {
            static const char flashStr_Key_InputPin[] PROGMEM = {"inputPin"};
            return flashStr_Key_InputPin;
        } break;
        case HStr_Key_Latitude: {
            static const char flashStr_Key_Latitude[] PROGMEM = {"latitude"};
            return flashStr_Key_Latitude;
        } break;
        case HStr_Key_LastChangeDate: {
            static const char flashStr_Key_LastChangeDate[] PROGMEM = {"lastChangeDate"};
            return flashStr_Key_LastChangeDate;
        } break;
        case HStr_Key_LimitTrigger: {
            static const char flashStr_Key_LimitTrigger[] PROGMEM = {"limitTrigger"};
            return flashStr_Key_LimitTrigger;
        } break;
        case HStr_Key_LogFilePrefix: {
            static const char flashStr_Key_LogFilePrefix[] PROGMEM = {"logFilePrefix"};
            return flashStr_Key_LogFilePrefix;
        } break;
        case HStr_Key_LogLevel: {
            static const char flashStr_Key_LogLevel[] PROGMEM = {"logLevel"};
            return flashStr_Key_LogLevel;
        } break;
        case HStr_Key_LogToSDCard: {
            static const char flashStr_Key_LogToSDCard[] PROGMEM = {"logToSDCard"};
            return flashStr_Key_LogToSDCard;
        } break;
        case HStr_Key_LogToWiFiStorage: {
            static const char flashStr_Key_LogToWiFiStorage[] PROGMEM = {"logToWiFiStorage"};
            return flashStr_Key_LogToWiFiStorage;
        } break;
        case HStr_Key_Logger: {
            static const char flashStr_Key_Logger[] PROGMEM = {"logger"};
            return flashStr_Key_Logger;
        } break;
        case HStr_Key_Longitude: {
            static const char flashStr_Key_Longitude[] PROGMEM = {"longitude"};
            return flashStr_Key_Longitude;
        } break;
        case HStr_Key_MACAddress: {
            static const char flashStr_Key_MACAddress[] PROGMEM = {"macAddress"};
            return flashStr_Key_MACAddress;
        } break;
        case HStr_Key_MaxActiveAtOnce: {
            static const char flashStr_Key_MaxActiveAtOnce[] PROGMEM = {"maxActiveAtOnce"};
            return flashStr_Key_MaxActiveAtOnce;
        } break;
        case HStr_Key_MaxPower: {
            static const char flashStr_Key_MaxPower[] PROGMEM = {"maxPower"};
            return flashStr_Key_MaxPower;
        } break;
        case HStr_Key_MeasureMode: {
            static const char flashStr_Key_MeasureMode[] PROGMEM = {"measureMode"};
            return flashStr_Key_MeasureMode;
        } break;
        case HStr_Key_MeasureRow: {
            static const char flashStr_Key_MeasureRow[] PROGMEM = {"measureRow"};
            return flashStr_Key_MeasureRow;
        } break;
        case HStr_Key_MeasureUnits: {
            static const char flashStr_Key_MeasureUnits[] PROGMEM = {"measureUnits"};
            return flashStr_Key_MeasureUnits;
        } break;
        case HStr_Key_Mode: {
            static const char flashStr_Key_Mode[] PROGMEM = {"mode"};
            return flashStr_Key_Mode;
        } break;
        case HStr_Key_Multiplier: {
            static const char flashStr_Key_Multiplier[] PROGMEM = {"multiplier"};
            return flashStr_Key_Multiplier;
        } break;
        case HStr_Key_Offset: {
            static const char flashStr_Key_Offset[] PROGMEM = {"offset"};
            return flashStr_Key_Offset;
        } break;
        case HStr_Key_OutputPin: {
            static const char flashStr_Key_OutputPin[] PROGMEM = {"outputPin"};
            return flashStr_Key_OutputPin;
        } break;
        case HStr_Key_OutputPin2: {
            static const char flashStr_Key_OutputPin2[] PROGMEM = {"outputPin2"};
            return flashStr_Key_OutputPin2;
        } break;
        case HStr_Key_PanelName: {
            static const char flashStr_Key_PanelName[] PROGMEM = {"panelName"};
            return flashStr_Key_PanelName;
        } break;
        case HStr_Key_Pin: {
            static const char flashStr_Key_Pin[] PROGMEM = {"pin"};
            return flashStr_Key_Pin;
        } break;
        case HStr_Key_PollingInterval: {
            static const char flashStr_Key_PollingInterval[] PROGMEM = {"pollingInterval"};
            return flashStr_Key_PollingInterval;
        } break;
        case HStr_Key_PositionSensor: {
            static const char flashStr_Key_PositionSensor[] PROGMEM = {"positionSensor"};
            return flashStr_Key_PositionSensor;
        } break;
        case HStr_Key_PowerSensor: {
            static const char flashStr_Key_PowerSensor[] PROGMEM = {"powerSensor"};
            return flashStr_Key_PowerSensor;
        } break;
        case HStr_Key_PowerUnits: {
            static const char flashStr_Key_PowerUnits[] PROGMEM = {"powerUnits"};
            return flashStr_Key_PowerUnits;
        } break;
        case HStr_Key_PublishToSDCard: {
            static const char flashStr_Key_PublishToSDCard[] PROGMEM = {"pubToSDCard"};
            return flashStr_Key_PublishToSDCard;
        } break;
        case HStr_Key_PublishToWiFiStorage: {
            static const char flashStr_Key_PublishToWiFiStorage[] PROGMEM = {"pubToWiFiStorage"};
            return flashStr_Key_PublishToWiFiStorage;
        } break;
        case HStr_Key_Publisher: {
            static const char flashStr_Key_Publisher[] PROGMEM = {"publisher"};
            return flashStr_Key_Publisher;
        } break;
        case HStr_Key_PWMChannel: {
            static const char flashStr_Key_PWMChannel[] PROGMEM = {"pwmChannel"};
            return flashStr_Key_PWMChannel;
        } break;
        case HStr_Key_PWMFrequency: {
            static const char flashStr_Key_PWMFrequency[] PROGMEM = {"pwmFrequency"};
            return flashStr_Key_PWMFrequency;
        } break;
        case HStr_Key_RailName: {
            static const char flashStr_Key_RailName[] PROGMEM = {"railName"};
            return flashStr_Key_RailName;
        } break;
        case HStr_Key_Revision: {
            static const char flashStr_Key_Revision[] PROGMEM = {"revision"};
            return flashStr_Key_Revision;
        } break;
        case HStr_Key_Scheduler: {
            static const char flashStr_Key_Scheduler[] PROGMEM = {"scheduler"};
            return flashStr_Key_Scheduler;
        } break;
        case HStr_Key_SensorName: {
            static const char flashStr_Key_SensorName[] PROGMEM = {"sensorName"};
            return flashStr_Key_SensorName;
        } break;
        case HStr_Key_SignalPin: {
            static const char flashStr_Key_SignalPin[] PROGMEM = {"signalPin"};
            return flashStr_Key_SignalPin;
        } break;
        case HStr_Key_SpeedSensor: {
            static const char flashStr_Key_SpeedSensor[] PROGMEM = {"speedSensor"};
            return flashStr_Key_SpeedSensor;
        } break;
        case HStr_Key_Spraying: {
            static const char flashStr_Key_Spraying[] PROGMEM = {"spraying"};
            return flashStr_Key_Spraying;
        } break;
        case HStr_Key_State: {
            static const char flashStr_Key_State[] PROGMEM = {"state"};
            return flashStr_Key_State;
        } break;
        case HStr_Key_SystemMode: {
            static const char flashStr_Key_SystemMode[] PROGMEM = {"systemMode"};
            return flashStr_Key_SystemMode;
        } break;
        case HStr_Key_SystemName: {
            static const char flashStr_Key_SystemName[] PROGMEM = {"systemName"};
            return flashStr_Key_SystemName;
        } break;
        case HStr_Key_TemperatureUnits: {
            static const char flashStr_Key_TemperatureUnits[] PROGMEM = {"temperatureUnits"};
            return flashStr_Key_TemperatureUnits;
        } break;
        case HStr_Key_TemperatureSensor: {
            static const char flashStr_Key_TemperatureSensor[] PROGMEM = {"tempSensor"};
            return flashStr_Key_TemperatureSensor;
        } break;
        case HStr_Key_TimeZoneOffset: {
            static const char flashStr_Key_TimeZoneOffset[] PROGMEM = {"timeZoneOffset"};
            return flashStr_Key_TimeZoneOffset;
        } break;
        case HStr_Key_Timestamp: {
            static const char flashStr_Key_Timestamp[] PROGMEM = {"timestamp"};
            return flashStr_Key_Timestamp;
        } break;
        case HStr_Key_Tolerance: {
            static const char flashStr_Key_Tolerance[] PROGMEM = {"tolerance"};
            return flashStr_Key_Tolerance;
        } break;
        case HStr_Key_ToleranceHigh: {
            static const char flashStr_Key_ToleranceHigh[] PROGMEM = {"toleranceHigh"};
            return flashStr_Key_ToleranceHigh;
        } break;
        case HStr_Key_ToleranceLow: {
            static const char flashStr_Key_ToleranceLow[] PROGMEM = {"toleranceLow"};
            return flashStr_Key_ToleranceLow;
        } break;
        case HStr_Key_TriggerBelow: {
            static const char flashStr_Key_TriggerBelow[] PROGMEM = {"triggerBelow"};
            return flashStr_Key_TriggerBelow;
        } break;
        case HStr_Key_TriggerOutside: {
            static const char flashStr_Key_TriggerOutside[] PROGMEM = {"triggerOutside"};
            return flashStr_Key_TriggerOutside;
        } break;
        case HStr_Key_Type: {
            static const char flashStr_Key_Type[] PROGMEM = {"type"};
            return flashStr_Key_Type;
        } break;
        case HStr_Key_Units: {
            static const char flashStr_Key_Units[] PROGMEM = {"units"};
            return flashStr_Key_Units;
        } break;
        case HStr_Key_UsingISR: {
            static const char flashStr_Key_UsingISR[] PROGMEM = {"usingISR"};
            return flashStr_Key_UsingISR;
        } break;
        case HStr_Key_Value: {
            static const char flashStr_Key_Value[] PROGMEM = {"value"};
            return flashStr_Key_Value;
        } break;
        case HStr_Key_Version: {
            static const char flashStr_Key_Version[] PROGMEM = {"version"};
            return flashStr_Key_Version;
        } break;
        case HStr_Key_WiFiPassword: {
            static const char flashStr_Key_WiFiPassword[] PROGMEM = {"wifiPassword"};
            return flashStr_Key_WiFiPassword;
        } break;
        case HStr_Key_WiFiPasswordSeed: {
            static const char flashStr_Key_WiFiPasswordSeed[] PROGMEM = {"wifiPwSeed"};
            return flashStr_Key_WiFiPasswordSeed;
        } break;
        case HStr_Key_WiFiSSID: {
            static const char flashStr_Key_WiFiSSID[] PROGMEM = {"wifiSSID"};
            return flashStr_Key_WiFiSSID;
        } break;
        case HStr_Key_WireDevAddress: {
            static const char flashStr_Key_WireDevAddress[] PROGMEM = {"wireDevAddress"};
            return flashStr_Key_WireDevAddress;
        } break;
        case HStr_Key_WirePosIndex: {
            static const char flashStr_Key_WirePosIndex[] PROGMEM = {"wirePosIndex"};
            return flashStr_Key_WirePosIndex;
        } break;

        case HStr_Enum_16x2LCD: {
            static const char flashStr_Enum_16x2LCD[] PROGMEM = {"16x2LCD"};
            return flashStr_Enum_16x2LCD;
        } break;
        case HStr_Enum_16x2LCDSwapped: {
            static const char flashStr_Enum_16x2LCDSwapped[] PROGMEM = {"16x2LCDSwapped"};
            return flashStr_Enum_16x2LCDSwapped;
        } break;
        case HStr_Enum_20x4LCD: {
            static const char flashStr_Enum_20x4LCD[] PROGMEM = {"20x4LCD"};
            return flashStr_Enum_20x4LCD;
        } break;
        case HStr_Enum_20x4LCDSwapped: {
            static const char flashStr_Enum_20x4LCDSwapped[] PROGMEM = {"20x4LCDSwapped"};
            return flashStr_Enum_20x4LCDSwapped;
        } break;
        case HStr_Enum_2x2Matrix: {
            static const char flashStr_Enum_2x2Matrix[] PROGMEM = {"2x2Matrix"};
            return flashStr_Enum_2x2Matrix;
        } break;
        case HStr_Enum_4xButton: {
            static const char flashStr_Enum_4xButton[] PROGMEM = {"4xButton"};
            return flashStr_Enum_4xButton;
        } break;
        case HStr_Enum_6xButton: {
            static const char flashStr_Enum_6xButton[] PROGMEM = {"6xButton"};
            return flashStr_Enum_6xButton;
        } break;
        case HStr_Enum_AC110V: {
            static const char flashStr_Enum_AC110V[] PROGMEM = {"AC110V"};
            return flashStr_Enum_AC110V;
        } break;
        case HStr_Enum_AC220V: {
            static const char flashStr_Enum_AC220V[] PROGMEM = {"AC220V"};
            return flashStr_Enum_AC220V;
        } break;
        case HStr_Enum_AnalogInput: {
            static const char flashStr_Enum_AnalogInput[] PROGMEM = {"AnalogInput"};
            return flashStr_Enum_AnalogInput;
        } break;
        case HStr_Enum_AnalogOutput: {
            static const char flashStr_Enum_AnalogOutput[] PROGMEM = {"AnalogOutput"};
            return flashStr_Enum_AnalogOutput;
        } break;
        case HStr_Enum_Angle: {
            static const char flashStr_Enum_Angle[] PROGMEM = {"Angle"};
            return flashStr_Enum_Angle;
        } break;
        case HStr_Enum_AscOrder: {
            static const char flashStr_Enum_AscOrder[] PROGMEM = {"AscOrder"};
            return flashStr_Enum_AscOrder;
        } break;
        case HStr_Enum_Average: {
            static const char flashStr_Enum_Average[] PROGMEM = {"Average"};
            return flashStr_Enum_Average;
        } break;
        case HStr_Enum_BrightnessBalancing: {
            static const char flashStr_Enum_BrightnessBalancing[] PROGMEM = {"BrightnessBalancing"};
            return flashStr_Enum_BrightnessBalancing;
        } break;
        case HStr_Enum_ContinuousServo: {
            static const char flashStr_Enum_ContinuousServo[] PROGMEM = {"ContinuousServo"};
            return flashStr_Enum_ContinuousServo;
        } break;
        case HStr_Enum_DC12V: {
            static const char flashStr_Enum_DC12V[] PROGMEM = {"DC12V"};
            return flashStr_Enum_DC12V;
        } break;
        case HStr_Enum_DC24V: {
            static const char flashStr_Enum_DC24V[] PROGMEM = {"DC24V"};
            return flashStr_Enum_DC24V;
        } break;
        case HStr_Enum_DC3V3: {
            static const char flashStr_Enum_DC3V3[] PROGMEM = {"DC3V3"};
            return flashStr_Enum_DC3V3;
        } break;
        case HStr_Enum_DC48V: {
            static const char flashStr_Enum_DC48V[] PROGMEM = {"DC48V"};
            return flashStr_Enum_DC48V;
        } break;
        case HStr_Enum_DC5V: {
            static const char flashStr_Enum_DC5V[] PROGMEM = {"DC5V"};
            return flashStr_Enum_DC5V;
        } break;
        case HStr_Enum_DesOrder: {
            static const char flashStr_Enum_DesOrder[] PROGMEM = {"DesOrder"};
            return flashStr_Enum_DesOrder;
        } break;
        case HStr_Enum_DigitalInput: {
            static const char flashStr_Enum_DigitalInput[] PROGMEM = {"DigitalInput"};
            return flashStr_Enum_DigitalInput;
        } break;
        case HStr_Enum_DigitalInputPullDown: {
            static const char flashStr_Enum_DigitalInputPullDown[] PROGMEM = {"DigitalInputPullDown"};
            return flashStr_Enum_DigitalInputPullDown;
        } break;
        case HStr_Enum_DigitalInputPullUp: {
            static const char flashStr_Enum_DigitalInputPullUp[] PROGMEM = {"DigitalInputPullUp"};
            return flashStr_Enum_DigitalInputPullUp;
        } break;
        case HStr_Enum_DigitalOutput: {
            static const char flashStr_Enum_DigitalOutput[] PROGMEM = {"DigitalOutput"};
            return flashStr_Enum_DigitalOutput;
        } break;
        case HStr_Enum_DigitalOutputPushPull: {
            static const char flashStr_Enum_DigitalOutputPushPull[] PROGMEM = {"DigitalOutputPushPull"};
            return flashStr_Enum_DigitalOutputPushPull;
        } break;
        case HStr_Enum_Distance: {
            static const char flashStr_Enum_Distance[] PROGMEM = {"Distance"};
            return flashStr_Enum_Distance;
        } break;
        case HStr_Enum_Endstop: {
            static const char flashStr_Enum_Endstop[] PROGMEM = {"Endstop"};
            return flashStr_Enum_Endstop;
        } break;
        case HStr_Enum_Equatorial: {
            static const char flashStr_Enum_Equatorial[] PROGMEM = {"Equatorial"};
            return flashStr_Enum_Equatorial;
        } break;
        case HStr_Enum_Gimballed: {
            static const char flashStr_Enum_Gimballed[] PROGMEM = {"Gimballed"};
            return flashStr_Enum_Gimballed;
        } break;
        case HStr_Enum_Highest: {
            static const char flashStr_Enum_Highest[] PROGMEM = {"Highest"};
            return flashStr_Enum_Highest;
        } break;
        case HStr_Enum_Horizontal: {
            static const char flashStr_Enum_Horizontal[] PROGMEM = {"Horizontal"};
            return flashStr_Enum_Horizontal;
        } break;
        case HStr_Enum_IceDetector: {
            static const char flashStr_Enum_IceDetector[] PROGMEM = {"IceDetector"};
            return flashStr_Enum_IceDetector;
        } break;
        case HStr_Enum_Imperial: {
            static const char flashStr_Enum_Imperial[] PROGMEM = {"Imperial"};
            return flashStr_Enum_Imperial;
        } break;
        case HStr_Enum_InOrder: {
            static const char flashStr_Enum_InOrder[] PROGMEM = {"InOrder"};
            return flashStr_Enum_InOrder;
        } break;
        case HStr_Enum_LightIntensity: {
            static const char flashStr_Enum_LightIntensity[] PROGMEM = {"LightIntensity"};
            return flashStr_Enum_LightIntensity;
        } break;
        case HStr_Enum_LinearActuator: {
            static const char flashStr_Enum_LinearActuator[] PROGMEM = {"LinearActuator"};
            return flashStr_Enum_LinearActuator;
        } break;
        case HStr_Enum_Lowest: {
            static const char flashStr_Enum_Lowest[] PROGMEM = {"Lowest"};
            return flashStr_Enum_Lowest;
        } break;
        case HStr_Enum_Metric: {
            static const char flashStr_Enum_Metric[] PROGMEM = {"Metric"};
            return flashStr_Enum_Metric;
        } break;
        case HStr_Enum_Multiply: {
            static const char flashStr_Enum_Multiply[] PROGMEM = {"Multiply"};
            return flashStr_Enum_Multiply;
        } break;
        case HStr_Enum_PanelHeater: {
            static const char flashStr_Enum_PanelHeater[] PROGMEM = {"PanelHeater"};
            return flashStr_Enum_PanelHeater;
        } break;
        case HStr_Enum_PanelSprayer: {
            static const char flashStr_Enum_PanelSprayer[] PROGMEM = {"PanelSprayer"};
            return flashStr_Enum_PanelSprayer;
        } break;
        case HStr_Enum_Percentile: {
            static const char flashStr_Enum_Percentile[] PROGMEM = {"Percentile"};
            return flashStr_Enum_Percentile;
        } break;
        case HStr_Enum_PositionalServo: {
            static const char flashStr_Enum_PositionalServo[] PROGMEM = {"PositionalServo"};
            return flashStr_Enum_PositionalServo;
        } break;
        case HStr_Enum_PositionCalculating: {
            static const char flashStr_Enum_PositionCalculating[] PROGMEM = {"PositionCalculating"};
            return flashStr_Enum_PositionCalculating;
        } break;
        case HStr_Enum_Power: {
            static const char flashStr_Enum_Power[] PROGMEM = {"Power"};
            return flashStr_Enum_Power;
        } break;
        case HStr_Enum_PowerLevel: {
            static const char flashStr_Enum_PowerLevel[] PROGMEM = {"PowerLevel"};
            return flashStr_Enum_PowerLevel;
        } break;
        case HStr_Enum_RevOrder: {
            static const char flashStr_Enum_RevOrder[] PROGMEM = {"RevOrder"};
            return flashStr_Enum_RevOrder;
        } break;
        case HStr_Enum_RotaryEncoder: {
            static const char flashStr_Enum_RotaryEncoder[] PROGMEM = {"RotaryEncoder"};
            return flashStr_Enum_RotaryEncoder;
        } break;
        case HStr_Enum_Scientific: {
            static const char flashStr_Enum_Scientific[] PROGMEM = {"Scientific"};
            return flashStr_Enum_Scientific;
        } break;
        case HStr_Enum_Speed: {
            static const char flashStr_Enum_Speed[] PROGMEM = {"Speed"};
            return flashStr_Enum_Speed;
        } break;
        case HStr_Enum_StokePosition: {
            static const char flashStr_Enum_StokePosition[] PROGMEM = {"StokePosition"};
            return flashStr_Enum_StokePosition;
        } break;
        case HStr_Enum_Temperature: {
            static const char flashStr_Enum_Temperature[] PROGMEM = {"Temperature"};
            return flashStr_Enum_Temperature;
        } break;
        case HStr_Enum_TempHumidity: {
            static const char flashStr_Enum_TempHumidity[] PROGMEM = {"TempHumidity"};
            return flashStr_Enum_TempHumidity;
        } break;
        case HStr_Enum_TiltAngle: {
            static const char flashStr_Enum_TiltAngle[] PROGMEM = {"TiltAngle"};
            return flashStr_Enum_TiltAngle;
        } break;
        case HStr_Enum_Vertical: {
            static const char flashStr_Enum_Vertical[] PROGMEM = {"Vertical"};
            return flashStr_Enum_Vertical;
        } break;
        case HStr_Enum_WindSpeed: {
            static const char flashStr_Enum_WindSpeed[] PROGMEM = {"WindSpeed"};
            return flashStr_Enum_WindSpeed;
        } break;

        case HStr_Unit_Celsius: {
            static const char flashStr_Unit_Celsius[] PROGMEM = {"°C"};
            return flashStr_Unit_Celsius;
        } break;
        case HStr_Unit_Count: {
            static const char flashStr_Unit_Count[] PROGMEM = {"[qty]"};
            return flashStr_Unit_Count;
        } break;
        case HStr_Unit_Fahrenheit: {
            static const char flashStr_Unit_Fahrenheit[] PROGMEM = {"°F"};
            return flashStr_Unit_Fahrenheit;
        } break;
        case HStr_Unit_Feet: {
            static const char flashStr_Unit_Feet[] PROGMEM = {"ft"};
            return flashStr_Unit_Feet;
        } break;
        case HStr_Unit_Kelvin: {
            static const char flashStr_Unit_Kelvin[] PROGMEM = {"°K"};
            return flashStr_Unit_Kelvin;
        } break;
        case HStr_Unit_PerMinute: {
            static const char flashStr_Unit_PerMinute[] PROGMEM = {"/min"};
            return flashStr_Unit_PerMinute;
        } break;
        case HStr_Unit_Radians: {
            static const char flashStr_Unit_Radians[] PROGMEM = {"°rad"};
            return flashStr_Unit_Radians;
        } break;
        case HStr_Unit_Undefined: {
            static const char flashStr_Unit_Undefined[] PROGMEM = {"[undef]"};
            return flashStr_Unit_Undefined;
        } break;
    }
}

#endif

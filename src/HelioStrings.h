/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Strings
*/

#ifndef HelioStrings_H
#define HelioStrings_H

// Strings Enumeration Table
enum Helio_String : unsigned short {
    HStr_ColonSpace,
    HStr_DoubleSpace,
    HStr_csv,
    HStr_dat,
    HStr_Disabled,
    HStr_raw,
    HStr_txt,
    HStr_Undefined,
    HStr_null,

    HStr_Default_SystemName,
    HStr_Default_ConfigFilename,

    HStr_UI_MatrixActions,
    HStr_UI_Matrix3x4Keys,
    HStr_UI_Matrix4x4Keys,

    HStr_Err_AllocationFailure,
    HStr_Err_AlreadyInitialized,
    HStr_Err_AssertionFailure,
    HStr_Err_ExportFailure,
    HStr_Err_ImportFailure,
    HStr_Err_InitializationFailure,
    HStr_Err_InvalidParameter,
    HStr_Err_InvalidPinOrType,
    HStr_Err_MeasurementFailure,
    HStr_Err_MissingLinkage,
    HStr_Err_NoPositionsAvailable,
    HStr_Err_NotConfiguredProperly,
    HStr_Err_NotYetInitialized,
    HStr_Err_OperationFailure,
    HStr_Err_UnsupportedOperation,

    HStr_Log_CalculatedTravel,
    HStr_Log_CoverSequence,
    HStr_Log_EnvReport,
    HStr_Log_HasBegan,
    HStr_Log_HasDisabled,
    HStr_Log_HasEnabled,
    HStr_Log_HasEnded,
    HStr_Log_MeasuredTravel,
    HStr_Log_NightSequence,
    HStr_Log_PreDawnCleaning,
    HStr_Log_PreDawnWarmup,
    HStr_Log_RTCBatteryFailure,
    HStr_Log_StormingSequence,
    HStr_Log_SystemDataSaved,
    HStr_Log_SystemUptime,
    HStr_Log_TrackingSequence,
    HStr_Log_UncoverSequence,

    HStr_Log_Prefix_Info,
    HStr_Log_Prefix_Warning,
    HStr_Log_Prefix_Error,

    HStr_Log_Field_Cleaning_Duration,
    HStr_Log_Field_Heating_Duration,
    HStr_Log_Field_Light_Duration,
    HStr_Log_Field_Solar_Panel,
    HStr_Log_Field_Temp_Measured,
    HStr_Log_Field_Time_Calculated,
    HStr_Log_Field_Time_Finish,
    HStr_Log_Field_Time_Measured,
    HStr_Log_Field_Time_Progress,
    HStr_Log_Field_Time_Start,
    HStr_Log_Field_Travel_Calculated,
    HStr_Log_Field_Travel_Measured,
    HStr_Log_Field_WindSpeed_Measured,

    HStr_Key_ActiveLow,
    HStr_Key_AlignedTolerance,
    HStr_Key_AutosaveEnabled,
    HStr_Key_AutosaveFallback,
    HStr_Key_AutosaveInterval,
    HStr_Key_AxisOffset,
    HStr_Key_AxisPosition,
    HStr_Key_AxisSensorHorz,
    HStr_Key_AxisSensorVert,
    HStr_Key_BitRes,
    HStr_Key_CalibrationUnits,
    HStr_Key_Channel,
    HStr_Key_ChannelPins,
    HStr_Key_ChipEnablePin,
    HStr_Key_CleaningIntervalDays,
    HStr_Key_ComputeHeatIndex,
    HStr_Key_ContinuousPowerUsage,
    HStr_Key_ContinuousSpeed,
    HStr_Key_CtrlInMode,
    HStr_Key_DailyLightHours,
    HStr_Key_DataFilePrefix,
    HStr_Key_DetriggerDelay,
    HStr_Key_DetriggerTol,
    HStr_Key_DHTType,
    HStr_Key_DisplayTheme,
    HStr_Key_DispOutMode,
    HStr_Key_DistanceUnits,
    HStr_Key_EnableMode,
    HStr_Key_Flags,
    HStr_Key_HeatingTrigger,
    HStr_Key_HomePosition,
    HStr_Key_Id,
    HStr_Key_InputInversion,
    HStr_Key_InputPin,
    HStr_Key_JoystickCalib,
    HStr_Key_LastAlignedTime,
    HStr_Key_LastCleanedTime,
    HStr_Key_LDRSensorHorzMin,
    HStr_Key_LDRSensorHorzMax,
    HStr_Key_LDRSensorVertMin,
    HStr_Key_LDRSensorVertMax,
    HStr_Key_LimitTrigger,
    HStr_Key_Location,
    HStr_Key_LocationOffset,
    HStr_Key_LogFilePrefix,
    HStr_Key_LogLevel,
    HStr_Key_LogToSDCard,
    HStr_Key_LogToWiFiStorage,
    HStr_Key_Logger,
    HStr_Key_MACAddress,
    HStr_Key_MaxActiveAtOnce,
    HStr_Key_MaxPower,
    HStr_Key_MeasureMode,
    HStr_Key_MeasurementRow,
    HStr_Key_MeasurementUnits,
    HStr_Key_MinIntensity,
    HStr_Key_Mode,
    HStr_Key_Multiplier,
    HStr_Key_Offset,
    HStr_Key_OutputPin,
    HStr_Key_OutputPin2,
    HStr_Key_PanelName,
    HStr_Key_Pin,
    HStr_Key_PollingInterval,
    HStr_Key_PositionSensor,
    HStr_Key_PowerProductionSensor,
    HStr_Key_PowerUsageSensor,
    HStr_Key_PowerUnits,
    HStr_Key_PreDawnCleaningMins,
    HStr_Key_PreDawnHeatingMins,
    HStr_Key_PublishToSDCard,
    HStr_Key_PublishToWiFiStorage,
    HStr_Key_Publisher,
    HStr_Key_PWMChannel,
    HStr_Key_PWMFrequency,
    HStr_Key_RailName,
    HStr_Key_ReflectPosition,
    HStr_Key_ReportInterval,
    HStr_Key_Revision,
    HStr_Key_Scheduler,
    HStr_Key_SensorName,
    HStr_Key_SignalPin,
    HStr_Key_SpeedSensor,
    HStr_Key_State,
    HStr_Key_StormingTrigger,
    HStr_Key_SystemMode,
    HStr_Key_SystemName,
    HStr_Key_TemperatureUnits,
    HStr_Key_TemperatureSensor,
    HStr_Key_TimeZoneOffset,
    HStr_Key_Timestamp,
    HStr_Key_Tolerance,
    HStr_Key_ToleranceHigh,
    HStr_Key_ToleranceLow,
    HStr_Key_TriggerBelow,
    HStr_Key_TriggerOutside,
    HStr_Key_Type,
    HStr_Key_Units,
    HStr_Key_UpdatesPerSec,
    HStr_Key_UsingISR,
    HStr_Key_Value,
    HStr_Key_Version,
    HStr_Key_WiFiPassword,
    HStr_Key_WiFiPasswordSeed,
    HStr_Key_WiFiSSID,
    HStr_Key_WindSpeedSensor,
    HStr_Key_WireDevAddress,
    HStr_Key_WirePosIndex,

    HStr_Enum_AC110V,
    HStr_Enum_AC220V,
    HStr_Enum_AnalogInput,
    HStr_Enum_AnalogJoystick,
    HStr_Enum_AnalogOutput,
    HStr_Enum_Angle,
    HStr_Enum_AscOrder,
    HStr_Enum_Average,
    HStr_Enum_Balancing,
    HStr_Enum_ContinuousServo,
    HStr_Enum_CustomOLED,
    HStr_Enum_DC12V,
    HStr_Enum_DC24V,
    HStr_Enum_DC3V3,
    HStr_Enum_DC48V,
    HStr_Enum_DC5V,
    HStr_Enum_DescOrder,
    HStr_Enum_DigitalInput,
    HStr_Enum_DigitalInputPullDown,
    HStr_Enum_DigitalInputPullUp,
    HStr_Enum_DigitalOutput,
    HStr_Enum_DigitalOutputPushPull,
    HStr_Enum_Distance,
    HStr_Enum_Equatorial,
    HStr_Enum_Gimballed,
    HStr_Enum_Highest,
    HStr_Enum_Horizontal,
    HStr_Enum_IceDetector,
    HStr_Enum_IL3820,
    HStr_Enum_IL3820V2,
    HStr_Enum_ILI9341,
    HStr_Enum_Imperial,
    HStr_Enum_InOrder,
    HStr_Enum_LCD16x2,
    HStr_Enum_LCD20x4,
    HStr_Enum_LightIntensity,
    HStr_Enum_LinearActuator,
    HStr_Enum_Lowest,
    HStr_Enum_Matrix2x2,
    HStr_Enum_Matrix3x4,
    HStr_Enum_Matrix4x4,
    HStr_Enum_Metric,
    HStr_Enum_Multiply,
    HStr_Enum_PanelBrake,
    HStr_Enum_PanelCover,
    HStr_Enum_PanelHeater,
    HStr_Enum_PanelSprayer,
    HStr_Enum_Percentile,
    HStr_Enum_PositionalServo,
    HStr_Enum_Power,
    HStr_Enum_PowerProduction,
    HStr_Enum_PowerUsage,
    HStr_Enum_RemoteControl,
    HStr_Enum_ResistiveTouch,
    HStr_Enum_RevOrder,
    HStr_Enum_RotaryEncoder,
    HStr_Enum_Scientific,
    HStr_Enum_SH1106,
    HStr_Enum_Speed,
    HStr_Enum_SSD1305,
    HStr_Enum_SSD1305x32Ada,
    HStr_Enum_SSD1305x64Ada,
    HStr_Enum_SSD1306,
    HStr_Enum_SSD1607,
    HStr_Enum_ST7735,
    HStr_Enum_ST7789,
    HStr_Enum_Temperature,
    HStr_Enum_TFTTouch,
    HStr_Enum_TiltAngle,
    HStr_Enum_TouchScreen,
    HStr_Enum_Tracking,
    HStr_Enum_TravelPosition,
    HStr_Enum_UpDownButtons,
    HStr_Enum_UpDownESP32Touch,
    HStr_Enum_Vertical,
    HStr_Enum_WindSpeed,

    HStr_Unit_Count,
    HStr_Unit_Degree,
    HStr_Unit_Feet,
    HStr_Unit_Minutes,
    HStr_Unit_PerMinute,
    HStr_Unit_Radians,
    HStr_Unit_Undefined,

    HStr_Count
};

// Returns memory resident string from PROGMEM (Flash) string enumeration.
extern String stringFromPGM(Helio_String strNum);
#define SFP(strNum) stringFromPGM((strNum))

// Returns memory resident string from PROGMEM (Flash) string address.
String stringFromPGMAddr(const char *flashStr);

// Makes Strings lookup go through EEPROM, with specified data begin address.
extern void beginStringsFromEEPROM(uint16_t dataAddress);

// Makes Strings lookup go through SD card strings file at file prefix.
extern void beginStringsFromSDCard(String dataFilePrefix);

#endif // /ifndef HelioStrings_H

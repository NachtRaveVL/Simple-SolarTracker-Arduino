// Enum to CPP export tests script - mainly for dev purposes

#include <Helioduino.h>

#ifdef HELIO_DISABLE_BUILTIN_DATA
#error The HELIO_DISABLE_BUILTIN_DATA flag is expected to be undefined in order to run this sketch
#endif

/// Pins & Class Instances
#define SETUP_PIEZO_BUZZER_PIN          -1              // Piezo buzzer pin, else -1
#define SETUP_EEPROM_DEVICE_TYPE        None            // EEPROM device type/size (24LC01, 24LC02, 24LC04, 24LC08, 24LC16, 24LC32, 24LC64, 24LC128, 24LC256, 24LC512, None)
#define SETUP_EEPROM_I2C_ADDR           B000            // EEPROM i2c address
#define SETUP_RTC_I2C_ADDR              B000            // RTC i2c address (only B000 can be used atm)
#define SETUP_RTC_DEVICE_TYPE           None            // RTC device type (DS1307, DS3231, PCF8523, PCF8563, None)
#define SETUP_SD_CARD_SPI               SPI             // SD card SPI class instance
#define SETUP_SD_CARD_SPI_CS            -1              // SD card CS pin, else -1
#define SETUP_SD_CARD_SPI_SPEED         F_SPD           // SD card SPI speed, in Hz (ignored on Teensy)
#define SETUP_I2C_WIRE                  Wire            // I2C wire class instance
#define SETUP_I2C_SPEED                 400000U         // I2C speed, in Hz
#define SETUP_ESP_I2C_SDA               SDA             // I2C SDA pin, if on ESP
#define SETUP_ESP_I2C_SCL               SCL             // I2C SCL pin, if on ESP

Helioduino helioController((pintype_t)SETUP_PIEZO_BUZZER_PIN,
                           JOIN(Helio_EEPROMType,SETUP_EEPROM_DEVICE_TYPE),
                           I2CDeviceSetup((uint8_t)SETUP_EEPROM_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
                           JOIN(Helio_RTCType,SETUP_RTC_DEVICE_TYPE),
                           I2CDeviceSetup((uint8_t)SETUP_RTC_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
                           SPIDeviceSetup((pintype_t)SETUP_SD_CARD_SPI_CS, &SETUP_SD_CARD_SPI, SETUP_SD_CARD_SPI_SPEED));

void testSystemModeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_SystemMode_Count; ++typeIndex) {
        String typeString = systemModeToString((Helio_SystemMode)typeIndex);
        int retTypeIndex = (int)systemModeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testSystemModeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testMeasurementModeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_MeasurementMode_Count; ++typeIndex) {
        String typeString = measurementModeToString((Helio_MeasurementMode)typeIndex);
        int retTypeIndex = (int)measurementModeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testMeasurementModeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testDisplayOutputModeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_DisplayOutputMode_Count; ++typeIndex) {
        String typeString = displayOutputModeToString((Helio_DisplayOutputMode)typeIndex);
        int retTypeIndex = (int)displayOutputModeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testDisplayOutputModeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testControlInputModeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_ControlInputMode_Count; ++typeIndex) {
        String typeString = controlInputModeToString((Helio_ControlInputMode)typeIndex);
        int retTypeIndex = (int)controlInputModeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testControlInputModeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testActuatorTypeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_ActuatorType_Count; ++typeIndex) {
        String typeString = actuatorTypeToString((Helio_ActuatorType)typeIndex);
        int retTypeIndex = (int)actuatorTypeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testActuatorTypeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testSensorTypeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_SensorType_Count; ++typeIndex) {
        String typeString = sensorTypeToString((Helio_SensorType)typeIndex);
        int retTypeIndex = (int)sensorTypeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testSensorTypeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testPanelTypeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_PanelType_Count; ++typeIndex) {
        String typeString = panelTypeToString((Helio_PanelType)typeIndex);
        int retTypeIndex = (int)panelTypeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testPanelTypeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testRailTypeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_RailType_Count; ++typeIndex) {
        String typeString = railTypeToString((Helio_RailType)typeIndex);
        int retTypeIndex = (int)railTypeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testRailTypeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testPinModeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_PinMode_Count; ++typeIndex) {
        String typeString = pinModeToString((Helio_PinMode)typeIndex);
        int retTypeIndex = (int)pinModeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testPinModeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testEnableModeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_EnableMode_Count; ++typeIndex) {
        String typeString = enableModeToString((Helio_EnableMode)typeIndex);
        int retTypeIndex = (int)enableModeFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testEnableModeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testUnitsCategoryEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_UnitsCategory_Count; ++typeIndex) {
        String typeString = unitsCategoryToString((Helio_UnitsCategory)typeIndex);
        int retTypeIndex = (int)unitsCategoryFromString(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testUnitsCategoryEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void testUnitsTypeEnums()
{
    for (int typeIndex = -1; typeIndex <= Helio_UnitsType_Count; ++typeIndex) {
        String typeString = unitsTypeToSymbol((Helio_UnitsType)typeIndex);
        int retTypeIndex = (int)unitsTypeFromSymbol(typeString);
        if (typeIndex != retTypeIndex) {
            getLogger()->logError(F("testUnitsTypeEnums: Conversion failure: "), String(typeIndex));
            getLogger()->logError(F("  Invalid return: "), String(retTypeIndex), String(F(" (")) + typeString + String(F(")")));
        }
    }
}

void setup() {
    // Setup base interfaces
    #ifdef HELIO_ENABLE_DEBUG_OUTPUT
        Serial.begin(115200);           // Begin USB Serial interface
        while (!Serial) { ; }           // Wait for USB Serial to connect
    #endif
    #if defined(ESP_PLATFORM)
        SETUP_I2C_WIRE.begin(SETUP_ESP_I2C_SDA, SETUP_ESP_I2C_SCL); // Begin i2c Wire for ESP
    #endif

    helioController.init();

    getLogger()->logMessage(F("=BEGIN="));

    testSystemModeEnums();
    testMeasurementModeEnums();
    testDisplayOutputModeEnums();
    testControlInputModeEnums();
    testActuatorTypeEnums();
    testSensorTypeEnums();
    testPanelTypeEnums();
    testRailTypeEnums();
    testPinModeEnums();
    testEnableModeEnums();
    testUnitsCategoryEnums();
    testUnitsTypeEnums();

    getLogger()->logMessage(F("=FINISH="));
}

void loop()
{ ; }

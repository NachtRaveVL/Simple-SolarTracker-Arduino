// Enum to CPP export script - mainly for dev purposes

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

struct TreeNode;
static TreeNode *_root;

// Trie Encoding Tree
struct TreeNode {
    String piece;
    Map<char, TreeNode *> map;
    bool end;
    int typeIndex;

    TreeNode(String slice) : piece(slice), end(false) { ; }
    TreeNode(String slice, int typeIndexIn) : piece(slice), end(true), typeIndex(typeIndexIn) { ; }

    ~TreeNode() {
        for (auto iter = map.begin(); iter != map.end(); ++iter) {
            delete iter->second;
        }
    }

    // Inserts fragment into node with given parent (or null for root) and typeIndex
    void insert(TreeNode *parent, String fragment, int typeIndexIn) {
        if (fragment.length()) {
            for (int i = 0; i <= piece.length() && i <= fragment.length(); ++i) {
                if (i == piece.length()) {
                    if (!map[fragment[i]]) {
                        map[fragment[i]] = new TreeNode(fragment.substring(i), typeIndexIn);
                    } else {
                        map[fragment[i]]->insert(this, fragment.substring(i), typeIndexIn);
                    }
                    return;
                }
                if (i == fragment.length() || piece[i] != fragment[i]) {
                    String pieceSplit = piece.substring(i);

                    if (pieceSplit.length()) {
                        auto newParent = new TreeNode(piece.substring(0, i));
                        piece = pieceSplit;

                        if (parent) {
                            parent->map[newParent->piece[0]] = newParent;
                        } else {
                            _root = newParent;
                        }
                        newParent->map[piece[0]] = this;

                        newParent->insert(parent, fragment, typeIndexIn);

                        return;
                    } else {
                        end = true;
                        typeIndex = typeIndexIn;
                    }
                    return;
                }
            }
        } else {
            end = true;
            typeIndex = typeIndexIn;
        }
    }

    // Prints tab spaces in front
    void printSpacer(int level) {
        for (int i = 0; i < (level << 2); ++i) { Serial.print(' '); }
    }

    // Prints encoding tree out for debug
    void printDebug(int level) {
        printSpacer(level);
        Serial.print('-'); Serial.print(' '); Serial.println(piece);
        for (auto iter = map.begin(); iter != map.end(); ++iter) {
            iter->second->printDebug(level+1);
        }
    }

    // Prints encoding tree out in code
    void printCode(int level, const String &varName, const String &typeCast, int index = 0) {
        bool printedSwitch = false;

        if (end) {
            if (!map.size()) {
                printSpacer(level);
            } else {
                if (!printedSwitch) {
                    printedSwitch = true;
                    printSpacer(level);
                    Serial.print(F("switch (")); Serial.print(varName); Serial.print(F(".length() >= ")); Serial.print(index + 1); Serial.print(F(" ? ")); Serial.print(varName); Serial.print(F("[")); Serial.print(index); Serial.println(F("] : '\\0') {"));
                }
                printSpacer(level + 1);
                Serial.println(F("case '\\0':"));
                printSpacer(level + 2);
            }
            Serial.print(F("return ")); Serial.print(typeCast); Serial.print(typeIndex); Serial.println(F(";"));
        }

        for (auto iter = map.begin(); iter != map.end(); ++iter) {
            if (!printedSwitch) {
                printedSwitch = true;
                printSpacer(level);
                Serial.print(F("switch (")); Serial.print(varName); Serial.print(F(".length() >= ")); Serial.print(index + 1); Serial.print(F(" ? ")); Serial.print(varName); Serial.print(F("[")); Serial.print(index); Serial.println(F("] : '\\0') {"));
            }
            printSpacer(level + 1);
            Serial.print(F("case '"));
            if (iter->first >= ' ') { Serial.print(iter->first); }
            else { Serial.print('\\'); Serial.print((int)iter->first); }
            Serial.println(F("':"));

            iter->second->printCode(level + 2, varName, typeCast, index + iter->second->piece.length());

            if (!(iter->second->end && !iter->second->map.size())) {
                printSpacer(level + 2);
                Serial.println(F("break;"));
            }
        }

        if (printedSwitch) {
            for (int i = 0; i < (level << 2); ++i) { Serial.print(' '); }
            Serial.println(F("}"));
        }
    }
};

void buildSystemModeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_SystemMode_Count; ++typeIndex) {
        _root->insert(nullptr, systemModeToString((Helio_SystemMode)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_SystemMode)"));
    String varName(F("systemModeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildMeasurementModeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_MeasurementMode_Count; ++typeIndex) {
        _root->insert(nullptr, measurementModeToString((Helio_MeasurementMode)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_MeasurementMode)"));
    String varName(F("measurementModeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildDisplayOutputModeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_DisplayOutputMode_Count; ++typeIndex) {
        _root->insert(nullptr, displayOutputModeToString((Helio_DisplayOutputMode)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_DisplayOutputMode)"));
    String varName(F("displayOutModeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildControlInputModeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_ControlInputMode_Count; ++typeIndex) {
        _root->insert(nullptr, controlInputModeToString((Helio_ControlInputMode)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_ControlInputMode)"));
    String varName(F("controlInModeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildActuatorTypeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_ActuatorType_Count; ++typeIndex) {
        _root->insert(nullptr, actuatorTypeToString((Helio_ActuatorType)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_ActuatorType)"));
    String varName(F("actuatorTypeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildSensorTypeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_SensorType_Count; ++typeIndex) {
        _root->insert(nullptr, sensorTypeToString((Helio_SensorType)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_SensorType)"));
    String varName(F("sensorTypeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildPanelTypeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_PanelType_Count; ++typeIndex) {
        _root->insert(nullptr, panelTypeToString((Helio_PanelType)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_PanelType)"));
    String varName(F("panelTypeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildRailTypeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_RailType_Count; ++typeIndex) {
        _root->insert(nullptr, railTypeToString((Helio_RailType)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_RailType)"));
    String varName(F("railTypeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildPinModeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_PinMode_Count; ++typeIndex) {
        _root->insert(nullptr, pinModeToString((Helio_PinMode)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_PinMode)"));
    String varName(F("pinModeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildEnableModeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_EnableMode_Count; ++typeIndex) {
        _root->insert(nullptr, enableModeToString((Helio_EnableMode)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_EnableMode)"));
    String varName(F("enableModeStr"));
    _root->printCode(1, varName, typeCast);
}

void buildUnitsCategoryTree() {
    if (_root) { delete _root; } _root = new TreeNode("");
    for (int typeIndex = -1; typeIndex <= Helio_UnitsCategory_Count; ++typeIndex) {
        _root->insert(nullptr, unitsCategoryToString((Helio_UnitsCategory)typeIndex), typeIndex);
    }
    String typeCast(F("(Helio_UnitsCategory)"));
    String varName(F("unitsCategoryStr"));
    _root->printCode(1, varName, typeCast);
}

void printUnitsTypeTree() {
    String typeCast(F("(Helio_UnitsType)"));
    String varName(F("unitsSymbolStr"));
    _root->printCode(1, varName, typeCast);
    delete _root; _root = new TreeNode("");
}

void buildUnitsTypeTree() {
    if (_root) { delete _root; } _root = new TreeNode("");

    for (char charIndex = 'A'; charIndex <= 'Z'; ++charIndex) {
        if (freeMemory() < 2000 && _root->map.size()) { printUnitsTypeTree(); }
        for (int typeIndex = -1; typeIndex <= Helio_UnitsType_Count; ++typeIndex) {
            String unitsType = unitsTypeToSymbol((Helio_UnitsType)typeIndex);
            if (toupper(unitsType[0]) == charIndex) {
                _root->insert(nullptr, unitsType, typeIndex);
            }
        }
        // aliases
        if (charIndex == 'J') { _root->insert(nullptr, F("J/s"), Helio_UnitsType_Power_Wattage); }
    }

    for (int typeIndex = -1; typeIndex <= Helio_UnitsType_Count; ++typeIndex) {
        String unitsType = unitsTypeToSymbol((Helio_UnitsType)typeIndex);
        if (toupper(unitsType[0]) < 'A' || toupper(unitsType[0]) > 'Z') {
            _root->insert(nullptr, unitsType, typeIndex);
        }
    }

    if (_root->map.size()) { printUnitsTypeTree(); }
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

    getLogger()->logMessage(F("Writing enum decoding tree..."));

    getLogger()->logMessage(F("System mode tree:"));
    buildSystemModeTree();

    getLogger()->logMessage(F("Measurements tree:"));
    buildMeasurementModeTree();

    getLogger()->logMessage(F("Display output tree:"));
    buildDisplayOutputModeTree();

    getLogger()->logMessage(F("Control input tree:"));
    buildControlInputModeTree();

    getLogger()->logMessage(F("Actuator type tree:"));
    buildActuatorTypeTree();

    getLogger()->logMessage(F("Sensor type tree:"));
    buildSensorTypeTree();

    getLogger()->logMessage(F("Panel type tree:"));
    buildPanelTypeTree();

    getLogger()->logMessage(F("Rail type tree:"));
    buildRailTypeTree();

    getLogger()->logMessage(F("Pin mode tree:"));
    buildPinModeTree();

    getLogger()->logMessage(F("Enable mode tree:"));
    buildEnableModeTree();

    getLogger()->logMessage(F("Units category tree:"));
    buildUnitsCategoryTree();

    getLogger()->logMessage(F("Units type tree: (multiple switches)"));
    buildUnitsTypeTree();

    getLogger()->logMessage(F("Done!"));
    if (_root) { delete _root; _root = nullptr; }
}

void loop()
{ ; }

// Simple-SolarTracking-Arduino Data Writer Example
//
// In this example we program an SD card or EEPROM to hold onto the various data items
// that normally are built into onboard device Flash / non-volatile memory.
// 
// Since dead code is stripped out of the final binary on most Arduino-like build
// processes, we can take advantage of that fact to create an "empty" system that only
// does one thing: programs the SD card or EEPROM attached to it. As such, this sketch
// can be compiled to run in such a constrained state to "prep" said storage, and thus
// we can offload any program constant data. Hopefully this is to such a point that it
// allows another system/example to compile and possibly fit into a constrained device.
//
// For binary storage, endianness is not a concern this way since the same device does
// both the writing & reading of data. For SD card storage, we can utilize human-readable
// JSON in pretty-print mode since that space is relatively cheap.
//
// For EEPROM device writing, make sure that any EEPROM Write-Protect jumpers are disabled,
// and keep track of the produced output that defines the various data locations written to
// the EEPROM chip (e.g. SYSDATA_ADDR, STRINGS_ADDR, etc.). These will need to be copied
// over into other sketches to prep their storage offsets. TODO: Deprecated, #6 in Hydruino.
//
// Also, make sure you have not defined HELIO_DISABLE_BUILTIN_DATA so that the full data
// is built into the onboard Flash. You may also enable Serial log output by defining
// HELIO_ENABLE_DEBUG_OUTPUT.
// You may refer to: https://forum.arduino.cc/index.php?topic=602603.0 on how to define
// custom build flags manually via modifying platform[.local].txt.
//
// In Helioduino.h:
// 
// // Uncomment or -D this define to enable external data storage (SD card or EEPROM) to save on sketch size. Required for constrained devices.
// //#define HELIO_DISABLE_BUILTIN_DATA              // Disables library data existing in Flash, see DataWriter example for exporting details
// 
// // Uncomment or -D this define to enable debug output (treats Serial output as attached to serial monitor).
// #define HELIO_ENABLE_DEBUG_OUTPUT
//
// Alternatively, in platform[.local].txt:
// build.extra_flags=-DHELIO_ENABLE_DEBUG_OUTPUT

#include <Helioduino.h>

// Compiler flag checks
#ifdef HELIO_DISABLE_BUILTIN_DATA
#error The HELIO_DISABLE_BUILTIN_DATA flag is expected to be undefined in order to run this sketch
#endif

// Pins & Class Instances
#define SETUP_PIEZO_BUZZER_PIN          -1              // Piezo buzzer pin, else -1
#define SETUP_EEPROM_DEVICE_TYPE        None            // EEPROM device type/size (AT24LC01, AT24LC02, AT24LC04, AT24LC08, AT24LC16, AT24LC32, AT24LC64, AT24LC128, AT24LC256, AT24LC512, None)
#define SETUP_EEPROM_I2C_ADDR           0b000           // EEPROM i2c address (A0-A2, bitwise or'ed with base address 0x50)
#define SETUP_RTC_DEVICE_TYPE           None            // RTC device type (DS1307, DS3231, PCF8523, PCF8563, None)
#define SETUP_SD_CARD_SPI               SPI             // SD card SPI class instance
#define SETUP_SD_CARD_SPI_CS            -1              // SD card CS pin, else -1
#define SETUP_SD_CARD_SPI_SPEED         F_SPD           // SD card SPI speed, in Hz (ignored on Teensy)
#define SETUP_I2C_WIRE                  Wire            // I2C wire class instance
#define SETUP_I2C_SPEED                 400000U         // I2C speed, in Hz
#define SETUP_ESP_I2C_SDA               SDA             // I2C SDA pin, if on ESP
#define SETUP_ESP_I2C_SCL               SCL             // I2C SCL pin, if on ESP

// External Data Settings
#define SETUP_EXTDATA_SD_ENABLE         true            // If data should be written to an external SD card
#define SETUP_EXTDATA_SD_LIB_PREFIX     "lib/"          // Library data folder/data file prefix (appended with {type}##.dat)
#define SETUP_EXTDATA_EEPROM_ENABLE     true            // If data should be written to an external EEPROM
#define SETUP_EXTDATA_EEPROM_BEG_ADDR   0               // Start data address for data to be written to EEPROM

Helioduino helioController((pintype_t)SETUP_PIEZO_BUZZER_PIN,
                           JOIN(Helio_EEPROMType,SETUP_EEPROM_DEVICE_TYPE),
                           I2CDeviceSetup((uint8_t)SETUP_EEPROM_I2C_ADDR, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
                           JOIN(Helio_RTCType,SETUP_RTC_DEVICE_TYPE),
                           I2CDeviceSetup((uint8_t)0b000, &SETUP_I2C_WIRE, SETUP_I2C_SPEED),
                           SPIDeviceSetup((pintype_t)SETUP_SD_CARD_SPI_CS, &SETUP_SD_CARD_SPI, SETUP_SD_CARD_SPI_SPEED));

// Wraps a formatted address as appended pseudo alt text, e.g. " (0xADDR)"
String altAddressToString(uint16_t addr)
{
    String retVal;
    retVal.concat(' '); retVal.concat('('); 
    retVal.concat(addressToString(addr));
    retVal.concat(')');
    return retVal;
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

    // Just a lone initializer is all that's needed since we won't actually be using the full controller.
    helioController.init();

    getLogger()->logMessage(F("Writing external data..."));

    #if SETUP_EXTDATA_SD_ENABLE
    {   auto sd = getController()->getSDCard();

        if (sd) {
            {   getLogger()->logMessage(F("=== Writing string data to SD card ==="));

                uint16_t lookupTable[HStr_Count];

                // Initializes lookup table with proper locations
                {   uint16_t writeAddr = sizeof(lookupTable);

                    for (int stringNum = 0; stringNum < HStr_Count; ++stringNum) {
                        String string = SFP((Helio_String)stringNum);
                        lookupTable[stringNum] = writeAddr;
                        writeAddr += string.length() + 1;
                    }
                }

                getLogger()->logMessage(F("Writing Strings"));
                String filename = String(String(F(SETUP_EXTDATA_SD_LIB_PREFIX)) + String(F("strings.")) + SFP(HStr_dat));
                getLogger()->logMessage(F("... to file: "), filename);

                createDirectoryFor(sd, filename);
                if (sd->exists(filename.c_str())) {
                    sd->remove(filename.c_str());
                }
                auto file = sd->open(filename.c_str(), FILE_WRITE);
                if (file) { // Strings data goes into a single file as binary
                    uint16_t bytesWritten = 0;

                    // Lookup table constructed first to avoid random seeking
                    bytesWritten += file.write((const uint8_t *)lookupTable, sizeof(lookupTable));

                    for (int stringNum = 0; stringNum < HStr_Count; ++stringNum) {
                        String string = SFP((Helio_String)stringNum);
                        bytesWritten += file.write((const uint8_t *)string.c_str(), string.length() + 1); // +1 to also write out null terminator
                    }

                    if (bytesWritten) {
                        getLogger()->logMessage(F("Wrote: "), String(bytesWritten), F(" bytes"));
                    } else {
                        getLogger()->logError(F("Failure writing to strings data file!"));
                    }

                    file.flush();
                    file.close();
                } else {
                    getLogger()->logError(F("Failure opening strings data file for writing!"));
                }

                yield();
            }

            getController()->endSDCard(sd);
        } else {
            getLogger()->logWarning(F("Could not find SD card device. Check that you have it set up properly."));
        }

        getLogger()->flush();
    }
    #endif

    #if SETUP_EXTDATA_EEPROM_ENABLE
    {   auto eeprom = getController()->getEEPROM();

        if (eeprom) {
            uint16_t stringsBegAddr = SETUP_EXTDATA_EEPROM_BEG_ADDR;
            uint16_t sysDataBegAddr = (uint16_t)-1;

            {   getLogger()->logMessage(F("=== Writing strings data to EEPROM ==="));

                // Similar to above, same deal with a lookup table.
                uint16_t writeAddr = stringsBegAddr + ((HStr_Count + 1) * sizeof(uint16_t));

                for (int stringNum = 0; stringNum < HStr_Count; ++stringNum) {
                    String string = SFP((Helio_String)stringNum);

                    getLogger()->logMessage(F("Writing String: #"), String(stringNum) + String(F(" \"")), string + String(F("\"")));
                    getLogger()->logMessage(F("... to byte offset: "), String(writeAddr), altAddressToString(writeAddr));

                    if(eeprom->updateBlockVerify(writeAddr, (const uint8_t *)string.c_str(), string.length() + 1) &&
                       eeprom->updateBlockVerify(stringsBegAddr + ((stringNum + 1) * sizeof(uint16_t)),
                                                 (const uint8_t *)&writeAddr, sizeof(uint16_t))) {
                        writeAddr += string.length() + 1;
                        getLogger()->logMessage(F("Wrote: "), String(string.length() + 1), F(" bytes"));
                    } else {
                        getLogger()->logError(F("Failure writing strings data to EEPROM!"));
                    }

                    yield();
                }

                sysDataBegAddr = stringsBegAddr;
                if (writeAddr > stringsBegAddr + ((HStr_Count + 1) * sizeof(uint16_t))) {
                    uint16_t totalBytesWritten = writeAddr - stringsBegAddr;

                    if (eeprom->updateBlockVerify(stringsBegAddr, (const uint8_t *)&totalBytesWritten, sizeof(uint16_t))) {
                        getLogger()->logMessage(F("Successfully wrote: "), String(totalBytesWritten), F(" bytes"));
                        sysDataBegAddr = writeAddr;
                    } else {
                        getLogger()->logError(F("Failure writing total strings data size to EEPROM!"));
                    }
                }
            }

            getLogger()->logMessage(F("Total EEPROM usage: "), String(sysDataBegAddr), F(" bytes"));
            getLogger()->logMessage(F("EEPROM capacity used: "), String(((float)sysDataBegAddr / eeprom->getDeviceSize()) * 100.0f) + String(F("% of ")), String(eeprom->getDeviceSize()) + String(F(" bytes")));
            getLogger()->logMessage(F("Use the following EEPROM setup defines in your sketch:"));
            Serial.print(F("#define SETUP_EEPROM_SYSDATA_ADDR       "));
            Serial.println(addressToString(sysDataBegAddr));
            Serial.print(F("#define SETUP_EEPROM_STRINGS_ADDR       "));
            Serial.println(addressToString(stringsBegAddr));
        } else {
            getLogger()->logWarning(F("Could not find EEPROM device. Check that you have it set up properly."));
        }

        getLogger()->flush();
    }
    #endif

    getLogger()->logMessage(F("Done!"));
}

void loop()
{ ; }

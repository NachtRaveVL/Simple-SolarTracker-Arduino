
/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Streams
*/

#ifndef HelioStreams_H
#define HelioStreams_H

class HelioEEPROMStream;
class HelioPROGMEMStream;

#include "Helioduino.h"

#ifdef ARDUINO_ARCH_SAM // Stream doesn't have availableForWrite
#define HELIO_STREAM_AVAIL4WRT_OVERRIDE
#else
#define HELIO_STREAM_AVAIL4WRT_OVERRIDE override
#endif

// EEPROM Stream
// Stream class for working with I2C_EEPROM data.
class HelioEEPROMStream : public Stream {
public:
    HelioEEPROMStream();
    HelioEEPROMStream(uint16_t dataAddress, size_t dataSize);

    virtual int available() override;
    virtual int read() override;
    size_t readBytes(char *buffer, size_t length);
    virtual int peek() override;
    virtual void flush() override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
    virtual size_t write(uint8_t data) override;
    virtual int availableForWrite() HELIO_STREAM_AVAIL4WRT_OVERRIDE;

protected:
    I2C_eeprom *_eeprom;
    uint16_t _readAddress, _writeAddress, _endAddress;
};


// PROGMEM Stream
// Stream class for working with PROGMEM data.
class HelioPROGMEMStream : public Stream {
public:
    HelioPROGMEMStream();
    HelioPROGMEMStream(uintptr_t dataAddress);
    HelioPROGMEMStream(uintptr_t dataAddress, size_t dataSize);

    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    virtual void flush() override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
    virtual size_t write(uint8_t data) override;

protected:
    uintptr_t _readAddress, _writeAddress, _endAddress;
};

#ifdef HELIO_USE_WIFI_STORAGE

class HelioWiFiStorageFileStream : public Stream {
public:
    HelioWiFiStorageFileStream(WiFiStorageFile file, uintptr_t seekPos = 0);
    virtual ~HelioWiFiStorageFileStream();

    virtual int available() override;
    virtual int read() override;
    size_t readBytes(char *buffer, size_t length);
    virtual int peek() override;
    virtual void flush() override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
    virtual size_t write(uint8_t data) override;
    virtual int availableForWrite() HELIO_STREAM_AVAIL4WRT_OVERRIDE;

protected:
    enum WiFiStorageFileDirection : signed char { ReadBuffer, WriteBuffer, None = -1 };

    WiFiStorageFile _file;
    uint8_t _buffer[HELIO_WIFISTREAM_BUFFER_SIZE];
    size_t _bufferOffset;
    uintptr_t _bufferFileOffset;
    WiFiStorageFileDirection _bufferDirection;
    uintptr_t _readOffset, _writeOffset, _endOffset;

    void prepareReadBuffer();
    void prepareWriteBuffer();
};

#endif

#endif // /ifndef HelioStreams_H

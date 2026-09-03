/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Streams
*/

#include "Helioduino.h"
#include <limits.h>

HelioEEPROMStream::HelioEEPROMStream()
    : Stream(), _eeprom(nullptr), _readAddress(0), _writeAddress(0), _endAddress(0)
{
    if (getController() && (_eeprom = getController()->getEEPROM())) {
        _endAddress = _eeprom->getDeviceSize();
    }
    HELIO_HARD_ASSERT(_eeprom, SFP(HStr_Err_UnsupportedOperation));
}

HelioEEPROMStream::HelioEEPROMStream(uint16_t dataAddress, size_t dataSize)
      : Stream(), _eeprom(nullptr), _readAddress(dataAddress), _writeAddress(dataAddress), _endAddress(dataAddress + dataSize)
{
    if (getController()) {
        _eeprom = getController()->getEEPROM();
    }
    HELIO_HARD_ASSERT(_eeprom, SFP(HStr_Err_UnsupportedOperation));
}

int HelioEEPROMStream::available()
{
    if (!_eeprom || _readAddress >= _endAddress) { return 0; }
    uint32_t available = _endAddress - _readAddress;
    return available > INT_MAX ? INT_MAX : (int)available;
}

int HelioEEPROMStream::read()
{
    if (!_eeprom || _readAddress >= _endAddress) { return -1; }
    return (int)_eeprom->readByte(_readAddress++);
}

size_t HelioEEPROMStream::readBytes(char *buffer, size_t length)
{
    if (!_eeprom || !buffer || !length || _readAddress >= _endAddress) { return 0; }
    uint32_t remaining = _endAddress - _readAddress;
    if (length > remaining) { length = remaining; }

    size_t bytesRead = 0;
    while (bytesRead < length) {
        size_t chunkSize = length - bytesRead;
        if (chunkSize > UINT16_MAX) { chunkSize = UINT16_MAX; }
        size_t chunkRead = _eeprom->readBlock((uint16_t)_readAddress, (uint8_t *)buffer + bytesRead, (uint16_t)chunkSize);
        _readAddress += chunkRead;
        bytesRead += chunkRead;
        if (chunkRead != chunkSize) { break; }
    }
    return bytesRead;
}

int HelioEEPROMStream::peek()
{
    if (!_eeprom || _readAddress >= _endAddress) { return -1; }
    return (int)_eeprom->readByte(_readAddress);
}

void HelioEEPROMStream::flush()
{
    //_eeprom->commit();
}

size_t HelioEEPROMStream::write(const uint8_t *buffer, size_t size)
{
    if (!_eeprom || !buffer || !size || _writeAddress >= _endAddress) { return 0; }
    uint32_t remaining = _endAddress - _writeAddress;
    if (size > remaining) { size = remaining; }

    size_t bytesWritten = 0;
    while (bytesWritten < size) {
        size_t chunkSize = size - bytesWritten;
        if (chunkSize > UINT16_MAX) { chunkSize = UINT16_MAX; }
        if (!_eeprom->updateBlockVerify((uint16_t)_writeAddress, buffer + bytesWritten, (uint16_t)chunkSize)) {
            HELIO_SOFT_ASSERT(false, SFP(HStr_Err_OperationFailure));
            break;
        }
        _writeAddress += chunkSize;
        bytesWritten += chunkSize;
    }
    return bytesWritten;
}

size_t HelioEEPROMStream::write(uint8_t data)
{
    if (!_eeprom || _writeAddress >= _endAddress) { return 0; }
    if (_eeprom->updateByteVerify(_writeAddress, data)) {
        _writeAddress += 1;
        return 1;
    } else {
        HELIO_SOFT_ASSERT(false, SFP(HStr_Err_OperationFailure));
        return 0;
    }
}

int HelioEEPROMStream::availableForWrite()
{
    if (!_eeprom || _writeAddress >= _endAddress) { return 0; }
    uint32_t available = _endAddress - _writeAddress;
    return available > INT_MAX ? INT_MAX : (int)available;
}


HelioPROGMEMStream::HelioPROGMEMStream()
    : Stream(), _readAddress(0), _writeAddress(0), _endAddress(UINTPTR_MAX)
{ ; }

HelioPROGMEMStream::HelioPROGMEMStream(uintptr_t dataAddress)
    : Stream(), _readAddress(dataAddress), _writeAddress(dataAddress), _endAddress(dataAddress + strlen_P((const char *)dataAddress))
{ ; }

HelioPROGMEMStream::HelioPROGMEMStream(uintptr_t dataAddress, size_t dataSize)
    : Stream(), _readAddress(dataAddress), _writeAddress(dataAddress), _endAddress(dataAddress + dataSize)
{ ; }

int HelioPROGMEMStream::available()
{
    return _endAddress - _readAddress;
}

int HelioPROGMEMStream::read()
{
    if (_readAddress >= _endAddress) { return -1; }
    #ifdef ESP8266
        return pgm_read_byte((const void *)(_readAddress++));
    #else
        return pgm_read_byte(_readAddress++);
    #endif
}

int HelioPROGMEMStream::peek()
{
    if (_readAddress >= _endAddress) { return -1; }
    #ifdef ESP8266
        return pgm_read_byte((const void *)(_readAddress));
    #else
        return pgm_read_byte(_readAddress);
    #endif
}

void HelioPROGMEMStream::flush()
{ ; }

size_t HelioPROGMEMStream::write(const uint8_t *buffer, size_t size)
{
    (void)buffer;
    (void)size;
    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_OperationFailure));
    return 0;
}

size_t HelioPROGMEMStream::write(uint8_t data)
{
    (void)data;
    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_OperationFailure));
    return 0;
}


#ifdef HELIO_USE_WIFI_STORAGE

HelioWiFiStorageFileStream::HelioWiFiStorageFileStream(WiFiStorageFile file, uint32_t seekPos)
    : Stream(), _file(file), _buffer{0}, _bufferOffset(0), _bufferFileOffset(-1), _bufferDirection(None), _readOffset(0), _writeOffset(0), _endOffset(0)
{
    _endOffset = _file.size();
    _readOffset = _writeOffset = seekPos;
}

HelioWiFiStorageFileStream::~HelioWiFiStorageFileStream()
{
    flush();
}

int HelioWiFiStorageFileStream::available()
{
    if (!_file || _readOffset >= _endOffset) { return 0; }
    uint32_t available = _endOffset - _readOffset;
    return available > INT_MAX ? INT_MAX : (int)available;
}

int HelioWiFiStorageFileStream::read()
{
    if (!_file || _readOffset >= _endOffset) { return -1; }
    prepareReadBuffer();
    _readOffset++;
    return _buffer[_bufferOffset++];
}

size_t HelioWiFiStorageFileStream::readBytes(char *buffer, size_t length)
{
    if (!_file || !buffer || !length || _readOffset >= _endOffset) { return 0; }
    size_t bytesRead = 0;
    while (length && _readOffset < _endOffset) {
        prepareReadBuffer();
        size_t howMany = min(length, _endOffset - _readOffset);
        howMany = min(howMany, HELIO_WIFISTREAM_BUFFER_SIZE - _bufferOffset);
        memcpy(buffer, &_buffer[_bufferOffset], howMany);
        _readOffset += howMany;
        _bufferOffset += howMany;
        buffer += howMany;
        length -= howMany;
        bytesRead += howMany;
    }
    return bytesRead;
}

int HelioWiFiStorageFileStream::peek()
{
    if (!_file || _readOffset >= _endOffset) { return -1; }
    prepareReadBuffer();
    return _buffer[_bufferOffset];
}

void HelioWiFiStorageFileStream::flush()
{
    if (_bufferDirection == WriteBuffer && _bufferOffset > 0) {
        _file.seek(_bufferFileOffset);
        _file.write((const void*)_buffer, _bufferOffset); _bufferOffset = 0;
        _bufferFileOffset = _writeOffset;
    }
}

size_t HelioWiFiStorageFileStream::write(const uint8_t *buffer, size_t size)
{
    if (!buffer || !size) { return 0; }
    size_t bytesWritten = 0;
    while (size) {
        prepareWriteBuffer();
        size_t howMany = min(size, HELIO_WIFISTREAM_BUFFER_SIZE - _bufferOffset);
        memcpy(&_buffer[_bufferOffset], buffer, howMany);
        _writeOffset += howMany;
        _bufferOffset += howMany;
        buffer += howMany;
        size -= howMany;
        bytesWritten += howMany;
    }
    if (_writeOffset > _endOffset) { _endOffset = _writeOffset; }
    return bytesWritten;
}

size_t HelioWiFiStorageFileStream::write(uint8_t data)
{
    prepareWriteBuffer();
    _buffer[_bufferOffset++] = data;
    _writeOffset++;
    if (_writeOffset > _endOffset) { _endOffset = _writeOffset; }
    return 1;
}

int HelioWiFiStorageFileStream::availableForWrite() 
{
    return _bufferDirection == WriteBuffer ? HELIO_WIFISTREAM_BUFFER_SIZE - _bufferOffset : HELIO_WIFISTREAM_BUFFER_SIZE;
}

void HelioWiFiStorageFileStream::prepareReadBuffer()
{
    if (_bufferDirection != ReadBuffer || _bufferFileOffset == UINT32_MAX || _readOffset < _bufferFileOffset || _readOffset >= _bufferFileOffset + HELIO_WIFISTREAM_BUFFER_SIZE) {
        if (_bufferDirection == WriteBuffer && _bufferOffset > 0) {
            _file.seek(_bufferFileOffset);
            _file.write((const void*)_buffer, _bufferOffset); //_bufferOffset = 0;
        }
        _bufferDirection = ReadBuffer;
        _bufferFileOffset = _readOffset;
        _bufferOffset = 0;

        _file.seek(_bufferFileOffset);
        _file.read((void *)_buffer, HELIO_WIFISTREAM_BUFFER_SIZE);
    }
}

void HelioWiFiStorageFileStream::prepareWriteBuffer()
{
    if (_bufferDirection != WriteBuffer || _bufferFileOffset == UINT32_MAX || _writeOffset < _bufferFileOffset || _writeOffset >= _bufferFileOffset + HELIO_WIFISTREAM_BUFFER_SIZE) {
        if (_bufferDirection == WriteBuffer && _bufferOffset > 0) {
            _file.seek(_bufferFileOffset);
            _file.write((const void*)_buffer, _bufferOffset); //_bufferOffset = 0;
        }
        _bufferDirection = WriteBuffer;
        _bufferFileOffset = _writeOffset;
        _bufferOffset = 0;
    }
}

#endif

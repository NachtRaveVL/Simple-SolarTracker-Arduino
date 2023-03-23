/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Streams
*/

#include "Helioduino.h"

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
    return _eeprom ? ((int)_endAddress - _readAddress) : 0;
}

int HelioEEPROMStream::read()
{
    if (!_eeprom || _readAddress >= _endAddress) { return -1; }
    return (int)_eeprom->readByte(_readAddress++);
}

size_t HelioEEPROMStream::readBytes(char *buffer, size_t length)
{
    if (!_eeprom || _readAddress >= _endAddress) { return -1; }
    size_t retVal = _eeprom->readBlock(_readAddress, (uint8_t *)buffer, length);
    _readAddress += retVal;
    return retVal;
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
    if (!_eeprom || _writeAddress >= _endAddress) { return 0; }
    size_t remaining = _endAddress - _writeAddress;
    if (size > remaining) { size = remaining; }
    if (_eeprom->updateBlockVerify(_writeAddress, buffer, size)) {
        _writeAddress += size;
        return size;
    } else {
        HELIO_SOFT_ASSERT(false, SFP(HStr_Err_OperationFailure));
        return 0;
    }
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
    return _eeprom ? ((int)_endAddress - _writeAddress) : 0;
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
    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_OperationFailure));
    return 0;
}

size_t HelioPROGMEMStream::write(uint8_t data)
{
    HELIO_SOFT_ASSERT(false, SFP(HStr_Err_OperationFailure));
    return 0;
}


#ifdef HELIO_USE_WIFI_STORAGE

HelioWiFiStorageFileStream::HelioWiFiStorageFileStream(WiFiStorageFile file, uintptr_t seekPos)
    : Stream(), _file(file), _buffer{0}, _bufferOffset(0), _bufferFileOffset(-1), _bufferDirection(None), _readOffset(0), _writeOffset(0), _endOffset(0)
{
    if (_file) {
        _endOffset = _file.size();
        _readOffset = _writeOffset = seekPos;
    }
}

HelioWiFiStorageFileStream::~HelioWiFiStorageFileStream()
{
    if (_file) {
        if (_bufferDirection == WriteBuffer && _bufferOffset > 0) {
            _file.seek(_bufferFileOffset);
            _file.write((const void*)_buffer, _bufferOffset); _bufferOffset = 0;
        }
    }
}

int HelioWiFiStorageFileStream::available()
{
    return _file ? _endOffset - _readOffset : 0;
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
    if (!_file || _readOffset >= _endOffset) { return -1; }
    while (length && _readOffset < _endOffset) {
        prepareReadBuffer();
        size_t howMany = min(length, _endOffset - _readOffset);
        howMany = min(howMany, HELIO_WIFISTREAM_BUFFER_SIZE - _bufferOffset);
        memcpy(buffer, &_buffer[_bufferOffset], howMany);
        _readOffset += howMany;
        _bufferOffset += howMany;
        buffer += howMany;
        length -= howMany;
    }
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
    }
}

size_t HelioWiFiStorageFileStream::write(const uint8_t *buffer, size_t size)
{
    if (!_file || _writeOffset >= _endOffset) { return -1; }
    while (size) {
        prepareWriteBuffer();
        size_t howMany = min(size, HELIO_WIFISTREAM_BUFFER_SIZE - _bufferOffset);
        memcpy(&_buffer[_bufferOffset], buffer, howMany);
        _writeOffset += howMany;
        _bufferOffset += howMany;
        buffer += howMany;
        size -= howMany;
    }
}

size_t HelioWiFiStorageFileStream::write(uint8_t data)
{
    if (!_file || _writeOffset >= _endOffset) { return -1; }
    prepareWriteBuffer();
    _buffer[_bufferOffset++] = data;
    _writeOffset++;
    return 1;
}

int HelioWiFiStorageFileStream::availableForWrite() 
{
    return _file ? _endOffset - _writeOffset : 0;
}

void HelioWiFiStorageFileStream::prepareReadBuffer()
{
    if (_bufferDirection != ReadBuffer || _bufferFileOffset == -1 || _readOffset < _bufferFileOffset || _readOffset >= _bufferFileOffset + HELIO_WIFISTREAM_BUFFER_SIZE) {
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
    if (_bufferDirection != WriteBuffer || _bufferFileOffset == -1 || _writeOffset < _bufferFileOffset || _writeOffset >= _bufferFileOffset + HELIO_WIFISTREAM_BUFFER_SIZE) {
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

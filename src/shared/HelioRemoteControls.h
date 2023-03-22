/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Remote Controls
*/

#include <Helioduino.h>
#ifdef HELIO_USE_GUI
#ifndef HelioRemoteControls_H
#define HelioRemoteControls_H

class HelioRemoteControl;
class HelioRemoteSerialControl;
class HelioRemoteSimhubControl;
#ifdef HELIO_USE_WIFI
class HelioRemoteWiFiControl;
#endif
#ifdef HELIO_USE_ETHERNET
class HelioRemoteEthernetControl;
#endif

#include "HelioduinoUI.h"

// Remote Control Base
// Base remote control class.
class HelioRemoteControl {
public:
    virtual ~HelioRemoteControl() = default;

    virtual BaseRemoteServerConnection *getConnection() = 0;
};


// Serial UART Remote Control
// Manages remote control over serial UART.
class HelioRemoteSerialControl : public HelioRemoteControl {
public:
    HelioRemoteSerialControl(UARTDeviceSetup serialSetup);
    virtual ~HelioRemoteSerialControl() = default;

    virtual BaseRemoteServerConnection *getConnection() override;

protected:
    SerialTagValueTransport _serialTransport;
    NoInitialisationNeeded _serialInitializer;
    TagValueRemoteServerConnection _serialConnection;
};


// Simhub Connector Remote Control
// Manages remote control over simhub connector.
class HelioRemoteSimhubControl : public HelioRemoteControl {
public:
    HelioRemoteSimhubControl(UARTDeviceSetup serialSetup, menuid_t statusMenuId);
    virtual ~HelioRemoteSimhubControl() = default;

    virtual BaseRemoteServerConnection *getConnection() override;

protected:
    SimHubRemoteConnection _simhubConnection;
};


#ifdef HELIO_USE_WIFI
// WiFi Remote Control
// Manages remote control over a WiFi connection.
class HelioRemoteWiFiControl : public HelioRemoteControl {
public:
    HelioRemoteWiFiControl(uint16_t listeningPort = HELIO_UI_REMOTESERVER_PORT);
    virtual ~HelioRemoteWiFiControl() = default;

    virtual BaseRemoteServerConnection *getConnection() override;

    inline WiFiServer &getRCServer() { return _rcServer; }

protected:
    WiFiServer _rcServer;
    WiFiInitialisation _netInitialisation;
    WiFiTagValTransport _netTransport;
    TagValueRemoteServerConnection _netConnection;
};
#endif


#ifdef HELIO_USE_ETHERNET
// Ethernet Remote Control
// Manages remote control over an Ethernet connection.
class HelioRemoteEthernetControl : public HelioRemoteControl {
public:
    HelioRemoteEthernetControl(uint16_t listeningPort = HELIO_UI_REMOTESERVER_PORT);
    virtual ~HelioRemoteEthernetControl() = default;

    virtual BaseRemoteServerConnection *getConnection() override;

    inline EthernetServer &getRCServer() { return _rcServer; }

protected:
    EthernetServer _rcServer;
    EthernetInitialisation _netInitialisation;
    EthernetTagValTransport _netTransport;
    TagValueRemoteServerConnection _netConnection;
};
#endif

#endif // /ifndef HelioRemoteControls_H
#endif

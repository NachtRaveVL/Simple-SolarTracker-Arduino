/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Remote Controls
*/

#include "HelioduinoUI.h"
#ifdef HELIO_USE_GUI

HelioRemoteSerialControl::HelioRemoteSerialControl(UARTDeviceSetup serialSetup)
    : _serialTransport(serialSetup.serial), _serialInitializer(), _serialConnection(_serialTransport, _serialInitializer)
{ ; }

BaseRemoteServerConnection *HelioRemoteSerialControl::getConnection()
{
    return &_serialConnection;
}


HelioRemoteSimhubControl::HelioRemoteSimhubControl(UARTDeviceSetup serialSetup, menuid_t statusMenuId)
    : _simhubConnection(serialSetup.serial, statusMenuId)
{ ; }

BaseRemoteServerConnection *HelioRemoteSimhubControl::getConnection()
{
    return &_simhubConnection;
}


#ifdef HELIO_USE_WIFI

HelioRemoteWiFiControl::HelioRemoteWiFiControl(uint16_t listeningPort)
    : _rcServer(listeningPort), _netInitialisation(&_rcServer), _netTransport(), _netConnection(_netTransport, _netInitialisation)
{ ; }

BaseRemoteServerConnection *HelioRemoteWiFiControl::getConnection()
{
    return &_netConnection;
}

#endif // /ifdef HELIO_USE_WIFI


#ifdef HELIO_USE_ETHERNET

HelioRemoteEthernetControl::HelioRemoteEthernetControl(uint16_t listeningPort)
    : _rcServer(listeningPort), _netInitialisation(&_rcServer), _netTransport(), _netConnection(_netTransport, _netInitialisation)
{ ; }

BaseRemoteServerConnection *HelioRemoteEthernetControl::getConnection()
{
    return &_netConnection;
}

#endif // /ifdef HELIO_USE_ETHERNET

#endif

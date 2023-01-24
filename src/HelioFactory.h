/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Factory
*/

#ifndef HelioFactory_H
#define HelioFactory_H

class HelioFactory;

#include "Helioduino.h"

class HelioFactory {
public:
    // Object Factory.

    // Convenience builders for common actuators (shared, nullptr return -> failure).

    // Adds a new panel heater relay to the system using the given parameters.
    // Panel heaters can keep panels clear during colder months or when ice is detected.
    SharedPtr<HelioRelayActuator> addPanelHeaterRelay(pintype_t outputPin);                 // Digital output pin this actuator sits on

    // Convenience builders for common sensors (shared, nullptr return -> failure).

    // Adds a new analog temperature sensor to the system using the given parameters.
    // Temperature sensors can be used to ensure proper temperature conditions.
    SharedPtr<HelioAnalogSensor> addAnalogTemperatureSensor(pintype_t inputPin,             // Analog input pin this sensor sits on
                                                            uint8_t inputBitRes = ADC_RESOLUTION); // ADC input bit resolution to use
    // Adds a new analog power usage meter to the system using the given parameters.
    // Power usage meters can be used to determine and manage the energy demands of a power rail.
    SharedPtr<HelioAnalogSensor> addPowerUsageMeter(pintype_t inputPin,                     // Analog input pin this sensor sits on
                                                    bool isWattageBased,                    // If power meter measures wattage (true) or amperage (false)
                                                    uint8_t inputBitRes = ADC_RESOLUTION);  // ADC input bit resolution to use

    // Convenience builders for common panels (shared, nullptr return -> failure).

    // Adds a new simple fluid panel to the system using the given parameters.
    // Fluid panels are basically just buckets of some liquid solution with a known or measurable volume.
    //SharedPtr<HelioSinglePanel> addSinglePanel(Helio_PanelType panelType);                  // Panel type

    // Convenience builders for common power rails (shared, nullptr return -> failure).

    // Adds a new simple power rail to the system using the given parameters.
    // Simple power rail uses a max active at once counting strategy to manage energy consumption.
    SharedPtr<HelioSimpleRail> addSimplePowerRail(Helio_RailType railType,                  // Rail type
                                                  int maxActiveAtOnce = 2);                 // Maximum active devices

    // Adds a new regulated power rail to the system using the given parameters.
    // Regulated power rails can use a power meter to measure energy consumption to limit overdraw.
    SharedPtr<HelioRegulatedRail> addRegulatedPowerRail(Helio_RailType railType,            // Rail type
                                                        float maxPower);                    // Maximum allowed power
};

#endif // /ifndef HelioFactory_H

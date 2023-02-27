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

    // Adds a new relay-based panel heater to the system using the given parameters.
    // Panel heaters can keep panels clear during colder months or when ice is detected.
    SharedPtr<HelioRelayActuator> addPanelHeaterRelay(pintype_t outputPin);                 // Digital output pin this actuator sits on

    // Adds a new relay-based panel sprayer to the system using the given parameters.
    // Panel sprayers can activate on schedule to keep panels clear of debris and dirt.
    SharedPtr<HelioRelayActuator> addPanelSprayerRelay(pintype_t outputPin);                // Digital output pin this actuator sits on

    // Adds a new PWM-based panel-axis-driving positional servo to the system using the given parameters.
    // PWM positional servos provide simple angular movement control over panels.
    SharedPtr<HelioVariableActuator> addPositionServo(pintype_t outputPin,                  // Analog output pin this actuator sits on
                                                      float minDegrees = -90.0f,            // Minimum angular degrees
                                                      float maxDegrees = 90.0,              // Maximum angular degrees
                                                      uint8_t outputBitRes = DAC_RESOLUTION // PWM output bit resolution to use
#ifdef ESP32
                                                      , uint8_t pwmChannel = 1              // PWM output channel (0 reserved for buzzer)
#endif
#ifdef ESP_PLATFORM
                                                      , float pwmFrequency = 1000           // PWM output frequency
#endif
    );

// TODO: #9 in Helio.
//     // Adds a new PWM-based panel-axis-driving continuous servo to the system using the given parameters.
//     // PWM continuous servos provide constant angular movement not just limited to two angles.
//     SharedPtr<HelioVariableMotorActuator> addContinuousServo(pintype_t outputPin,           // Analog output pin this actuator sits on
//                                                              float maxTravel = __FLT_MAX__, // Maximum travel range
//                                                              float minTravel = -__FLT_MAX__, // Minimum travel range
//                                                              uint8_t outputBitRes = DAC_RESOLUTION // PWM output bit resolution to use
// #ifdef ESP32
//                                                              , uint8_t pwmChannel = 1    // PWM output channel (0 reserved for buzzer)
// #endif
// #ifdef ESP_PLATFORM
//                                                              , float pwmFrequency = 1000 // PWM output frequency
// #endif
//     );

    // Adds a new relay-based panel-axis-driving linear actuator to the system using the given parameters.
    // Linear actuators allow angular movement of panels that are too large for servos by instead using hydraulics and lever action.
    SharedPtr<HelioRelayMotorActuator> addLinearActuatorRelay(pintype_t outputPinA,         // Digital output pin A (forward) this actuator sits on
                                                              pintype_t outputPinB,         // Digital output pin B (reverse) this actuator sits on
                                                              float maxPosition,            // Maximum stroke distance / position
                                                              float minPosition = 0.0f);    // Minimum stroke distance / position

// TODO: #9 in Helio.
//     // Adds a new analog PWM-based panel-axis-driving linear actuator to the system using the given parameters.
//     // PWM linear actuators allow a graduated adaptive speed control, typically via H-bridge.
//     SharedPtr<HelioVariableMotorActuator> addAnalogLinearActuator(pintype_t outputPinA,     // Analog output pin A (forward) this actuator sits on
//                                                                   pintype_t outputPinB,     // Analog output pin B (reverse) this actuator sits on
//                                                                   float maxStroke,          // Maximum stroke distance
//                                                                   float minStroke = 0.0f,   // Minimum stroke distance
//                                                                   uint8_t outputBitRes = DAC_RESOLUTION // PWM output bit resolution to use
// #ifdef ESP32
//                                                                   , uint8_t pwmChannel = 1  // PWM output channel (0 reserved for buzzer)
// #endif
// #ifdef ESP_PLATFORM
//                                                                   , float pwmFrequency = 1000 // PWM output frequency
// #endif
//     );

    // Convenience builders for common sensors (shared, nullptr return -> failure).

    // Adds a new binary endstop indicator to the system using the given parameters.
    // Endstops can provide triggering that limit actuator travel to within an enclosed track.
    SharedPtr<HelioBinarySensor> addEndstopIndicator(pintype_t inputPin,                    // Digital input pin this sensor sits on
                                                     float position);                       // Position representation when triggered

    // Adds a new binary ice indicator to the system using the given parameters.
    // Ice detection can drive panel heaters that keep ice and snow off panels during cold months.
    SharedPtr<HelioBinarySensor> addIceIndicator(pintype_t inputPin);                       // Digital input pin this sensor sits on

    // Adds a new analog light intensity sensor/LDR to the system using the given parameters.
    // LDRs can be used in a simple balancing panel to orient towards the strongest light source.
    SharedPtr<HelioAnalogSensor> addLightIntensitySensor(pintype_t inputPin);               // Analog input pin this sensor sits on

    // Adds a new analog power production meter to the system using the given parameters.
    // Power production meters can be used to determine the amount of energy generation.
    SharedPtr<HelioAnalogSensor> addPowerProductionMeter(pintype_t inputPin,                // Analog input pin this sensor sits on
                                                         bool isWattageBased = true,        // If power meter measures wattage (true) or amperage (false)
                                                         uint8_t inputBitRes = ADC_RESOLUTION); // ADC input bit resolution to use

    // Adds a new analog power usage meter to the system using the given parameters.
    // Power usage meters can be used to determine and manage the energy demands of a power rail.
    SharedPtr<HelioAnalogSensor> addPowerUsageLevelMeter(pintype_t inputPin,                // Analog input pin this sensor sits on
                                                         bool isWattageBased = true,        // If power meter measures wattage (true) or amperage (false)
                                                         uint8_t inputBitRes = ADC_RESOLUTION); // ADC input bit resolution to use

    // Adds a new analog stroke position/distance sensor to the system using the given parameters.
    // Linear actuators with feedback can utilize potentiometer-based position sensing for accurate travel.
    SharedPtr<HelioAnalogSensor> addAnalogPositionSensor(pintype_t inputPin,                // Analog input pin this sensor sits on
                                                         float maxDistance,                 // Maximum distance (max extended position)
                                                         float minDistance = 0,             // Minimum distance (min retracted position)
                                                         float maxReading = 1.0f,           // Maximum normalized analogRead() value [0,1]
                                                         float minReading = 0.0f,           // Minimum normalized analogRead() value [0,1]
                                                         uint8_t inputBitRes = ADC_RESOLUTION); // ADC input bit resolution to use

    // Adds a new analog tilt/lean angle sensor to the system using the given parameters.
    // Tilt angle sensors measure resistance using natural gravity, but can only measure straight-up to fully-tilted elevations.
    SharedPtr<HelioAnalogSensor> addAnalogTiltAngleSensor(pintype_t inputPin,               // Analog input pin this sensor sits on
                                                          float maxElevation = 90,          // Maximum elevation (max vertical angle)
                                                          float minElevation = 0,           // Minimum elevation (min vertical angle)
                                                          float maxReading = 1.0f,          // Maximum normalized analogRead() value [0,1]
                                                          float minReading = 0.0f,          // Minimum normalized analogRead() value [0,1]
                                                          uint8_t inputBitRes = ADC_RESOLUTION); // ADC input bit resolution to use

    // Adds a new analog temperature sensor to the system using the given parameters.
    // Temperature sensors can be used to ensure proper temperature conditions are being met.
    SharedPtr<HelioAnalogSensor> addAnalogTemperatureSensor(pintype_t inputPin,             // Analog input pin this sensor sits on
                                                            uint8_t inputBitRes = ADC_RESOLUTION); // ADC input bit resolution to use

    // Adds a new analog wind-speed sensor to the system using the given parameters.
    // Wind-speed sensors can be used to ensure proper wind conditions are being met.
    SharedPtr<HelioAnalogSensor> addAnalogWindSpeedSensor(pintype_t inputPin,               // Analog input pin this sensor sits on
                                                          uint8_t inputBitRes = ADC_RESOLUTION); // ADC input bit resolution to use

    // Adds a new digital DHT* OneWire temperature & humidity sensor to the system using the given parameters.
    // Uses the DHT library. A very common digital sensor, included in most Arduino starter kits.
    SharedPtr<HelioDHTTempHumiditySensor> addDHTTempHumiditySensor(pintype_t inputPin,      // OneWire digital input pin this sensor sits on
                                                                   Helio_DHTType dhtType = Helio_DHTType_DHT12); // DHT sensor type

    // Convenience builders for common panels (shared, nullptr return -> failure).

    // Adds a new LDR-based balancing panel to the system using the given parameters.
    // Balancing panels equalize two opposing photoresistors to drive their orientation.
    SharedPtr<HelioBalancingPanel> addLDRBalancingPanel(Helio_PanelType panelType,          // Panel type (mounting configuration)
                                                        float homePosition[2] = {0});       // Home position (azi,ele or RA,dec)

    // Adds a new solar-tracking smart panel to the system using the given parameters.
    // Tracking panels recalculate the sun's position across the day to drive their orientation.
    SharedPtr<HelioTrackingPanel> addSolarTrackingPanel(Helio_PanelType panelType,          // Panel type (mounting configuration)
                                                        float axisOffset[2] = {0},          // Axis calibration offsets (azi,ele or RA,dec)
                                                        float homePosition[2] = {0});       // Home position (azi,ele or RA,dec)

    // Adds a new reflecting panel to the system using the given parameters.
    // Reflecting "panels" are mirrors that reflect the sun's light towards a preset direction.
    SharedPtr<HelioReflectingPanel> addSolarReflectingPanel(Helio_PanelType panelType,           // Panel type (mounting configuration)
                                                            float reflectTowards[2] = {0},       // Direction to reflect towards (azi,ele or RA,dec)
                                                            float axisOffset[2] = {0},           // Axis calibration offsets (azi,ele or RA,dec)
                                                            float homePosition[2] = {0});        // Home position (azi,ele or RA,dec)

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

#include "Helioduino.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void testSystemData()
{
    HelioSystemData data;
    strncpy(data.systemName, "Backyard Solar Tracker", sizeof(data.systemName) - 1);
    data.systemName[sizeof(data.systemName) - 1] = '\0';
    data.systemMode = Helio_SystemMode_Tracking;
    data.measureMode = Helio_MeasurementMode_Imperial;
    data.timeZoneOffset = -7.0f;
    data.pollingInterval = 500;
    data.latitude = 49.2827;
    data.longitude = -123.1207;
    data.altitude = 70.0;
    data.scheduler.cleaningIntervalDays = 10;
    data.scheduler.preDawnCleaningMins = 2;
    data.scheduler.preDawnHeatingMins = 20;
    data.scheduler.reportInterval = 3600;
    data.logger.logLevel = Helio_LogLevel_Warnings;

    StaticJsonDocument<2048> doc;
    JsonObject object = doc.to<JsonObject>();
    data.toJSONObject(object);

    HelioSystemData decoded;
    JsonObjectConst objectConst = doc.as<JsonObjectConst>();
    decoded.fromJSONObject(objectConst);
    assert(decoded.isSystemData());
    assert(strcmp(decoded.systemName, data.systemName) == 0);
    assert(decoded.systemMode == data.systemMode);
    assert(decoded.measureMode == data.measureMode);
    assert(isFPEqual(decoded.timeZoneOffset, data.timeZoneOffset));
    assert(decoded.pollingInterval == data.pollingInterval);
    assert(isFPEqual(decoded.latitude, data.latitude));
    assert(isFPEqual(decoded.longitude, data.longitude));
    assert(isFPEqual(decoded.altitude, data.altitude));
    assert(decoded.scheduler.cleaningIntervalDays == data.scheduler.cleaningIntervalDays);
    assert(decoded.scheduler.preDawnCleaningMins == data.scheduler.preDawnCleaningMins);
    assert(decoded.scheduler.preDawnHeatingMins == data.scheduler.preDawnHeatingMins);
    assert(decoded.scheduler.reportInterval == data.scheduler.reportInterval);
    assert(decoded.logger.logLevel == data.logger.logLevel);

    HelioData *allocated = newDataFromJSONObject(objectConst);
    assert(allocated && allocated->isSystemData());
    delete allocated;
}

static void testCalibrationData()
{
    HelioCalibrationData data(HelioIdentity(Helio_SensorType_TemperatureHumidity, 1), Helio_UnitsType_Temperature_Celsius);
    data.setFromTwoPoints(0.1f, -10.0f, 0.9f, 30.0f);
    assert(isFPEqual(data.transform(0.5f), 10.0f));
    assert(isFPEqual(data.inverseTransform(10.0f), 0.5f));

    StaticJsonDocument<256> doc;
    JsonObject object = doc.to<JsonObject>();
    data.toJSONObject(object);
    JsonObjectConst objectConst = doc.as<JsonObjectConst>();

    HelioCalibrationData decoded;
    decoded.fromJSONObject(objectConst);
    assert(decoded.isCalibrationData());
    assert(strcmp(decoded.ownerName, data.ownerName) == 0);
    assert(decoded.calibrationUnits == data.calibrationUnits);
    assert(isFPEqual(decoded.multiplier, data.multiplier));
    assert(isFPEqual(decoded.offset, data.offset));
}

static void testActuatorData()
{
    HelioActuatorData data;
    data.id.object.idType = HelioIdentity::Actuator;
    data.id.object.objType = Helio_ActuatorType_PanelHeater;
    data.id.object.posIndex = 2;
    data.id.object.classType = HelioActuator::Relay;
    data.enableMode = Helio_EnableMode_InOrder;
    HelioDigitalPin(8, Helio_PinMode_Digital_Output_PushPull, false).saveToData(&data.outputPin);
    strncpy(data.railName, "AC110V #0", sizeof(data.railName) - 1);
    data.railName[sizeof(data.railName) - 1] = '\0';
    strncpy(data.panelName, "HorizontalPanel #0", sizeof(data.panelName) - 1);
    data.panelName[sizeof(data.panelName) - 1] = '\0';

    StaticJsonDocument<512> doc;
    JsonObject object = doc.to<JsonObject>();
    data.toJSONObject(object);
    JsonObjectConst objectConst = doc.as<JsonObjectConst>();

    HelioData *allocated = newDataFromJSONObject(objectConst);
    assert(allocated && allocated->isObjectData());
    HelioActuatorData *decoded = static_cast<HelioActuatorData *>(allocated);
    assert(decoded->id.object.idType == HelioIdentity::Actuator);
    assert(decoded->id.object.objType == Helio_ActuatorType_PanelHeater);
    assert(decoded->id.object.posIndex == 2);
    assert(decoded->id.object.classType == HelioActuator::Relay);
    assert(decoded->enableMode == Helio_EnableMode_InOrder);
    assert(decoded->outputPin.pin == 8);
    assert(!decoded->outputPin.dataAs.digitalPin.activeLow);
    assert(strcmp(decoded->railName, data.railName) == 0);
    assert(strcmp(decoded->panelName, data.panelName) == 0);
    delete allocated;
}

static void testBinarySensorData()
{
    HelioBinarySensorData data;
    data.id.object.idType = HelioIdentity::Sensor;
    data.id.object.objType = Helio_SensorType_TravelPosition;
    data.id.object.posIndex = 1;
    data.id.object.classType = HelioSensor::Binary;
    HelioDigitalPin(14, Helio_PinMode_Digital_Input_PullUp, true).saveToData(&data.inputPin);
    data.usingISR = false;
    data.stateStableTimeMs = 75;
    strncpy(data.panelName, "HorizontalPanel #0", sizeof(data.panelName) - 1);
    data.panelName[sizeof(data.panelName) - 1] = '\0';

    StaticJsonDocument<512> doc;
    JsonObject object = doc.to<JsonObject>();
    data.toJSONObject(object);
    JsonObjectConst objectConst = doc.as<JsonObjectConst>();

    HelioData *allocated = newDataFromJSONObject(objectConst);
    assert(allocated && allocated->isObjectData());
    HelioBinarySensorData *decoded = static_cast<HelioBinarySensorData *>(allocated);
    assert(decoded->id.object.classType == HelioSensor::Binary);
    assert(decoded->inputPin.pin == 14);
    assert(decoded->inputPin.dataAs.digitalPin.activeLow);
    assert(decoded->stateStableTimeMs == 75);
    assert(strcmp(decoded->panelName, data.panelName) == 0);

    HelioSensor *sensor = newSensorObjectFromData(decoded);
    assert(sensor && sensor->isBinaryClass() && !sensor->isDigitalClass());
    delete sensor;
    delete allocated;
}

static void testTriggerSubData()
{
    HelioTriggerSubData data;
    data.type = HelioTrigger::MeasureValue;
    strncpy(data.sensorName, "Temperature #1", sizeof(data.sensorName) - 1);
    data.sensorName[sizeof(data.sensorName) - 1] = '\0';
    data.measurementRow = 0;
    data.measurementUnits = Helio_UnitsType_Temperature_Celsius;
    data.detriggerTol = 0.5f;
    data.detriggerDelay = 1000;
    data.dataAs.measureValue.tolerance = 5.0f;
    data.dataAs.measureValue.triggerBelow = true;

    StaticJsonDocument<384> doc;
    JsonObject object = doc.to<JsonObject>();
    data.toJSONObject(object);
    JsonObjectConst objectConst = doc.as<JsonObjectConst>();

    HelioTriggerSubData decoded;
    decoded.fromJSONObject(objectConst);
    assert(decoded.type == data.type);
    assert(strcmp(decoded.sensorName, data.sensorName) == 0);
    assert(decoded.measurementRow == data.measurementRow);
    assert(decoded.measurementUnits == data.measurementUnits);
    assert(isFPEqual(decoded.detriggerTol, data.detriggerTol));
    assert(decoded.detriggerDelay == data.detriggerDelay);
    assert(isFPEqual(decoded.dataAs.measureValue.tolerance, data.dataAs.measureValue.tolerance));
    assert(decoded.dataAs.measureValue.triggerBelow == data.dataAs.measureValue.triggerBelow);
}

int main()
{
    testSystemData();
    testCalibrationData();
    testActuatorData();
    testBinarySensorData();
    testTriggerSubData();
    puts("PASS serialization");
    return 0;
}

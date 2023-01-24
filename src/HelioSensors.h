/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Sensors
*/

#ifndef HelioSensors_H
#define HelioSensors_H

class HelioSensor;
class HelioBinarySensor;
class HelioAnalogSensor;
class HelioDigitalSensor;
class HelioDHTTempHumiditySensor;
class HelioDSTemperatureSensor;

struct HelioSensorData;
struct HelioBinarySensorData;
struct HelioAnalogSensorData;
struct HelioDigitalSensorData;
struct HelioDHTTempHumiditySensorData;
struct HelioDSTemperatureSensorData;

#include "Helioduino.h"
#include "HelioDatas.h"

// Creates sensor object from passed sensor data (return ownership transfer - user code *must* delete returned object)
extern HelioSensor *newSensorObjectFromData(const HelioSensorData *dataIn);

// Returns default measurement units to use based on sensorType, optional row index, and measureMode (if undefined then uses active Helio instance's measurement mode, else default mode).
extern Helio_UnitsType defaultMeasureUnitsForSensorType(Helio_SensorType sensorType, uint8_t measurementRow = 0, Helio_MeasurementMode measureMode = Helio_MeasurementMode_Undefined);
// Returns default measurement category to use based on sensorType and optional row index (note: this may not accurately produce the correct category, e.g. an ultrasonic distance sensor being used for distance and not volume).
extern Helio_UnitsCategory defaultMeasureCategoryForSensorType(Helio_SensorType sensorType, uint8_t measurementRow = 0);


// Sensor Base
// This is the base class for all sensors, which defines how the sensor is identified,
// where it lives, and what it's attached to.
class HelioSensor : public HelioObject, public HelioSensorObjectInterface, public HelioPanelAttachmentInterface {
public:
    const enum : signed char { Binary, Analog, Digital, DHT1W, Unknown = -1 } classType; // Sensor class type (custom RTTI)
    inline bool isBinaryClass() const { return classType == Binary; }
    inline bool isAnalogClass() const { return classType == Analog; }
    inline bool isDigitalClass() const { return classType == Digital; }
    inline bool isDHTClass() const { return classType == DHT1W; }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioSensor(Helio_SensorType sensorType,
                Helio_PositionIndex sensorIndex,
                int classType = Unknown);
    HelioSensor(const HelioSensorData *dataIn);
    virtual ~HelioSensor();

    virtual void update() override;

    virtual bool takeMeasurement(bool force = false) = 0;
    virtual const HelioMeasurement *getLatestMeasurement() const = 0;
    virtual bool isTakingMeasurement() const override;
    virtual bool needsPolling(uint32_t allowance = 0) const override;

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t measurementRow = 0) = 0;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t measurementRow = 0) const = 0;

    virtual HelioAttachment &getParentPanel(bool resolve = true) override;

    void setUserCalibrationData(HelioCalibrationData *userCalibrationData);
    inline const HelioCalibrationData *getUserCalibrationData() const { return _calibrationData; }

    inline Helio_SensorType getSensorType() const { return _id.objTypeAs.sensorType; }
    inline Helio_PositionIndex getSensorIndex() const { return _id.posIndex; }

    Signal<const HelioMeasurement *, HELIO_SENSOR_MEASUREMENT_SLOTS> &getMeasurementSignal();

protected:
    bool _isTakingMeasure;                                  // Taking measurement flag
    HelioAttachment _panel;                                 // Panel attachment
    const HelioCalibrationData *_calibrationData;           // Calibration data
    Signal<const HelioMeasurement *, HELIO_SENSOR_MEASUREMENT_SLOTS> _measureSignal; // New measurement signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;
};


// Simple Binary Sensor
// This class can both read from and assign interrupt routines to a digital input signal,
// allowing it to act as an on/off switch of sorts. Examples include water level indicators.
class HelioBinarySensor : public HelioSensor {
public:
    HelioBinarySensor(Helio_SensorType sensorType,
                      Helio_PositionIndex sensorIndex,
                      HelioDigitalPin inputPin,
                      int classType = Binary);
    HelioBinarySensor(const HelioBinarySensorData *dataIn);
    virtual ~HelioBinarySensor();

    virtual bool takeMeasurement(bool force = false) override;
    virtual const HelioMeasurement *getLatestMeasurement() const override;

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t measurementRow = 0) override;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t measurementRow = 0) const override;

    bool tryRegisterAsISR();

    inline const HelioDigitalPin &getInputPin() const { return _inputPin; }

    Signal<bool, HELIO_SENSOR_MEASUREMENT_SLOTS> &getStateSignal();

    inline void notifyISRTriggered() { takeMeasurement(true); }

protected:
    HelioDigitalPin _inputPin;                              // Digital input pin
    bool _usingISR;                                         // Using ISR flag
    HelioBinaryMeasurement _lastMeasurement;                // Latest successful measurement
    Signal<bool, HELIO_SENSOR_MEASUREMENT_SLOTS> _stateSignal; // State changed signal

    virtual void saveToData(HelioData *dataOut) override;
};


// Standard Analog Sensor
// The ever reliant master of the analog read, this class manages polling an analog input
// signal and converting it into the proper figures for use. Examples include everything
// from TDS EC meters to PWM based flow sensors.
class HelioAnalogSensor : public HelioSensor {
public:
    HelioAnalogSensor(Helio_SensorType sensorType,
                      Helio_PositionIndex sensorIndex,
                      HelioAnalogPin inputPin,
                      bool inputInversion = false,
                      int classType = Analog);
    HelioAnalogSensor(const HelioAnalogSensorData *dataIn);

    virtual bool takeMeasurement(bool force = false) override;
    virtual const HelioMeasurement *getLatestMeasurement() const override;

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t measurementRow = 0) override;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t measurementRow = 0) const override;

    inline const HelioAnalogPin &getInputPin() const { return _inputPin; }
    inline bool getInputInversion() const { return _inputInversion; }

protected:
    HelioAnalogPin _inputPin;                               // Analog input pin
    bool _inputInversion;                                   // Analog input inversion
    HelioSingleMeasurement _lastMeasurement;                // Latest successful measurement
    Helio_UnitsType _measurementUnits;                      // Measurement units preferred

    void _takeMeasurement(unsigned int taskId);

    virtual void saveToData(HelioData *dataOut) override;
};


// Digital Sensor
// Intermediate class for all digital sensors.
class HelioDigitalSensor : public HelioSensor {
public:
    HelioDigitalSensor(Helio_SensorType sensorType,
                       Helio_PositionIndex sensorIndex,
                       HelioDigitalPin inputPin,
                       uint8_t bitRes1W = 9,
                       bool allocate1W = false,
                       int classType = Digital);
    HelioDigitalSensor(const HelioDigitalSensorData *dataIn, bool allocate1W = false);

    virtual bool setWirePositionIndex(Helio_PositionIndex wirePosIndex);
    virtual Helio_PositionIndex getWirePositionIndex() const;

    virtual bool setWireDeviceAddress(const uint8_t wireDevAddress[8]);
    virtual const uint8_t *getWireDeviceAddress() const;

    inline OneWire *getOneWire() const { return _oneWire; }

protected:
    HelioDigitalPin _inputPin;                              // Digital input pin
    OneWire *_oneWire;                                      // OneWire comm instance (strong, nullptr when not used)
    uint8_t _wireBitRes;                                    // OneWire bit resolution
    Helio_PositionIndex _wirePosIndex;                      // OneWire sensor position index
    uint8_t _wireDevAddress[8];                             // OneWire sensor device address

    void resolveDeviceAddress();

    virtual void saveToData(HelioData *dataOut) override;
};


// Digital DHT* Temperature & Humidity Sensor
// This class is for working with DHT* OneWire-based air temperature and humidity sensors.
class HelioDHTTempHumiditySensor : public HelioDigitalSensor {
public:
    HelioDHTTempHumiditySensor(Helio_PositionIndex sensorIndex,
                               HelioDigitalPin inputPin,
                               Helio_DHTType dhtType,
                               bool computeHeatIndex = true,
                               int classType = DHT1W);
    HelioDHTTempHumiditySensor(const HelioDHTTempHumiditySensorData *dataIn);
    virtual ~HelioDHTTempHumiditySensor();

    virtual bool takeMeasurement(bool force = false) override;
    virtual const HelioMeasurement *getLatestMeasurement() const override;

    inline uint8_t getMeasurementRowForTemperature() const { return 0; }
    inline uint8_t getMeasurementRowForHumidity() const { return 1; }
    inline uint8_t getMeasurementRowForHeatIndex() const { return 2; }

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t measurementRow) override;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t measurementRow = 0) const override;

    virtual bool setWirePositionIndex(Helio_PositionIndex wirePosIndex) override; // disabled
    virtual Helio_PositionIndex getWirePositionIndex() const override; // disabled

    virtual bool setWireDeviceAddress(const uint8_t wireDevAddress[8]) override; // disabled
    virtual const uint8_t *getWireDeviceAddress() const override; // disabled

    void setComputeHeatIndex(bool computeHeatIndex);
    inline bool getComputeHeatIndex() const { return _computeHeatIndex; }

protected:
    Helio_DHTType _dhtType;                                 // DHT sensor type
    DHT *_dht;                                              // DHT sensor instance (owned)
    bool _computeHeatIndex;                                 // Flag to compute heat index
    HelioTripleMeasurement _lastMeasurement;                // Latest successful measurement
    Helio_UnitsType _measurementUnits[3];                   // Measurement units preferred

    void _takeMeasurement(unsigned int taskId);

    virtual void saveToData(HelioData *dataOut) override;
};


// Sensor Serialization Data
struct HelioSensorData : public HelioObjectData {
    HelioPinData inputPin;
    char cropName[HELIO_NAME_MAXSIZE];
    char panelName[HELIO_NAME_MAXSIZE];

    HelioSensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Binary Sensor Serialization Data
struct HelioBinarySensorData : public HelioSensorData {
    bool usingISR;

    HelioBinarySensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Analog Sensor Serialization Data
struct HelioAnalogSensorData : public HelioSensorData {
    bool inputInversion;
    Helio_UnitsType measurementUnits;

    HelioAnalogSensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Digital Sensor Serialization Data
struct HelioDigitalSensorData : public HelioSensorData {
    uint8_t wireBitRes;
    Helio_PositionIndex wirePosIndex;
    uint8_t wireDevAddress[8];

    HelioDigitalSensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// DHT TempHumid Sensor Serialization Data
struct HelioDHTTempHumiditySensorData : public HelioDigitalSensorData {
    Helio_DHTType dhtType;
    bool computeHeatIndex;
    Helio_UnitsType measurementUnits;

    HelioDHTTempHumiditySensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioSensors_H

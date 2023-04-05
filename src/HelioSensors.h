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

struct HelioSensorData;
struct HelioBinarySensorData;
struct HelioAnalogSensorData;
struct HelioDigitalSensorData;
struct HelioDHTTempHumiditySensorData;

#include "Helioduino.h"
#include "HelioDatas.h"

// Creates sensor object from passed sensor data (return ownership transfer - user code *must* delete returned object)
extern HelioSensor *newSensorObjectFromData(const HelioSensorData *dataIn);

// Returns default measurement units based on sensorType, optional row index, and measureMode (if undefined then uses active controller's measurement mode, else default measurement mode).
extern Helio_UnitsType defaultUnitsForSensor(Helio_SensorType sensorType, uint8_t measurementRow = 0, Helio_MeasurementMode measureMode = Helio_MeasurementMode_Undefined);
// Returns default measurement category based on sensorType and optional row index (note: this may not accurately produce the correct category, e.g. an ultrasonic distance sensor being used for distance and not volume).
extern Helio_UnitsCategory defaultCategoryForSensor(Helio_SensorType sensorType, uint8_t measurementRow = 0);


// Sensor Base
// This is the base class for all sensors, which defines how the sensor is identified,
// where it lives, and what it's attached to.
class HelioSensor : public HelioObject,
                    public HelioSensorObjectInterface,
                    public HelioMeasurementUnitsInterface,
                    public HelioParentPanelAttachmentInterface {
public:
    const enum : signed char { Binary, Analog, Digital, DHT1W, Unknown = -1 } classType; // Sensor class type (custom RTTI)
    inline bool isBinaryClass() const { return classType == Binary; }
    inline bool isAnalogClass() const { return classType == Analog; }
    inline bool isDigitalClass() const { return classType == Digital; }
    inline bool isDHTClass() const { return classType == DHT1W; }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioSensor(Helio_SensorType sensorType,
                hposi_t sensorIndex,
                int classType = Unknown);
    HelioSensor(const HelioSensorData *dataIn);
    virtual ~HelioSensor();

    virtual void update() override;

    virtual bool isTakingMeasurement() const override;

    void yieldForMeasurement(millis_t timeout = HELIO_DATA_LOOP_INTERVAL);

    virtual HelioAttachment &getParentPanelAttachment() override;

    void setUserCalibrationData(HelioCalibrationData *userCalibrationData);
    inline const HelioCalibrationData *getUserCalibrationData() const { return _calibrationData; }

    // Transformation methods that convert from normalized reading intensity/driver value to calibration units
    inline float calibrationTransform(float value) const { return _calibrationData ? _calibrationData->transform(value) : value; }
    inline void calibrationTransform(float *valueInOut, Helio_UnitsType *unitsOut = nullptr) const { if (valueInOut && _calibrationData) { _calibrationData->transform(valueInOut, unitsOut); } }
    inline HelioSingleMeasurement calibrationTransform(HelioSingleMeasurement measurement) { return _calibrationData ? HelioSingleMeasurement(_calibrationData->transform(measurement.value), _calibrationData->calibrationUnits, measurement.timestamp, measurement.frame) : measurement; }
    inline void calibrationTransform(HelioSingleMeasurement *measurementInOut) const { if (measurementInOut && _calibrationData) { _calibrationData->transform(&measurementInOut->value, &measurementInOut->units); } }

    // Transformation methods that convert from calibration units to normalized reading intensity/driver value
    inline float calibrationInvTransform(float value) const { return _calibrationData ? _calibrationData->inverseTransform(value) : value; }
    inline void calibrationInvTransform(float *valueInOut, Helio_UnitsType *unitsOut = nullptr) const { if (valueInOut && _calibrationData) { _calibrationData->inverseTransform(valueInOut, unitsOut); } }
    inline HelioSingleMeasurement calibrationInvTransform(HelioSingleMeasurement measurement) { return _calibrationData ? HelioSingleMeasurement(_calibrationData->inverseTransform(measurement.value), _calibrationData->calibrationUnits, measurement.timestamp, measurement.frame) : measurement; }
    inline void calibrationInvTransform(HelioSingleMeasurement *measurementInOut) const { if (measurementInOut && _calibrationData) { _calibrationData->inverseTransform(&measurementInOut->value, &measurementInOut->units); } }

    inline Helio_SensorType getSensorType() const { return _id.objTypeAs.sensorType; }
    inline hposi_t getSensorIndex() const { return _id.posIndex; }

    Signal<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS> &getMeasurementSignal();

protected:
    bool _isTakingMeasure;                                  // Taking measurement flag
    HelioAttachment _parentPanel;                           // Parent solar panel attachment
    const HelioCalibrationData *_calibrationData;           // Calibration data
    Signal<const HelioMeasurement *, HELIO_SENSOR_SIGNAL_SLOTS> _measureSignal; // New measurement signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;
};


// Simple Binary Sensor
// This class can both read from and assign interrupt routines to a digital input signal,
// allowing it to act as an on/off switch of sorts. Examples include ice indicators.
// Registering as an ISR allows faster attachment responses than interval polling.
class HelioBinarySensor : public HelioSensor {
public:
    HelioBinarySensor(Helio_SensorType sensorType,
                      hposi_t sensorIndex,
                      HelioDigitalPin inputPin,
                      int classType = Binary);
    HelioBinarySensor(const HelioBinarySensorData *dataIn);
    virtual ~HelioBinarySensor();

    virtual bool takeMeasurement(bool force = false) override;
    virtual const HelioMeasurement *getMeasurement(bool poll = false) override;
    virtual bool needsPolling(hframe_t allowance = 0) const override;

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t = 0) override;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t = 0) const override;

    // ISR registration requires an interruptable pin, i.e. a valid digitalPinToInterrupt(inputPin).
    // Unless anyChange true, active-low input pins interrupt on falling-edge, while active-high input pins interrupt on rising-edge.
    // Interrupt routine does little but create an async Task via TaskManager, eliminating majority of race conditions.
    // Once registered, the ISR cannot be unregistered/changed. It is advised to use a lowered numbered pin # if able ([1-15,18]).
    bool tryRegisterISR(bool anyChange = false);

    inline const HelioDigitalPin &getInputPin() const { return _inputPin; }

    Signal<bool, HELIO_SENSOR_SIGNAL_SLOTS> &getStateSignal();

    inline void notifyISRTriggered() { takeMeasurement(true); }

protected:
    HelioDigitalPin _inputPin;                              // Digital input pin
    bool _usingISR;                                         // Using ISR flag
    HelioBinaryMeasurement _lastMeasurement;                // Latest successful measurement
    Signal<bool, HELIO_SENSOR_SIGNAL_SLOTS> _stateSignal;   // State changed signal

    virtual void saveToData(HelioData *dataOut) override;
};


// Standard Analog Sensor
// The ever reliant master of the analogRead(), this class manages polling an analog input
// signal and converting it into the proper figures for use. Examples include everything
// from TDS EC meters to PWM based flow sensors.
class HelioAnalogSensor : public HelioSensor,
                          public HelioMeasurementUnitsInterfaceStorageSingle {
public:
    HelioAnalogSensor(Helio_SensorType sensorType,
                      hposi_t sensorIndex,
                      HelioAnalogPin inputPin,
                      bool inputInversion = false,
                      int classType = Analog);
    HelioAnalogSensor(const HelioAnalogSensorData *dataIn);

    virtual bool takeMeasurement(bool force = false) override;
    virtual const HelioMeasurement *getMeasurement(bool poll = false) override;
    virtual bool needsPolling(hframe_t allowance = 0) const override;

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t = 0) override;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t = 0) const override;

    inline const HelioAnalogPin &getInputPin() const { return _inputPin; }
    inline bool getInputInversion() const { return _inputInversion; }

protected:
    HelioAnalogPin _inputPin;                               // Analog input pin
    bool _inputInversion;                                   // Analog input inversion
    HelioSingleMeasurement _lastMeasurement;                // Latest successful measurement

    void _takeMeasurement(unsigned int taskId);

    virtual void saveToData(HelioData *dataOut) override;
};


// Digital Sensor
// Intermediate class for all digital sensors.
class HelioDigitalSensor : public HelioSensor {
public:
    HelioDigitalSensor(Helio_SensorType sensorType,
                       hposi_t sensorIndex,
                       HelioDigitalPin inputPin,
                       uint8_t bitRes1W = 9,
                       bool allocate1W = false,
                       int classType = Digital);
    HelioDigitalSensor(const HelioDigitalSensorData *dataIn, bool allocate1W = false);

    virtual bool setWirePositionIndex(hposi_t wirePosIndex);
    virtual hposi_t getWirePositionIndex() const;

    virtual bool setWireDeviceAddress(const uint8_t wireDevAddress[8]);
    virtual const uint8_t *getWireDeviceAddress() const;

    inline OneWire *getOneWire() const { return _oneWire; }

protected:
    HelioDigitalPin _inputPin;                              // Digital input pin
    OneWire *_oneWire;                                      // OneWire comm instance (strong, nullptr when not used)
    uint8_t _wireBitRes;                                    // OneWire bit resolution
    hposi_t _wirePosIndex;                                  // OneWire sensor position index
    uint8_t _wireDevAddress[8];                             // OneWire sensor device address

    void resolveDeviceAddress();

    virtual void saveToData(HelioData *dataOut) override;
};


// Digital DHT* Temperature & Humidity Sensor
// This class is for working with DHT* OneWire-based air temperature and humidity sensors.
class HelioDHTTempHumiditySensor : public HelioDigitalSensor,
                                   public HelioMeasurementUnitsInterfaceStorageTriple {
public:
    HelioDHTTempHumiditySensor(hposi_t sensorIndex,
                               HelioDigitalPin inputPin,
                               Helio_DHTType dhtType,
                               bool computeHeatIndex = true,
                               int classType = DHT1W);
    HelioDHTTempHumiditySensor(const HelioDHTTempHumiditySensorData *dataIn);
    virtual ~HelioDHTTempHumiditySensor();

    virtual bool takeMeasurement(bool force = false) override;
    virtual const HelioMeasurement *getMeasurement(bool poll = false) override;
    virtual bool needsPolling(hframe_t allowance = 0) const override;

    inline uint8_t getMeasurementRowForTemperature() const { return 0; }
    inline uint8_t getMeasurementRowForHumidity() const { return 1; }
    inline uint8_t getMeasurementRowForHeatIndex() const { return 2; }

    virtual void setMeasurementUnits(Helio_UnitsType measurementUnits, uint8_t measurementRow = 0) override;
    virtual Helio_UnitsType getMeasurementUnits(uint8_t measurementRow = 0) const override;

    virtual bool setWirePositionIndex(hposi_t wirePosIndex) override; // disabled
    virtual hposi_t getWirePositionIndex() const override; // disabled

    virtual bool setWireDeviceAddress(const uint8_t wireDevAddress[8]) override; // disabled
    virtual const uint8_t *getWireDeviceAddress() const override; // disabled

    void setComputeHeatIndex(bool computeHeatIndex);
    inline bool getComputeHeatIndex() const { return _computeHeatIndex; }

protected:
    Helio_DHTType _dhtType;                                 // DHT sensor type
    DHT *_dht;                                              // DHT sensor instance (owned)
    bool _computeHeatIndex;                                 // Flag to compute heat index
    HelioTripleMeasurement _lastMeasurement;                // Latest successful measurement

    void _takeMeasurement(unsigned int taskId);

    virtual void saveToData(HelioData *dataOut) override;
};


// Sensor Serialization Data
struct HelioSensorData : public HelioObjectData {
    HelioPinData inputPin;                                  // Input pin
    char panelName[HELIO_NAME_MAXSIZE];                     // Parent panel

    HelioSensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Binary Sensor Serialization Data
struct HelioBinarySensorData : public HelioSensorData {
    bool usingISR;                                          // Using ISR flag

    HelioBinarySensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Analog Sensor Serialization Data
struct HelioAnalogSensorData : public HelioSensorData {
    bool inputInversion;                                    // Input inversion flag
    Helio_UnitsType measurementUnits;                       // Measurement units

    HelioAnalogSensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Digital Sensor Serialization Data
struct HelioDigitalSensorData : public HelioSensorData {
    uint8_t wireBitRes;                                     // 1-Wire bit resolution
    hposi_t wirePosIndex;                                   // 1-Wire position index
    uint8_t wireDevAddress[8];                              // 1-Wire device address

    HelioDigitalSensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// DHT TempHumid Sensor Serialization Data
struct HelioDHTTempHumiditySensorData : public HelioDigitalSensorData {
    Helio_DHTType dhtType;                                  // DHT sensor type
    bool computeHeatIndex;                                  // Compute heat index
    Helio_UnitsType measurementUnits;                       // Measurement units

    HelioDHTTempHumiditySensorData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioSensors_H

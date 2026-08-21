/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Variable Motor Actuator
*/

#ifndef HelioVariableMotorActuator_H
#define HelioVariableMotorActuator_H

#include "Helioduino.h"

// Variable Motor Actuator
// Variable-speed bidirectional motor output. With two output pins the actuator drives
// one PWM direction at a time. With one output pin and a ContinuousServo actuator type,
// the output is centered at the standard hobby-servo neutral pulse and driven in either
// direction around that center point.
class HelioVariableMotorActuator : public HelioVariableActuator,
                                   public HelioMotorObjectInterface,
                                   public HelioDistanceUnitsInterfaceStorage,
                                   public HelioPositionSensorAttachmentInterface,
                                   public HelioSpeedSensorAttachmentInterface,
                                   public HelioMinimumTriggerAttachmentInterface,
                                   public HelioMaximumTriggerAttachmentInterface {
public:
    HelioVariableMotorActuator(Helio_ActuatorType actuatorType,
                               hposi_t actuatorIndex,
                               HelioAnalogPin outputPin,
                               HelioAnalogPin outputPin2 = HelioAnalogPin(),
                               Pair<float,float> travelRange = make_pair(-FLT_MAX, FLT_MAX));
    HelioVariableMotorActuator(const HelioMotorActuatorData *dataIn);
    virtual ~HelioVariableMotorActuator();

    virtual void update() override;
    virtual SharedPtr<HelioObjInterface> getSharedPtrFor(const HelioObjInterface *obj) const override;

    virtual bool getCanEnable() override;
    virtual float getDriveIntensity() const override;
    virtual bool isEnabled(float tolerance = 0.0f) const override;

    virtual bool canTravel(Helio_DirectionMode direction, float distance,
                           Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) override;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, float distance,
                                         Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) override;
    virtual bool canTravel(Helio_DirectionMode direction, millis_t time) override;
    virtual HelioActivationHandle travel(Helio_DirectionMode direction, millis_t time) override;

    virtual void setDistanceUnits(Helio_UnitsType distanceUnits) override;
    virtual void setContinuousSpeed(HelioSingleMeasurement contSpeed) override;
    virtual const HelioSingleMeasurement &getContinuousSpeed() override;

    void setCoastTimeMillis(uint32_t coastTimeMillis);
    inline uint32_t getCoastTimeMillis() const { return _coastTimeMillis; }
    float getCoastDistance(Helio_UnitsType distanceUnits = Helio_UnitsType_Undefined) const;
    Pair<float,float> getTravelRange() const;

    virtual bool isMinTravel(bool poll = false) override;
    virtual bool isMaxTravel(bool poll = false) override;

    virtual HelioSensorAttachment &getPositionSensorAttachment() override;
    virtual HelioSensorAttachment &getSpeedSensorAttachment() override;
    virtual HelioTriggerAttachment &getMinimumTriggerAttachment() override;
    virtual HelioTriggerAttachment &getMaximumTriggerAttachment() override;

    inline const HelioAnalogPin &getOutputPin2() const { return _outputPin2; }
    inline bool isCenteredServoOutput() const {
        return getActuatorType() == Helio_ActuatorType_ContinuousServo && !_outputPin2.isValid();
    }

protected:
    HelioAnalogPin _outputPin2;                             // Reverse-direction PWM output, optional for centered servo
    HelioSingleMeasurement _contSpeed;                     // Continuous speed at full drive
    HelioSensorAttachment _position;                       // Position sensor attachment
    HelioSensorAttachment _speed;                          // Speed sensor attachment
    HelioTriggerAttachment _minimum;                       // Minimum travel trigger
    HelioTriggerAttachment _maximum;                       // Maximum travel trigger
    Pair<float,float> _travelRange;                        // Minimum/maximum travel range
    float _signedIntensity;                                // Signed variable drive intensity [-1,1]
    float _travelPosStart;                                 // Position at movement start
    float _travelDistAccum;                                // Accumulated travel distance
    millis_t _travelTimeStart;                             // Movement start time
    millis_t _travelTimeAccum;                             // Last travel accounting time
    uint32_t _coastTimeMillis;                             // Expected mechanical coast time after power removal

    virtual void saveToData(HelioData *dataOut) override;
    virtual void _enableActuator(float intensity = 1.0f) override;
    virtual void _disableActuator() override;
    virtual void handleActivation() override;
    virtual void handleTravelTime(millis_t time) override;

    void handleMinimumTrigger(Helio_TriggerState minimumTrigger);
    void handleMaximumTrigger(Helio_TriggerState maximumTrigger);
    void writeMotorOutput(float intensity);
};

#endif // /ifndef HelioVariableMotorActuator_H

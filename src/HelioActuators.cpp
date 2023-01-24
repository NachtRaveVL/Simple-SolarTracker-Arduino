/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Actuators
*/

#include "Helioduino.h"

HelioActuator *newActuatorObjectFromData(const HelioActuatorData *dataIn)
{
    if (dataIn && dataIn->id.object.idType == -1) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && dataIn->isObjectData(), SFP(HStr_Err_InvalidParameter));

    if (dataIn && dataIn->isObjectData()) {
        switch (dataIn->id.object.classType) {
            case (int8_t)HelioActuator::Relay:
                return new HelioRelayActuator((const HelioActuatorData *)dataIn);
            case (int8_t)HelioActuator::VariablePWM:
                return new HelioPWMActuator((const HelioActuatorData *)dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioActuator::HelioActuator(Helio_ActuatorType actuatorType,
                             Helio_PositionIndex actuatorIndex,
                             int classTypeIn)
    : HelioObject(HelioIdentity(actuatorType, actuatorIndex)), classType((typeof(classType))classTypeIn),
      _enabled(false), _rail(this), _panel(this)
{ ; }

HelioActuator::HelioActuator(const HelioActuatorData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))dataIn->id.object.classType),
      _enabled(false),
      _contPowerUsage(&(dataIn->contPowerUsage)),
      _rail(this), _panel(this)
{
    _rail.setObject(dataIn->railName);
    _panel.setObject(dataIn->panelName);
}

void HelioActuator::update()
{
    HelioObject::update();

    _rail.resolve();
    _panel.resolve();
}

bool HelioActuator::getCanEnable()
{
    if (getRail() && !getRail()->canActivate(this)) { return false; }
    if (getPanel() && !getPanel()->canActivate(this)) { return false; }
    return true;
}

void HelioActuator::setContinuousPowerUsage(float contPowerUsage, Helio_UnitsType contPowerUsageUnits)
{
    _contPowerUsage.value = contPowerUsage;
    _contPowerUsage.units = contPowerUsageUnits;
    _contPowerUsage.updateTimestamp();
    _contPowerUsage.updateFrame(1);
}

void HelioActuator::setContinuousPowerUsage(HelioSingleMeasurement contPowerUsage)
{
    _contPowerUsage = contPowerUsage;
    _contPowerUsage.setMinFrame(1);
}

const HelioSingleMeasurement &HelioActuator::getContinuousPowerUsage()
{
    return _contPowerUsage;
}

HelioAttachment &HelioActuator::getParentRail(bool resolve)
{
    if (resolve) { _rail.resolve(); }
    return _rail;
}

HelioAttachment &HelioActuator::getParentPanel(bool resolve)
{
    if (resolve) { _panel.resolve(); }
    return _panel;
}

Signal<HelioActuator *> &HelioActuator::getActivationSignal()
{
    return _activateSignal;
}

HelioData *HelioActuator::allocateData() const
{
    return _allocateDataForObjType((int8_t)_id.type, (int8_t)classType);
}

void HelioActuator::saveToData(HelioData *dataOut)
{
    HelioObject::saveToData(dataOut);

    dataOut->id.object.classType = (int8_t)classType;
    if (_contPowerUsage.frame) {
        _contPowerUsage.saveToData(&(((HelioActuatorData *)dataOut)->contPowerUsage));
    }
    if (_panel.getId()) {
        strncpy(((HelioActuatorData *)dataOut)->panelName, _panel.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_rail.getId()) {
        strncpy(((HelioActuatorData *)dataOut)->railName, _rail.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
}


HelioRelayActuator::HelioRelayActuator(Helio_ActuatorType actuatorType,
                                       Helio_PositionIndex actuatorIndex,
                                       HelioDigitalPin outputPin,
                                       int classType)
    : HelioActuator(actuatorType, actuatorIndex, classType),
      _outputPin(outputPin)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.deactivate();
}

HelioRelayActuator::HelioRelayActuator(const HelioActuatorData *dataIn)
    : HelioActuator(dataIn), _outputPin(&dataIn->outputPin)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.deactivate();
}

HelioRelayActuator::~HelioRelayActuator()
{
    if (_enabled) {
        _enabled = false;
        _outputPin.deactivate();
    }
}

bool HelioRelayActuator::enableActuator(float intensity, bool force)
{
    if (_outputPin.isValid()) {
        bool wasEnabledBefore = _enabled;

        if (!_enabled && (force || getCanEnable())) {
            _enabled = true;
            _outputPin.activate();
        }

        if (_enabled && !wasEnabledBefore) {
            getLoggerInstance()->logActivation(this);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
            #else
                _activateSignal.fire(this);
            #endif
        }
    }

    return _enabled;
}

void HelioRelayActuator::disableActuator()
{
    if (_outputPin.isValid()) {
        bool wasEnabledBefore = _enabled;

        if (_enabled) {
            _enabled = false;
            _outputPin.deactivate();
        }

        if (!_enabled && wasEnabledBefore) {
            getLoggerInstance()->logDeactivation(this);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
            #else
                _activateSignal.fire(this);
            #endif
        }
    }
}

bool HelioRelayActuator::isEnabled(float tolerance) const
{
    return _enabled;
}

void HelioRelayActuator::saveToData(HelioData *dataOut)
{
    HelioActuator::saveToData(dataOut);

    _outputPin.saveToData(&((HelioActuatorData *)dataOut)->outputPin);
}


HelioPWMActuator::HelioPWMActuator(Helio_ActuatorType actuatorType,
                                   Helio_PositionIndex actuatorIndex,
                                   HelioAnalogPin outputPin,
                                   int classType)
    : HelioActuator(actuatorType, actuatorIndex, classType),
      _outputPin(outputPin), _pwmAmount(0.0f)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.analogWrite_raw(0);
}

HelioPWMActuator::HelioPWMActuator(const HelioActuatorData *dataIn)
    : HelioActuator(dataIn),
      _outputPin(&dataIn->outputPin), _pwmAmount(0.0f)
{
    HELIO_HARD_ASSERT(_outputPin.isValid(), SFP(HStr_Err_InvalidPinOrType));
    _outputPin.init();
    _outputPin.analogWrite_raw(0);
}

HelioPWMActuator::~HelioPWMActuator()
{
    if (_enabled) {
        _enabled = false;
        _outputPin.analogWrite_raw(0);
    }
}

bool HelioPWMActuator::enableActuator(float intensity, bool force)
{
    if (_outputPin.isValid()) {
        bool wasEnabledBefore = _enabled;

        if ((!_enabled && (force || getCanEnable())) || (_enabled && !isFPEqual(_pwmAmount, intensity))) {
            _enabled = true;
            _pwmAmount = constrain(intensity, 0.0f, 1.0f);
            _outputPin.analogWrite(_pwmAmount);
        }

        if (_enabled && !wasEnabledBefore) {
            getLoggerInstance()->logActivation(this);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
            #else
                _activateSignal.fire(this);
            #endif
        }
    }

    return _enabled;
}

void HelioPWMActuator::disableActuator()
{
    if (_outputPin.isValid()) {
        bool wasEnabledBefore = _enabled;

        if (_enabled) {
            _enabled = false;
            _outputPin.analogWrite_raw(0);
        }

        if (!_enabled && wasEnabledBefore) {
            getLoggerInstance()->logDeactivation(this);

            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioActuator *>(getSharedPtr(), _activateSignal, this);
            #else
                _activateSignal.fire(this);
            #endif
        }
    }
}

bool HelioPWMActuator::isEnabled(float tolerance) const
{
    return _enabled && _pwmAmount >= tolerance - FLT_EPSILON;
}

int HelioPWMActuator::getPWMAmount(int) const
{
    return _outputPin.bitRes.inverseTransform(_pwmAmount);
}

void HelioPWMActuator::setPWMAmount(float amount)
{
    _pwmAmount = constrain(amount, 0.0f, 1.0f);

    if (_enabled) { _outputPin.analogWrite(_pwmAmount); }
}

void HelioPWMActuator::setPWMAmount(int amount)
{
    _pwmAmount = _outputPin.bitRes.transform(amount);

    if (_enabled) { _outputPin.analogWrite(_pwmAmount); }
}

void HelioPWMActuator::saveToData(HelioData *dataOut)
{
    HelioActuator::saveToData(dataOut);

    _outputPin.saveToData(&((HelioActuatorData *)dataOut)->outputPin);
}


HelioActuatorData::HelioActuatorData()
    : HelioObjectData(), outputPin(), contPowerUsage(), railName{0}, panelName{0}
{
    _size = sizeof(*this);
}

void HelioActuatorData::toJSONObject(JsonObject &objectOut) const
{
    HelioObjectData::toJSONObject(objectOut);

    if (isValidPin(outputPin.pin)) {
        JsonObject outputPinObj = objectOut.createNestedObject(SFP(HStr_Key_OutputPin));
        outputPin.toJSONObject(outputPinObj);
    }
    if (contPowerUsage.value > FLT_EPSILON) {
        JsonObject contPowerUsageObj = objectOut.createNestedObject(SFP(HStr_Key_ContPowerUsage));
        contPowerUsage.toJSONObject(contPowerUsageObj);
    }
    if (railName[0]) { objectOut[SFP(HStr_Key_RailName)] = charsToString(railName, HELIO_NAME_MAXSIZE); }
    if (panelName[0]) { objectOut[SFP(HStr_Key_PanelName)] = charsToString(panelName, HELIO_NAME_MAXSIZE); }
}

void HelioActuatorData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioObjectData::fromJSONObject(objectIn);

    JsonObjectConst outputPinObj = objectIn[SFP(HStr_Key_OutputPin)];
    if (!outputPinObj.isNull()) { outputPin.fromJSONObject(outputPinObj); }
    JsonVariantConst contPowerUsageVar = objectIn[SFP(HStr_Key_ContPowerUsage)];
    if (!contPowerUsageVar.isNull()) { contPowerUsage.fromJSONVariant(contPowerUsageVar); }
    const char *railNameStr = objectIn[SFP(HStr_Key_RailName)];
    if (railNameStr && railNameStr[0]) { strncpy(railName, railNameStr, HELIO_NAME_MAXSIZE); }
    const char *panelNameStr = objectIn[SFP(HStr_Key_PanelName)];
    if (panelNameStr && panelNameStr[0]) { strncpy(panelName, panelNameStr, HELIO_NAME_MAXSIZE); }
}

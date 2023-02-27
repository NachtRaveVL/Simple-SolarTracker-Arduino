/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Power Rails 
*/

#include "Helioduino.h"

HelioRail *newRailObjectFromData(const HelioRailData *dataIn)
{
    if (dataIn && isValidType(dataIn->id.object.idType)) return nullptr;
    HELIO_SOFT_ASSERT(dataIn && dataIn->isObjectData(), SFP(HStr_Err_InvalidParameter));

    if (dataIn && dataIn->isObjectData()) {
        switch (dataIn->id.object.classType) {
            case (hid_t)HelioRail::Simple:
                return new HelioSimpleRail((const HelioSimpleRailData *)dataIn);
            case (hid_t)HelioRail::Regulated:
                return new HelioRegulatedRail((const HelioRegulatedRailData *)dataIn);
            default: break;
        }
    }

    return nullptr;
}


HelioRail::HelioRail(Helio_RailType railType, hposi_t railIndex, int classTypeIn)
    : HelioObject(HelioIdentity(railType, railIndex)), classType((typeof(classType))classTypeIn),
      HelioPowerUnitsInterfaceStorage(defaultPowerUnits()),
      _limitState(Helio_TriggerState_Undefined)
{
    allocateLinkages(HELIO_RAILS_LINKS_BASESIZE);
}

HelioRail::HelioRail(const HelioRailData *dataIn)
    : HelioObject(dataIn), classType((typeof(classType))(dataIn->id.object.classType)),
      HelioPowerUnitsInterfaceStorage(definedUnitsElse(dataIn->powerUnits, defaultPowerUnits())),
      _limitState(Helio_TriggerState_Undefined)
{
    allocateLinkages(HELIO_RAILS_LINKS_BASESIZE);
}

HelioRail::~HelioRail()
{
    if (_links) {
        auto actuators = linksFilterActuators(getLinkages());
        for (auto iter = actuators.begin(); iter != actuators.end(); ++iter) { removeLinkage(*iter); }
    }
}

void HelioRail::update()
{
    HelioObject::update();

    handleLimit(triggerStateFromBool(getCapacity(true) >= 1.0f - FLT_EPSILON));
}

bool HelioRail::addLinkage(HelioObject *object)
{
    if (HelioObject::addLinkage(object)) {
        if (object->isActuatorType()) {
            HELIO_HARD_ASSERT(isSimpleClass() || isRegulatedClass(), HStr_Err_OperationFailure);
            if (isSimpleClass()) {
                auto methodSlot = MethodSlot<HelioSimpleRail, HelioActuator *>((HelioSimpleRail *)this, &HelioSimpleRail::handleActivation);
                ((HelioActuator *)object)->getActivationSignal().attach(methodSlot);
            } else if (isRegulatedClass()) {
                auto methodSlot = MethodSlot<HelioRegulatedRail, HelioActuator *>((HelioRegulatedRail *)this, &HelioRegulatedRail::handleActivation);
                ((HelioActuator *)object)->getActivationSignal().attach(methodSlot);
            }
        }
        return true;
    }
    return false;
}

bool HelioRail::removeLinkage(HelioObject *object)
{
    if (HelioObject::removeLinkage(object)) {
        if (((HelioObject *)object)->isActuatorType()) {
            HELIO_HARD_ASSERT(isSimpleClass() || isRegulatedClass(), HStr_Err_OperationFailure);
            if (isSimpleClass()) {
                auto methodSlot = MethodSlot<HelioSimpleRail, HelioActuator *>((HelioSimpleRail *)this, &HelioSimpleRail::handleActivation);
                ((HelioActuator *)object)->getActivationSignal().detach(methodSlot);
            } else if (isRegulatedClass()) {
                auto methodSlot = MethodSlot<HelioRegulatedRail, HelioActuator *>((HelioRegulatedRail *)this, &HelioRegulatedRail::handleActivation);
                ((HelioActuator *)object)->getActivationSignal().detach(methodSlot);
            }
        }
        return true;
    }
    return false;
}

Signal<HelioRail *, HELIO_RAIL_SIGNAL_SLOTS> &HelioRail::getCapacitySignal()
{
    return _capacitySignal;
}

HelioData *HelioRail::allocateData() const
{
    return _allocateDataForObjType((int8_t)_id.type, (int8_t)classType);
}

void HelioRail::saveToData(HelioData *dataOut)
{
    HelioObject::saveToData(dataOut);

    dataOut->id.object.classType = (int8_t)classType;

    ((HelioRailData *)dataOut)->powerUnits = _powerUnits;
}

void HelioRail::handleLimit(Helio_TriggerState limitState)
{
    if (limitState == Helio_TriggerState_Disabled || limitState == Helio_TriggerState_Undefined) { return; }

    if (_limitState != limitState) {
        _limitState = limitState;

        if (_limitState == Helio_TriggerState_NotTriggered) {
            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioRail *>(getSharedPtr(), _capacitySignal, this);
            #else
                _capacitySignal.fire(this);
            #endif
        }
    }
}


HelioSimpleRail::HelioSimpleRail(Helio_RailType railType, hposi_t railIndex, int maxActiveAtOnce, int classType)
    : HelioRail(railType, railIndex, classType), _activeCount(0), _maxActiveAtOnce(maxActiveAtOnce)
{ ; }

HelioSimpleRail::HelioSimpleRail(const HelioSimpleRailData *dataIn)
    : HelioRail(dataIn), _activeCount(0), _maxActiveAtOnce(dataIn->maxActiveAtOnce)
{ ; }

bool HelioSimpleRail::canActivate(HelioActuator *actuator)
{
    return _activeCount < _maxActiveAtOnce;
}

float HelioSimpleRail::getCapacity(bool poll)
{
    return _activeCount / (float)_maxActiveAtOnce;
}

void HelioSimpleRail::setPowerUnits(Helio_UnitsType powerUnits)
{
    if (_powerUnits != powerUnits) {
        _powerUnits = powerUnits;
    }
}

void HelioSimpleRail::saveToData(HelioData *dataOut)
{
    HelioRail::saveToData(dataOut);

    ((HelioSimpleRailData *)dataOut)->maxActiveAtOnce = _maxActiveAtOnce;
}

void HelioSimpleRail::handleActivation(HelioActuator *actuator)
{
    bool activeCountBefore = _activeCount;

    if (actuator->isEnabled()) {
        _activeCount++;
    } else {
        _activeCount--;
    }

    if (_activeCount < activeCountBefore) {
        #ifdef HELIO_USE_MULTITASKING
            scheduleSignalFireOnce<HelioRail *>(getSharedPtr(), _capacitySignal, this);
        #else
            _capacitySignal.fire(this);
        #endif
    }
}


HelioRegulatedRail::HelioRegulatedRail(Helio_RailType railType, hposi_t railIndex, float maxPower, int classType)
    : HelioRail(railType, railIndex, classType), _maxPower(maxPower), _powerUsage(this), _limitTrigger(this)
{
    _powerUsage.setMeasurementUnits(getPowerUnits(), getRailVoltage());
    _powerUsage.setHandleMethod(&HelioRegulatedRail::handlePower);

    _limitTrigger.setHandleMethod(&HelioRail::handleLimit);
}

HelioRegulatedRail::HelioRegulatedRail(const HelioRegulatedRailData *dataIn)
    : HelioRail(dataIn),
      _maxPower(dataIn->maxPower),
      _powerUsage(this), _limitTrigger(this)
{
    _powerUsage.setMeasurementUnits(HelioRail::getPowerUnits(), getRailVoltage());
    _powerUsage.setHandleMethod(&HelioRegulatedRail::handlePower);
    _powerUsage.initObject(dataIn->powerUsageSensor);

    _limitTrigger.setHandleMethod(&HelioRail::handleLimit);
    _limitTrigger.setObject(newTriggerObjectFromSubData(&(dataIn->limitTrigger)));
    HELIO_SOFT_ASSERT(_limitTrigger, SFP(HStr_Err_AllocationFailure));
}

void HelioRegulatedRail::update()
{
    HelioRail::update();

    _powerUsage.updateIfNeeded(true);

    _limitTrigger.updateIfNeeded();
}

SharedPtr<HelioObjInterface> HelioRegulatedRail::getSharedPtrFor(const HelioObjInterface *obj) const
{
    return obj->getKey() == _limitTrigger.getKey() ? _limitTrigger.getSharedPtrFor(obj) :
           HelioObject::getSharedPtrFor(obj);
}

bool HelioRegulatedRail::canActivate(HelioActuator *actuator)
{
    if (_limitTrigger.resolve() && triggerStateToBool(_limitTrigger.getTriggerState())) { return false; }

    HelioSingleMeasurement powerReq = actuator->getContinuousPowerUsage().asUnits(getPowerUnits(), getRailVoltage());

    return _powerUsage.getMeasurementValue(true) + powerReq.value < (HELIO_RAILS_FRACTION_SATURATED * _maxPower) - FLT_EPSILON;
}

float HelioRegulatedRail::getCapacity(bool poll)
{
    if (_limitTrigger.resolve() && triggerStateToBool(_limitTrigger.getTriggerState())) { return 1.0f; }
    return _powerUsage.getMeasurementValue(poll) / (HELIO_RAILS_FRACTION_SATURATED * _maxPower);
}

void HelioRegulatedRail::setPowerUnits(Helio_UnitsType powerUnits)
{
    if (_powerUnits != powerUnits) {
        _powerUnits = powerUnits;

        _powerUsage.setMeasurementUnits(getPowerUnits(), getRailVoltage());
        bumpRevisionIfNeeded();
    }
}

HelioSensorAttachment &HelioRegulatedRail::getPowerUsageSensorAttachment()
{
    return _powerUsage;
}

HelioTriggerAttachment &HelioRegulatedRail::getLimitTriggerAttachment()
{
    return _limitTrigger;
}

void HelioRegulatedRail::saveToData(HelioData *dataOut)
{
    HelioRail::saveToData(dataOut);

    ((HelioRegulatedRailData *)dataOut)->maxPower = roundForExport(_maxPower, 1);
    if (_powerUsage.isSet()) {
        strncpy(((HelioRegulatedRailData *)dataOut)->powerUsageSensor, _powerUsage.getKeyString().c_str(), HELIO_NAME_MAXSIZE);
    }
    if (_limitTrigger.isSet()) {
        _limitTrigger->saveToData(&(((HelioRegulatedRailData *)dataOut)->limitTrigger));
    }
}

void HelioRegulatedRail::handleActivation(HelioActuator *actuator)
{
    if (!getPowerUsageSensor(true) && actuator) {
        auto powerReq = actuator->getContinuousPowerUsage().asUnits(getPowerUnits(), getRailVoltage());
        auto powerUsage = getPowerUsageSensorAttachment().getMeasurement(true);
        bool enabled = actuator->isEnabled();

        if (enabled) {
            powerUsage.value += powerReq.value;
        } else {
            powerUsage.value -= powerReq.value;
        }

        getPowerUsageSensorAttachment().setMeasurement(powerUsage);

        if (!enabled) {
            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioRail *>(getSharedPtr(), _capacitySignal, this);
            #else
                _capacitySignal.fire(this);
            #endif
        }
    }
}

void HelioRegulatedRail::handlePower(const HelioMeasurement *measurement)
{
    if (measurement && measurement->frame) {
        float capacityBefore = getCapacity();

        getPowerUsageSensorAttachment().setMeasurement(getAsSingleMeasurement(measurement, _powerUsage.getMeasurementRow(), _maxPower, getPowerUnits()));

        if (getCapacity() < capacityBefore - FLT_EPSILON) {
            #ifdef HELIO_USE_MULTITASKING
                scheduleSignalFireOnce<HelioRail *>(getSharedPtr(), _capacitySignal, this);
            #else
                _capacitySignal.fire(this);
            #endif
        }
    }
}


HelioRailData::HelioRailData()
    : HelioObjectData(), powerUnits(Helio_UnitsType_Undefined)
{
    _size = sizeof(*this);
}

void HelioRailData::toJSONObject(JsonObject &objectOut) const
{
    HelioObjectData::toJSONObject(objectOut);

    if (powerUnits != Helio_UnitsType_Undefined) { objectOut[SFP(HStr_Key_PowerUnits)] = unitsTypeToSymbol(powerUnits); }
}

void HelioRailData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioObjectData::fromJSONObject(objectIn);

    powerUnits = unitsTypeFromSymbol(objectIn[SFP(HStr_Key_PowerUnits)]);
}

HelioSimpleRailData::HelioSimpleRailData()
    : HelioRailData(), maxActiveAtOnce(2)
{
    _size = sizeof(*this);
}

void HelioSimpleRailData::toJSONObject(JsonObject &objectOut) const
{
    HelioRailData::toJSONObject(objectOut);

    if (maxActiveAtOnce != 2) { objectOut[SFP(HStr_Key_MaxActiveAtOnce)] = maxActiveAtOnce; }
}

void HelioSimpleRailData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioRailData::fromJSONObject(objectIn);

    maxActiveAtOnce = objectIn[SFP(HStr_Key_MaxActiveAtOnce)] | maxActiveAtOnce;
}

HelioRegulatedRailData::HelioRegulatedRailData()
    : HelioRailData(), maxPower(0), powerUsageSensor{0}, limitTrigger()
{
    _size = sizeof(*this);
}

void HelioRegulatedRailData::toJSONObject(JsonObject &objectOut) const
{
    HelioRailData::toJSONObject(objectOut);

    objectOut[SFP(HStr_Key_MaxPower)] = maxPower;
    if (powerUsageSensor[0]) { objectOut[SFP(HStr_Key_PowerUsageSensor)] = charsToString(powerUsageSensor, HELIO_NAME_MAXSIZE); }
    if (isValidType(limitTrigger.type)) {
        JsonObject limitTriggerObj = objectOut.createNestedObject(SFP(HStr_Key_LimitTrigger));
        limitTrigger.toJSONObject(limitTriggerObj);
    }
}

void HelioRegulatedRailData::fromJSONObject(JsonObjectConst &objectIn)
{
    HelioRailData::fromJSONObject(objectIn);

    maxPower = objectIn[SFP(HStr_Key_MaxPower)] | maxPower;
    const char *powerUsageSensorStr = objectIn[SFP(HStr_Key_PowerUsageSensor)];
    if (powerUsageSensorStr && powerUsageSensorStr[0]) { strncpy(powerUsageSensor, powerUsageSensorStr, HELIO_NAME_MAXSIZE); }
    JsonObjectConst limitTriggerObj = objectIn[SFP(HStr_Key_LimitTrigger)];
    if (!limitTriggerObj.isNull()) { limitTrigger.fromJSONObject(limitTriggerObj); }
}

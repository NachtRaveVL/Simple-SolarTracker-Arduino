#!/usr/bin/env python3
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def validate_factories():
    files = ["HelioObject.cpp", "HelioActuators.cpp", "HelioSensors.cpp", "HelioPanels.cpp", "HelioRails.cpp"]
    for filename in files:
        text = (SRC / filename).read_text()
        require("dataIn && !isValidType(dataIn->id.object.idType)" in text,
                f"{filename}: factory validity guard is missing or inverted")


def validate_actuators():
    text = (SRC / "HelioActuators.cpp").read_text()
    header = (SRC / "HelioActuators.h").read_text()
    require("getPanel()" not in text, "Motor actuator still references nonexistent getPanel()")

    save_start = text.index("void HelioRelayMotorActuator::saveToData")
    save_end = text.index("void HelioRelayMotorActuator::handleTravelTime", save_start)
    save_body = text[save_start:save_end]
    require("travelRange[0]" in save_body and "travelRange[1]" in save_body,
            "Motor actuator does not save its travel range")
    require("minTrigger" in save_body and "maxTrigger" in save_body,
            "Motor actuator does not save its travel limit triggers")
    require("coastTimeMillis" in save_body,
            "Motor actuator does not save its coast time")

    ctor_start = text.index("HelioMotorActuatorData::HelioMotorActuatorData")
    to_json_start = text.index("void HelioMotorActuatorData::toJSONObject", ctor_start)
    ctor_body = text[ctor_start:to_json_start]
    require("travelRange{0.0f, FLT_UNDEF}" in ctor_body,
            "Motor actuator data does not initialize its travel range")
    require("coastTimeMillis(0)" in ctor_body,
            "Motor actuator data does not default coast time to zero")

    from_json_start = text.index("void HelioMotorActuatorData::fromJSONObject", to_json_start)
    to_json_body = text[to_json_start:from_json_start]
    from_json_body = text[from_json_start:]
    require("HStr_Key_TravelRange" in to_json_body and "HStr_Key_TravelRange" in from_json_body,
            "Motor actuator JSON does not round-trip its travel range")
    require("HStr_Key_MinTrigger" in to_json_body and "HStr_Key_MinTrigger" in from_json_body and
            "HStr_Key_MaxTrigger" in to_json_body and "HStr_Key_MaxTrigger" in from_json_body,
            "Motor actuator JSON does not round-trip its travel limit triggers")
    require("HStr_Key_OutputPin2" in from_json_body and "outputPin2.fromJSONObject" in from_json_body,
            "Motor actuator data does not restore its second output pin")
    require('"coastTimeMillis"' in to_json_body and '"coastTimeMillis"' in from_json_body,
            "Motor actuator JSON does not round-trip its coast time")

    struct_start = header.index("// Motor Actuator Serialization Data")
    struct_body = header[struct_start:]
    require(struct_body.index("coastTimeMillis") > struct_body.index("maxTrigger"),
            "Motor coast time must remain append-only at the end of motor actuator data")

    travel_start = text.index("void HelioRelayMotorActuator::handleTravelTime")
    travel_body = text[travel_start:]
    require("_intensity < 0.0f ? -1.0f : 1.0f" in travel_body,
            "Sensorless motor travel does not preserve reverse direction")

    activation_start = text.index("void HelioRelayMotorActuator::handleActivation")
    activation_end = text.index("bool HelioRelayMotorActuator::canTravel", activation_start)
    activation_body = text[activation_start:activation_end]
    require("_travelTimeStart = 0;" in activation_body,
            "Motor travel accounting remains active after the motor stops")


def validate_drivers():
    text = (SRC / "HelioDrivers.cpp").read_text()
    require("helioLargerMagnitude" in text,
            "Driver maximum target offset is not sign-safe")
    require("fabsf(getMaxTargetOffset(true))" in text,
            "Driver alignment state is not using absolute target error")
    require("helioShouldHoldIncrementalMotor" in text,
            "Incremental driver is not using coast/overshoot hold logic")
    require("getCoastDistance" in text,
            "Incremental driver is not using per-motor coast distance")


def validate_tracking_correction():
    text = (SRC / "HelioPanels.cpp").read_text()
    header = (SRC / "HelioPanels.h").read_text()
    require("addNetworkSunPositionSample" in header and "addNetworkSunPositionSample" in text,
            "Tracking panel does not expose optional network sun correction samples")
    require("clearNetworkSunCorrection();" in text[text.index("void HelioTrackingPanel::notifyDateChanged"):],
            "Daily network correction is not reset on date change")
    require("correctedSunPosition(0) + _axisOffset[0]" in text and
            "correctedSunPosition(1) + _axisOffset[1]" in text,
            "Tracking correction is not applied before the user axis offset")
    require("_sunPosition[0]" in text[text.index("void HelioTrackingPanel::notifyAlignmentChanged"):text.index("double HelioTrackingPanel::correctedSunPosition")],
            "Mechanical alignment calibration is not kept separate from network correction")


def validate_binary_persistence():
    data = (SRC / "HelioData.cpp").read_text()
    controller = (SRC / "Helioduino.cpp").read_text()
    sensors = (SRC / "HelioSensors.cpp").read_text()
    require("const size_t serializedSize = baseDecode._size;" in data and "helioBinaryDataReadPlan" in data,
            "Binary loader is not respecting serialized record size")
    require("migrateFromBinaryVersion(baseDecode._version)" in data,
            "Binary loader is not applying data-version migrations")
    require("HelioBinarySensorData::migrateFromBinaryVersion" in sensors and "_version = 2;" in sensors,
            "Binary sensor data does not migrate its stable-time field")
    require("HELIO_SOFT_ASSERT(!bytesWritten" not in controller,
            "Binary save still reports successful writes as assertion failures")


def validate_binary_sensor():
    text = (SRC / "HelioSensors.cpp").read_text()
    start = text.index("bool HelioBinarySensor::takeMeasurement")
    end = text.index("const HelioMeasurement *HelioBinarySensor::getMeasurement", start)
    body = text[start:end]
    require("helioUpdateStableBinaryState" in body, "Binary sensor is not using stable-state filtering")
    require("returnPinLock" not in body, "Binary sensor returns a pin lock it did not acquire")


def validate_family_consistency():
    attachments = (SRC / "HelioAttachments.cpp").read_text()
    core_logic = (SRC / "HelioCoreLogic.h").read_text()
    datas = (SRC / "HelioDatas.cpp").read_text()

    require("helioDirectionForOffset(value, FLT_EPSILON)" in attachments,
            "Motor attachment direction is bypassing the shared floating-point direction helper")
    require("value > FLT_EPSILON ? Helio_DirectionMode_Forward" not in attachments,
            "Motor attachment direction still uses an inline raw-epsilon ternary")
    require("isFPEqual(bTerm, 0.0f)" in datas,
            "Calibration two-point setup is bypassing isFPEqual() for its zero-span guard")

    for field in ["copyBytes", "skipBytes"]:
        lines = [line for line in core_logic.splitlines() if re.search(rf"\b{field}\s*;", line)]
        require(lines and all("//" in line for line in lines),
                f"HelioBinaryDataReadPlan.{field} is missing its inline member comment")


def validate_readme():
    readme = (ROOT / "README.md").read_text()
    require("UNDER ACTIVE DEVELOPMENT -- WORK IN PROGRESS" not in readme, "README still has WIP banner")


if __name__ == "__main__":
    validate_factories()
    validate_actuators()
    validate_drivers()
    validate_tracking_correction()
    validate_binary_persistence()
    validate_binary_sensor()
    validate_family_consistency()
    validate_readme()
    print("Helioduino source validation passed")

#!/usr/bin/env python3
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
    strings = (SRC / "HelioStrings.cpp").read_text()
    require("getPanel()" not in text, "Motor actuator still references nonexistent getPanel()")

    save_start = text.index("void HelioRelayMotorActuator::saveToData")
    save_end = text.index("void HelioRelayMotorActuator::handleTravelTime", save_start)
    save_body = text[save_start:save_end]
    require("travelRange[0]" in save_body and "travelRange[1]" in save_body,
            "Motor actuator does not save its travel range")
    require("minTrigger" in save_body and "maxTrigger" in save_body,
            "Motor actuator does not save its travel limit triggers")

    ctor_start = text.index("HelioMotorActuatorData::HelioMotorActuatorData")
    to_json_start = text.index("void HelioMotorActuatorData::toJSONObject", ctor_start)
    ctor_body = text[ctor_start:to_json_start]
    require("travelRange{0.0f, FLT_UNDEF}" in ctor_body,
            "Motor actuator data does not initialize its travel range")

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
    require('"travelRange"' in strings and '"minTrigger"' in strings and '"maxTrigger"' in strings,
            "Motor actuator serialization keys are missing")


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


def validate_readme():
    readme = (ROOT / "README.md").read_text()
    require("UNDER ACTIVE DEVELOPMENT -- WORK IN PROGRESS" not in readme, "README still has WIP banner")


if __name__ == "__main__":
    validate_factories()
    validate_actuators()
    validate_binary_persistence()
    validate_binary_sensor()
    validate_readme()
    print("Helioduino source validation passed")

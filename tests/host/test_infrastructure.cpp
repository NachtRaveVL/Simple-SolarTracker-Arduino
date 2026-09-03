#include "Helioduino.h"
#include <cassert>
#include <iostream>

int main()
{
    Helioduino controller;
    controller.init();

    SharedPtr<HelioObject> nullObject;
    assert(!controller.registerObject(nullObject));
    assert(!controller.unregisterObject(nullObject));

    auto first = controller.addSolarTrackingPanel(Helio_PanelType_Horizontal);
    auto second = controller.addSolarTrackingPanel(Helio_PanelType_Horizontal);
    assert(first && second);
    assert(first->getId().isPanelType());
    assert(first->getId().objTypeAs.panelType == Helio_PanelType_Horizontal);
    assert(first->getId().posIndex != second->getId().posIndex);
    assert(first->getKey() != second->getKey());
    assert(controller.objectById(first->getId()).get() == first.get());
    assert(controller.unregisterObject(second));
    assert(!controller.objectById(second->getId()));
    assert(controller.registerObject(second));
    assert(controller.objectById(second->getId()).get() == second.get());

    HelioSingleMeasurement celsius(20.0f, Helio_UnitsType_Temperature_Celsius, 100, 1);
    HelioSingleMeasurement fahrenheit = celsius.asUnits(Helio_UnitsType_Temperature_Fahrenheit);
    assert(fahrenheit.isSet());
    assert(isFPEqual(fahrenheit.value, 68.0f));

    auto endstop = controller.addEndstopIndicator(4, true);
    assert(endstop);

    HelioCalibrationData calibration(endstop->getId(), Helio_UnitsType_Raw_1);
    calibration.setFromTwoPoints(0.0f, 0.0f, 1.0f, 1.0f);
    endstop->setUserCalibrationData(&calibration);
    assert(controller.hasUserCalibrations());
    assert(endstop->getUserCalibrationData() != nullptr);
    endstop->setUserCalibrationData(nullptr);
    assert(!controller.hasUserCalibrations());
    assert(endstop->getUserCalibrationData() == nullptr);

    endstop->setUserCalibrationData(&calibration);
    assert(controller.hasUserCalibrations());
    assert(endstop->getUserCalibrationData() != nullptr);
    controller.clearUserCalibrations();
    assert(!controller.hasUserCalibrations());
    assert(endstop->getUserCalibrationData() == nullptr);
    assert(controller.getUserCalibrationData(endstop->getKey()) == nullptr);

    HelioData *saved = first->newSaveData();
    assert(saved && saved->isObjectData());
    HelioObject *restored = newObjectFromData(static_cast<HelioObjectData *>(saved));
    assert(restored);
    assert(restored->getId() == first->getId());
    delete restored;
    delete saved;

    std::cout << "PASS Helioduino infrastructure" << std::endl;
    return 0;
}

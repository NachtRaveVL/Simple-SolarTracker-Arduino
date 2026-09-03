#include "Helioduino.h"
#include <cassert>
#include <iostream>

int main()
{
    Helioduino controller;
    controller.init();

    auto panel = controller.addSolarTrackingPanel(Helio_PanelType_Horizontal);
    auto heater = controller.addPanelHeaterRelay(5);
    assert(panel && heater);
    assert(heater->getOutputPin().isValid());

    HelioCalibrationData calibration(heater->getId(), Helio_UnitsType_Raw_1);
    calibration.setFromTwoPoints(0.0f, 0.0f, 1.0f, 1.0f);
    heater->setUserCalibrationData(&calibration);
    assert(heater->getUserCalibrationData() != nullptr);
    controller.clearUserCalibrations();
    assert(heater->getUserCalibrationData() == nullptr);

    heater->setEnableMode(Helio_EnableMode_Highest);
    HelioActivationHandle handle = heater->enableActuator(0.5f, (millis_t)-1, true);
    heater->update();
    assert(heater->isEnabled());

    handle.unset();
    heater->update();
    assert(!heater->isEnabled());

    heater->setParentPanel(panel);
    assert(heater->getParentPanelAttachment().getKey() == panel->getKey());

    std::cout << "PASS Helioduino hardware" << std::endl;
    return 0;
}

cmake_minimum_required(VERSION 3.16)
project(HelioduinoHostTests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wfloat-equal -Werror)
endif()

find_package(Python3 COMPONENTS Interpreter)

set(HELIODUINO_SRC
    ../src/HelioActivation.cpp
    ../src/HelioActuators.cpp
    ../src/HelioAttachments.cpp
    ../src/HelioData.cpp
    ../src/HelioDatas.cpp
    ../src/HelioDrivers.cpp
    ../src/HelioFactory.cpp
    ../src/HelioLogger.cpp
    ../src/HelioMeasurements.cpp
    ../src/HelioModules.cpp
    ../src/HelioObject.cpp
    ../src/HelioPanels.cpp
    ../src/HelioPins.cpp
    ../src/HelioPublisher.cpp
    ../src/HelioRails.cpp
    ../src/HelioScheduler.cpp
    ../src/HelioSensors.cpp
    ../src/HelioStreams.cpp
    ../src/HelioStrings.cpp
    ../src/HelioTriggers.cpp
    ../src/HelioUtils.cpp
    ../src/HelioVariableMotorActuator.cpp
    ../src/Helioduino.cpp
    ../src/shared/HelioUIData.cpp
)

add_library(helioduino_host STATIC ${HELIODUINO_SRC})
target_include_directories(helioduino_host PUBLIC ../src)

add_executable(helioduino_core_tests
    host/test_core_logic.cpp
)
target_link_libraries(helioduino_core_tests PRIVATE helioduino_host)

add_executable(helioduino_hardening_tests
    host/test_hardening.cpp
)
target_link_libraries(helioduino_hardening_tests PRIVATE helioduino_host)

add_executable(helioduino_infrastructure_tests
    host/test_infrastructure.cpp
)
target_link_libraries(helioduino_infrastructure_tests PRIVATE helioduino_host)

add_executable(helioduino_hardware_tests
    host/test_hardware.cpp
)
target_link_libraries(helioduino_hardware_tests PRIVATE helioduino_host)

enable_testing()
add_test(NAME helioduino_core_tests COMMAND helioduino_core_tests)
add_test(NAME helioduino_hardening_tests COMMAND helioduino_hardening_tests)
add_test(NAME helioduino_infrastructure_tests COMMAND helioduino_infrastructure_tests)
add_test(NAME helioduino_hardware_tests COMMAND helioduino_hardware_tests)

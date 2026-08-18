cmake_minimum_required(VERSION 3.16)
project(HelioduinoHostTests LANGUAGES CXX)

enable_testing()

add_executable(helioduino_core_tests
    host/test_core_logic.cpp
)
target_include_directories(helioduino_core_tests PRIVATE ../src)
target_compile_features(helioduino_core_tests PRIVATE cxx_std_17)

add_test(NAME helioduino_core_tests COMMAND helioduino_core_tests)

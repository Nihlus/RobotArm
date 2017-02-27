cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME RobotArm)
project(${PROJECT_NAME})

set(${CMAKE_PROJECT_NAME}_SKETCH RobotArm.ino)
set(${CMAKE_PROJECT_NAME}_BOARD uno)
set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyACM0)

include_directories(${CMAKE_SOURCE_DIR}/lib)
enerate_arduino_firmware(${CMAKE_PROJECT_NAME})
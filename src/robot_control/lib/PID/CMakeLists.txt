cmake_minimum_required(VERSION 3.0.2)
project(PID)

include_directories(include)

add_library(${PROJECT_NAME} PID_interface.hpp PID_interface.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/libsolution)

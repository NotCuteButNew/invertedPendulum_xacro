cmake_minimum_required(VERSION 3.0.2)
project(PID)

# set(CMAKE_CXX_STANDARD 17)
# set(CXX_STANDARD_REQUIRED ON)
message("C++ standard: ${CMAKE_CXX_STANDARD}")
include_directories(include)

aux_source_directory(include INCLUDE_DIR)
aux_source_directory(src SRC_DIR)

add_library(${PROJECT_NAME} ${INCLUDE_DIR} ${SRC_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/PID)

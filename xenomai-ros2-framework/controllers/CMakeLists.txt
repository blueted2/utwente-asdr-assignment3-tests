cmake_minimum_required(VERSION 3.0)

project(ControllerLib)

set(CMAKE_CXX_STANDARD 11)

# glob cpp files from common
file(GLOB COMMON_SRC "common/*.cpp")

add_library(ControllerLib STATIC
    ${COMMON_SRC}
    ControllerPan.cpp
    ControllerTilt.cpp
)

target_include_directories(ControllerLib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)


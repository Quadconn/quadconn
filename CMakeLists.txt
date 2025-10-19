cmake_minimum_required(VERSION 3.23)

project(Control LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
    src/main.cpp
    src/Manipulator.cpp
    src/FastMath.cpp
)

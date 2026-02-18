cmake_minimum_required(VERSION 3.15)

# Project metadata
project(Quadconn
    LANGUAGES CXX
)

# Compiler Options 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/install")

# Build Configuration Options
option(CONTROL    "Build control code"    ON)
option(MOTOR      "Build motor code"      ON)
option(PERCEPTION "Build perception code" ON)

# Libraries
add_subdirectory(src/cpp/common)
add_subdirectory(external/moteus)
add_subdirectory(external/eigen)

# Optionally Build Executables
if (CONTROL)
    add_subdirectory(src/cpp/control)
endif()

if (MOTOR)
    add_subdirectory(src/cpp/motor)
endif()

if (PERCEPTION)
    add_subdirectory(src/cpp/perception)
endif()

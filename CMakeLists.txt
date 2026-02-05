cmake_minimum_required(VERSION 3.15)

# Project metadata
project(quadconn
    LANGUAGES CXX
)

# --- Compiler options ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake")  # Adjust if iceoryx2 is installed elsewhere

# --- Dependencies ---
# Pinocchio exports a modern CMake target: pinocchio::pinocchio
find_package(pinocchio REQUIRED)
# Make sure to install iceoryx2 to usr/local
find_package(iceoryx2-cxx REQUIRED)
# Optional: recommended for robotics math
find_package(Eigen3 REQUIRED)

# for low latency video streaming
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0)


# define folders within src
set(SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(MOTEUS_DIR ${SRC_ROOT}/cpp/moteus)
set(ICEORYX_DIR ${SRC_ROOT}/cpp/iceory)
set(USR_DIR ${SRC_ROOT}/cpp/usr)


# Primary application entries
set(DIAGNOSTICS_SOURCES ${USR_DIR}/diagnostics_reader.cpp)
set(SMOTOR_SOURCES ${USR_DIR}/sMotor_Interface.cpp)
set(CAMERA_SOURCES ${USR_DIR}/camera_stream.cpp)
set(MOTORINIT_SOURCES ${USR_DIR}/sMotor_Interface.cpp)

# `iceoryx2_node` and `moteus` in this repository currently contain headers only.
# If you add .cpp files there later, list them here explicitly, for example:
# set(ICEORYX_SOURCES ${ICEORYX_DIR}/some_impl.cpp)
# set(MOTEUS_SOURCES ${MOTEUS_DIR}/another_impl.cpp)
set(ICEORYX_SOURCES)
set(MOTEUS_SOURCES)

# Create interface libraries for header-only modules
add_library(iceoryx2_node INTERFACE)
target_include_directories(iceoryx2_node INTERFACE ${ICEORYX_DIR})

add_library(moteus INTERFACE)
target_include_directories(moteus INTERFACE ${MOTEUS_DIR})


# --- executables: one per logic block ---
add_executable(diagnostics_reader ${DIAGNOSTICS_SOURCES})
add_executable(sMotor_Interface ${SMOTOR_SOURCES})
add_executable(camera_stream ${CAMERA_SOURCES})

# --- Include directories ---

# Common Dependencies helper to avoid repetition
set(COMMON_LIBS
    pinocchio::pinocchio
    Eigen3::Eigen
    iceoryx2-cxx::shared-lib-cxx
    iceoryx2_node
    moteus
)

# Target: diagnostics_reader
target_include_directories(diagnostics_reader PRIVATE ${USR_DIR})
target_link_libraries(diagnostics_reader PRIVATE ${COMMON_LIBS})

# Target: sMotor_Interface
target_include_directories(sMotor_Interface PRIVATE ${USR_DIR})
target_link_libraries(sMotor_Interface PRIVATE ${COMMON_LIBS})

# Target: camera_stream (Includes GStreamer)
target_include_directories(camera_stream PRIVATE ${USR_DIR})
target_link_libraries(camera_stream 
    PRIVATE 
        ${COMMON_LIBS}
        PkgConfig::GST  # This automatically handles includes AND libs
)

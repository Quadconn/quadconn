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

# define folders within src
set(SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(MOTEUS_DIR ${SRC_ROOT}/cpp/moteus)
set(ICEORYX_DIR ${SRC_ROOT}/cpp/iceory)
set(USR_DIR ${SRC_ROOT}/cpp/usr)


# Primary application entries
set(DIAGNOSTICS_SOURCES
    ${USR_DIR}/diagnostics_reader.cpp
)

set(SMOTOR_SOURCES
    ${USR_DIR}/sMotor_Interface.cpp
)

set(SMOTORG_SOURCES
    ${USR_DIR}/sMotor_Gamepad.cpp
)



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
add_executable(diagnostics_reader ${DIAGNOSTICS_SOURCES} ${ICEORYX_SOURCES} ${MOTEUS_SOURCES})
add_executable(sMotor_Interface ${SMOTOR_SOURCES} ${ICEORYX_SOURCES} ${MOTEUS_SOURCES})
add_executable(sMotor_Gamepad ${SMOTORG_SOURCES} ${ICEORYX_SOURCES} ${MOTEUS_SOURCES})

# --- Include directories ---
# Modern CMake: avoid include_directories() and use target_include_directories only if needed
target_include_directories(diagnostics_reader
    PRIVATE
        ${USR_DIR}
)

target_include_directories(sMotor_Interface
    PRIVATE
        ${USR_DIR}
)

target_include_directories(sMotor_Gamepad
    PRIVATE
        ${USR_DIR}
)

# --- Linking (Modern CMake way) ---
target_link_libraries(diagnostics_reader
    PRIVATE
        pinocchio::pinocchio
        Eigen3::Eigen           # If needed (Pinocchio already uses Eigen internally)
        iceoryx2-cxx::shared-lib-cxx
        iceoryx2_node
        moteus
)

target_link_libraries(sMotor_Interface
    PRIVATE
        pinocchio::pinocchio
        Eigen3::Eigen           # If needed (Pinocchio already uses Eigen internally)
        iceoryx2-cxx::shared-lib-cxx
        iceoryx2_node
        moteus
)

target_link_libraries(sMotor_Gamepad
    PRIVATE
        pinocchio::pinocchio
        Eigen3::Eigen           # If needed (Pinocchio already uses Eigen internally)
        iceoryx2-cxx::shared-lib-cxx
        iceoryx2_node
        moteus
)

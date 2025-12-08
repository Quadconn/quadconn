cmake_minimum_required(VERSION 3.15)

# Project metadata
project(quadconn
    LANGUAGES CXX
)

# --- Compiler options ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
# Pinocchio exports a modern CMake target: pinocchio::pinocchio
find_package(pinocchio REQUIRED)

# Optional: recommended for robotics math
find_package(Eigen3 REQUIRED)

# --- Executable ---
add_executable(quadconn src/main.cpp)

# --- Linking (Modern CMake way) ---
target_link_libraries(quadconn
    PRIVATE
        pinocchio::pinocchio
        Eigen3::Eigen           # If needed (Pinocchio already uses Eigen internally)
)

# --- Include directories ---
# Modern CMake: avoid include_directories() and use target_include_directories only if needed
target_include_directories(quadconn
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

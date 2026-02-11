# --- TODO: REMOVE BEFORE PUSH---
cmake_minimum_required(VERSION 3.15)

# Project metadata
project(quadconn
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_PREFIX_PATH "~/projects/external/iceoryx2/target/ff/cc/install")
list(APPEND CMAKE_PREFIX_PATH "~/projects/external/eigen/build")
# -------------------------------


find_package(iceoryx2-cxx REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(Control)

target_sources(Control
    PRIVATE
        main.cpp
        leg_ik.cpp

    PUBLIC
        FILE_SET HEADERS
        FILES
            leg_ik.hpp
            
)

target_link_libraries(Control
    PRIVATE
        iceoryx2-cxx::static-lib-cxx
        Eigen3::Eigen
)

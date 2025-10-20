cmake_minimum_required(VERSION 3.23)

project(Quadconn LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
    src/main.cpp
)

set(CONTROL_LIB ${PROJECT_NAME}Control)
add_library(${CONTROL_LIB})

target_sources(${CONTROL_LIB}
    PRIVATE
        src/Manipulator.cpp
        src/FastMath.cpp

        PUBLIC
            FILE_SET HEADERS
            BASE_DIRS
                src
            FILES
            src/Manipulator.hpp
            src/FastMath.hpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${CONTROL_LIB}
)


set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

set(PROJECT_BINDINGS ${PROJECT_NAME}Bindings)
pybind11_add_module(${PROJECT_BINDINGS}
    src/Bindings.cpp
)

target_link_libraries(${PROJECT_BINDINGS}
    PRIVATE
        ${CONTROL_LIB}
)

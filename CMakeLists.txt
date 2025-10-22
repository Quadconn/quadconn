cmake_minimum_required(VERSION 3.23)

# -------------------------------
#           Quadconn 
# -------------------------------
project(Quadconn LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
    src/main.cpp
)

# -------------------------------
#          Controls Lib
# -------------------------------
set(CONTROL_LIB "${PROJECT_NAME}Control")
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

# -------------------------------
#           pybind11 
# -------------------------------
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

set(PROJECT_BINDINGS "${PROJECT_NAME}Bindings")
pybind11_add_module(${PROJECT_BINDINGS}
    src/Bindings.cpp
)

target_link_libraries(${PROJECT_BINDINGS}
    PRIVATE
        ${CONTROL_LIB}
)

add_custom_command(
    TARGET ${PROJECT_BINDINGS}
    POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E echo "Generating .pyi for ${PROJECT_BINDINGS}"
    COMMAND "${Python_EXECUTABLE}" -m pybind11_stubgen
            -o ${CMAKE_CURRENT_BINARY_DIR} ${PROJECT_BINDINGS}

    COMMAND ${CMAKE_COMMAND} -E echo "Symlinking binding files to scripts"
    COMMAND ${CMAKE_COMMAND} -E create_symlink 
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_BINDINGS}.pyi
            ${CMAKE_SOURCE_DIR}/scripts/${PROJECT_BINDINGS}.pyi
    COMMAND ${CMAKE_COMMAND} -E create_symlink 
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_BINDINGS}.pyi
            ${CMAKE_SOURCE_DIR}/sim/${PROJECT_BINDINGS}.pyi

    COMMAND ${CMAKE_COMMAND} -E echo "Symlinking binding files to sim"
    COMMAND ${CMAKE_COMMAND} -E create_symlink 
            ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_FILE_NAME:${PROJECT_BINDINGS}>
            ${CMAKE_SOURCE_DIR}/scripts/$<TARGET_FILE_NAME:${PROJECT_BINDINGS}>
    COMMAND ${CMAKE_COMMAND} -E create_symlink 
            ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_FILE_NAME:${PROJECT_BINDINGS}>
            ${CMAKE_SOURCE_DIR}/sim/$<TARGET_FILE_NAME:${PROJECT_BINDINGS}>
    VERBATIM
)

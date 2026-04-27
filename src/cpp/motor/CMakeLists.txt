find_package(iceoryx2-cxx REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SYSTEMD REQUIRED IMPORTED_TARGET libsystemd)
# ----------------------------------------
add_executable(diagnostics_reader)

target_sources(diagnostics_reader
    PRIVATE
        diagnostics_reader.cpp
)

target_link_libraries(diagnostics_reader
    PRIVATE
        iceoryx2-cxx::static-lib-cxx
        moteus 
        common
)

# ----------------------------------------

add_executable(motor_interface)

target_sources(motor_interface
    PRIVATE
        motor_interface.cpp
)

target_link_libraries(motor_interface
    PRIVATE
        iceoryx2-cxx::static-lib-cxx
        moteus
        common
        PkgConfig::SYSTEMD
        
)

# ----------------------------------------

add_executable(zero_motors)

target_sources(zero_motors
    PRIVATE
        zero_motors.cpp
)

target_link_libraries(zero_motors
    PRIVATE
        iceoryx2-cxx::static-lib-cxx
        moteus
        common
)


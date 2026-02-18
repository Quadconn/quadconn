find_package(iceoryx2-cxx REQUIRED)

add_executable(control)

target_sources(control
    PRIVATE
        main.cpp
        quad_control.cpp
)

target_link_libraries(control
    PRIVATE
        iceoryx2-cxx::static-lib-cxx
        Eigen3::Eigen
        common
)

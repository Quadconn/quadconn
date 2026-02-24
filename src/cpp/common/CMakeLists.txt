find_package(iceoryx2-cxx REQUIRED)

add_library(common)

target_sources(common
    PUBLIC
        FILE_SET HEADERS
        FILES
            quad_ipc.hpp
            quad_common.hpp
            joint_angles.hpp
            command.hpp
            
)

target_link_libraries(common
    PRIVATE
        iceoryx2-cxx::static-lib-cxx
)

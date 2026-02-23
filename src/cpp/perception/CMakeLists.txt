find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0)

add_executable(camera_stream)

target_sources(camera_stream
    PRIVATE
        camera_stream.cpp
)

target_link_libraries(camera_stream 
    PRIVATE 
        PkgConfig::GST  # This automatically handles includes AND libs
)



add_executable(udp_sender)

target_sources(udp_sender
    PRIVATE
        udp_sender.cpp
)

target_link_libraries(udp_sender 
    PRIVATE 
        PkgConfig::GST  # This automatically handles includes AND libs
)
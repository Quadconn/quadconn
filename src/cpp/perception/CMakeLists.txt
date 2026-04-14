find_package(PkgConfig REQUIRED)
find_package(iceoryx2-cxx REQUIRED)

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
        iceoryx2-cxx::static-lib-cxx
        moteus
        common
)

add_executable(sf45_collector)

target_sources(sf45_collector
    PRIVATE
        sf45_collector.cpp
        
)

target_link_libraries(sf45_collector 
    PRIVATE 
        PkgConfig::GST  # This automatically handles includes AND libs
        common
)
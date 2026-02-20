add_library(moteus INTERFACE)

# NOTE: Adding "SYSTEM" tells build that this is an external library
# and reduces warnings
target_include_directories(moteus
    SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

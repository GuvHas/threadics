cmake_minimum_required(VERSION 3.16)

# Set the components path for esp-matter
if(DEFINED ENV{ESP_MATTER_PATH})
    # MATTER_SDK_PATH must be set here, before components are processed.
    # The esp_matter component's CMakeLists.txt uses it on its very first line
    # to locate zzz_generated and other connectedhomeip source trees.
    set(MATTER_SDK_PATH "$ENV{ESP_MATTER_PATH}/connectedhomeip/connectedhomeip"
        CACHE INTERNAL "Path to the connectedhomeip (Matter) SDK")

    set(EXTRA_COMPONENT_DIRS $ENV{ESP_MATTER_PATH}/components
                             $ENV{ESP_MATTER_PATH}/connectedhomeip/connectedhomeip/config/esp32/components
                             $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(threadics)

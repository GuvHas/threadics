cmake_minimum_required(VERSION 3.16)

# Set the components path for esp-matter
if(DEFINED ENV{ESP_MATTER_PATH})
    set(EXTRA_COMPONENT_DIRS $ENV{ESP_MATTER_PATH}/components
                             $ENV{ESP_MATTER_PATH}/connectedhomeip/connectedhomeip/config/esp32/components
                             $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(threadics)

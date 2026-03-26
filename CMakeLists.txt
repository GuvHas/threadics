cmake_minimum_required(VERSION 3.16)

# GCC 14 + gnu++2b can fail in connectedhomeip closure-control sources pulled by
# recent esp_matter on ESP-IDF 5.5.x toolchains.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(threadics)

# esp_matter currently injects -std=gnu++2b for its target on some versions.
# That overrides the project standard and reintroduces the closure-control error.
# Normalize esp_matter target flags back to gnu++17.
if(TARGET __idf_espressif__esp_matter)
    get_target_property(_esp_matter_compile_opts __idf_espressif__esp_matter COMPILE_OPTIONS)
    if(_esp_matter_compile_opts)
        list(FILTER _esp_matter_compile_opts EXCLUDE REGEX "^-std=gnu\\+\\+2b$")
        list(FILTER _esp_matter_compile_opts EXCLUDE REGEX "^-std=c\\+\\+2b$")
        set_target_properties(__idf_espressif__esp_matter PROPERTIES COMPILE_OPTIONS "${_esp_matter_compile_opts}")
    endif()
    target_compile_options(__idf_espressif__esp_matter PRIVATE -std=gnu++17)
endif()

idf_component_register(
    SRCS
        "app_main.cpp"
        "matter_device.cpp"
        "lk_ics2.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_matter
        esp_matter_console
        nvs_flash
        esp_event
        esp_netif
        driver
        freertos
        log
        openthread
)

# ---------------------------------------------------------------------------
# GCC 13 / connectedhomeip warning workaround
#
# GCC 13 introduced -Wdangling-reference which flags const-ref-to-temporary
# patterns that are common in connectedhomeip template code.  The Matter SDK
# build system tries to silence unknown diagnostics with the Clang-only flag
# -Wno-unknown-warning-option; GCC 13 doesn't recognise that flag either, so
# the build log shows:
#
#   error: [-Werror=dangling-reference]
#   note: unrecognized command-line option '-Wno-unknown-warning-option'
#         may have been intended to silence earlier diagnostics
#
# Demote -Wdangling-reference from an error for the whole build so the
# connectedhomeip source tree can compile without patching upstream files.
# ---------------------------------------------------------------------------
idf_build_set_property(CXX_COMPILE_OPTIONS "-Wno-dangling-reference" APPEND)

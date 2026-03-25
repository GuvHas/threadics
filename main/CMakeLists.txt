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

# Generate Matter ZAP/data model files if using custom ZAP configuration
# idf_component_set_property(${COMPONENT_NAME} WHOLE_ARCHIVE TRUE)

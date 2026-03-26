idf_component_register(
    SRCS "app_main.cpp" "lk_ics2.cpp" "matter_device.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_matter nvs_flash freertos log openthread driver
)

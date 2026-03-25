#pragma once

// ============================================================
// Matter (CHIP) Project Configuration for LK ICS2 Bridge
// ============================================================

// Device identity
#define CHIP_DEVICE_CONFIG_DEVICE_VENDOR_ID     0xFFF1  // Test vendor (replace with real VID)
#define CHIP_DEVICE_CONFIG_DEVICE_PRODUCT_ID    0x8001  // Product ID

// Device type: Thermostat bridge
#define CHIP_DEVICE_CONFIG_DEVICE_TYPE          0x0301  // Thermostat

// Firmware version
#define CHIP_DEVICE_CONFIG_DEVICE_FIRMWARE_REVISION     "1.0.0"
#define CHIP_DEVICE_CONFIG_DEVICE_FIRMWARE_REVISION_STRING "1.0.0"

// Enable Thread as network transport
#define CHIP_DEVICE_CONFIG_ENABLE_THREAD        1
#define CHIP_DEVICE_CONFIG_THREAD_FTD           1

// Disable WiFi (using Thread only on ESP32-C6)
#define CHIP_DEVICE_CONFIG_ENABLE_WIFI_STATION  0
#define CHIP_DEVICE_CONFIG_ENABLE_WIFI_AP       0

// BLE commissioning window
#define CHIP_DEVICE_CONFIG_BLE_DEVICE_NAME      "LK-ICS2-Bridge"

// Pairing
#define CHIP_DEVICE_CONFIG_USE_TEST_SETUP_PIN_CODE      1
#define CHIP_DEVICE_CONFIG_USE_TEST_PAIRING_CODE        "14526-23040"

// Enable persistent storage
#define CHIP_DEVICE_CONFIG_ENABLE_FACTORY_PROVISIONING  0

// Logging
#ifndef CHIP_DEVICE_CONFIG_LOG_PROVISIONING
#define CHIP_DEVICE_CONFIG_LOG_PROVISIONING  1
#endif

// Failsafe timeout
#define CHIP_DEVICE_CONFIG_FAILSAFE_EXPIRY_LENGTH       60

// Max fabrics
#define CHIP_CONFIG_MAX_FABRICS                 5

// Thread network
#define CHIP_DEVICE_CONFIG_THREAD_TASK_STACK_SIZE       8192

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "lk_ics2.h"
#include "matter_device.h"

// Kconfig-derived values
#include "sdkconfig.h"

static const char *TAG = "app_main";

// ============================================================
// LK ICS 2 update callback
//
// Invoked from the lk_ics2 poll task each time a zone is refreshed.
// Forwards the new data to the Matter attribute store so Home Assistant
// sees live temperature and mode updates.
// ============================================================

static void lk_update_cb(uint8_t zone, const lk_ics2_zone_t *data, void *ctx)
{
    matter_device_update_zone(zone, data);
}

// ============================================================
// NVS initialisation (required by Matter for persistent storage)
// ============================================================

static esp_err_t nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erasing and reinitializing...");
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase failed");
        ret = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "nvs_flash_init failed");

    // Also initialize the factory NVS partition (used by Matter for DAC)
    ret = nvs_flash_init_partition("fctry");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Factory NVS erasing...");
        nvs_flash_erase_partition("fctry");
        ret = nvs_flash_init_partition("fctry");
    }
    if (ret != ESP_OK) {
        // Non-fatal: factory partition may not be present in all flash layouts
        ESP_LOGW(TAG, "Factory NVS init failed (%s) - using default test DAC",
                 esp_err_to_name(ret));
    }

    return ESP_OK;
}

// ============================================================
// Main entry point
// ============================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  LK ICS 2 Matter/Thread Bridge");
    ESP_LOGI(TAG, "  Firmware v1.0.0 - ESP32-C6");
    ESP_LOGI(TAG, "==============================================");

    // --- 1. System init ---
    ESP_ERROR_CHECK(nvs_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_LOGI(TAG, "System init done");

    // --- 2. Start LK ICS 2 Modbus driver ---
    lk_ics2_config_t ics2_cfg = {
        .uart_port          = CONFIG_LK_ICS2_UART_PORT,
        .tx_pin             = CONFIG_LK_ICS2_UART_TX_PIN,
        .rx_pin             = CONFIG_LK_ICS2_UART_RX_PIN,
        .de_pin             = CONFIG_LK_ICS2_RS485_DE_PIN,
        .baud_rate          = CONFIG_LK_ICS2_MODBUS_BAUD_RATE,
        .slave_addr         = (uint8_t)CONFIG_LK_ICS2_MODBUS_SLAVE_ADDR,
        .num_zones          = (uint8_t)CONFIG_LK_ICS2_NUM_ZONES,
        .poll_interval_ms   = CONFIG_LK_ICS2_POLL_INTERVAL_MS,
        .response_timeout_ms = CONFIG_LK_ICS2_MODBUS_TIMEOUT_MS,
        .update_cb          = lk_update_cb,
        .update_cb_ctx      = NULL,
    };

    ESP_ERROR_CHECK(lk_ics2_init(&ics2_cfg));
    ESP_LOGI(TAG, "LK ICS 2 driver started (%d zones)", ics2_cfg.num_zones);

    // --- 3. Initialize Matter node + thermostat endpoints ---
    ESP_ERROR_CHECK(matter_device_init(ics2_cfg.num_zones));
    ESP_LOGI(TAG, "Matter device initialized");

    // --- 4. Start Matter stack (blocks until running) ---
    ESP_ERROR_CHECK(matter_device_start());

    // The Matter stack runs its own tasks. The main task can now idle.
    // The lk_ics2 poll task and Matter event loop handle everything.
    ESP_LOGI(TAG, "Bridge running - waiting for Matter commissioning");
    ESP_LOGI(TAG, "Use a Matter controller (e.g. Home Assistant) to commission this device");
    ESP_LOGI(TAG, "Discriminator: 0x%03X  Passcode: %d",
             CONFIG_MATTER_DISCRIMINATOR, CONFIG_MATTER_PASSCODE);

    // Keep the main task alive (Matter and poll tasks run independently)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "Bridge running - heap free: %lu bytes",
                 (unsigned long)esp_get_free_heap_size());
    }
}

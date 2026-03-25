#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "lk_ics2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize the Matter node.
 *
 * Creates the root endpoint (Basic Information, OTA, etc.) plus one
 * Thermostat endpoint per zone.  Must be called after lk_ics2_init().
 *
 * @param  num_zones  Number of thermostat endpoints to create.
 * @return ESP_OK on success.
 */
esp_err_t matter_device_init(uint8_t num_zones);

/**
 * @brief  Start the Matter stack (commissioning + event loop).
 *
 * Call this after matter_device_init().  Blocks until the stack is running.
 * @return ESP_OK on success.
 */
esp_err_t matter_device_start(void);

/**
 * @brief  Update zone attributes from fresh LK ICS 2 data.
 *
 * Called from the lk_ics2 update callback to push new temperature /
 * mode data into the Matter attribute store so Home Assistant sees it.
 *
 * @param  zone  Zone index (0-based).
 * @param  data  Latest zone data from the LK ICS 2 driver.
 */
void matter_device_update_zone(uint8_t zone, const lk_ics2_zone_t *data);

#ifdef __cplusplus
}
#endif

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "lk_ics2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Create the Matter node and one Thermostat endpoint per heating zone.
 *
 * Registers the attribute write callback (setpoint / mode changes from a
 * Matter controller are forwarded to the LK ICS 2 via Modbus).
 * Must be called before esp_matter::start().
 *
 * @param  num_zones  Number of thermostat endpoints to create (1-12).
 * @return ESP_OK on success.
 */
esp_err_t matter_device_init(uint8_t num_zones);

/**
 * @brief  Push fresh LK ICS 2 data into the Matter attribute store.
 *
 * Must be called from the CHIP task context (e.g. via PlatformMgr().ScheduleWork()).
 *
 * @param  zone  Zone index (0-based).
 * @param  data  Latest zone data from the LK ICS 2 driver.
 */
void matter_device_update_zone(uint8_t zone, const lk_ics2_zone_t *data);

#ifdef __cplusplus
}
#endif

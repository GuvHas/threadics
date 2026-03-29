#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// LK ICS 2 Modbus Register Map
//
// The LK ICS 2 (Intelligent Climate System 2) communicates via
// RS485 Modbus RTU. This driver polls zone temperatures and
// controls setpoints.
//
// Register map (all addresses are 0-based, Modbus function code 3/6):
//
//   Input Registers (FC=04):
//     0x0000 + zone (0..11): Room temperature (signed, unit = 0.1°C)
//     0x0010 + zone (0..11): Floor temperature (signed, unit = 0.1°C)
//     0x0020 + zone (0..11): Actuator position (0-100 %)
//     0x0030               : Controller firmware version
//     0x0031               : Number of configured zones
//
//   Holding Registers (FC=03 read, FC=06 write):
//     0x0100 + zone (0..11): Heating setpoint (signed, unit = 0.1°C)
//     0x0110 + zone (0..11): Zone mode
//                            0 = Off
//                            1 = Heat
//                            2 = Cool
//                            3 = Auto
//     0x0120               : Global system mode (same encoding as zone mode)
//
// NOTE: These register addresses are based on common LK Systems
// Modbus documentation. Verify against your specific ICS 2 firmware
// version using the LK ICS 2 Modbus manual.
// ============================================================

#define LK_ICS2_MAX_ZONES           12

// Modbus register base addresses
#define LK_REG_ROOM_TEMP_BASE       0x0000  // Input register: room temp (0.1°C)
#define LK_REG_FLOOR_TEMP_BASE      0x0010  // Input register: floor temp (0.1°C)
#define LK_REG_ACTUATOR_BASE        0x0020  // Input register: actuator % (0-100)
#define LK_REG_FW_VERSION           0x0030  // Input register: FW version
#define LK_REG_NUM_ZONES            0x0031  // Input register: zone count
#define LK_REG_SETPOINT_BASE        0x0100  // Holding register: heating setpoint (0.1°C)
#define LK_REG_ZONE_MODE_BASE       0x0110  // Holding register: zone mode
#define LK_REG_GLOBAL_MODE          0x0120  // Holding register: global mode

// Zone mode values
typedef enum {
    LK_ZONE_MODE_OFF  = 0,
    LK_ZONE_MODE_HEAT = 1,
    LK_ZONE_MODE_COOL = 2,
    LK_ZONE_MODE_AUTO = 3,
} lk_zone_mode_t;

// Per-zone live data (updated by poll task)
typedef struct {
    bool     valid;                 // true once first successful read
    int16_t  room_temp_cdeg;        // Room temperature in centidegrees (°C x 100)
    int16_t  floor_temp_cdeg;       // Floor temperature in centidegrees (°C x 100)
    uint8_t  actuator_pct;          // Actuator open percentage 0-100
    int16_t  setpoint_cdeg;         // Current setpoint in centidegrees (°C x 100)
    lk_zone_mode_t mode;            // Current zone mode
} lk_ics2_zone_t;

// Callback invoked whenever zone data is refreshed
typedef void (*lk_ics2_update_cb_t)(uint8_t zone, const lk_ics2_zone_t *data, void *ctx);

// Driver configuration
typedef struct {
    int      uart_port;             // UART port number (e.g. UART_NUM_1)
    int      tx_pin;                // UART TX GPIO
    int      rx_pin;                // UART RX GPIO
    int      de_pin;                // RS485 DE/RE GPIO (active-high = TX)
    int      baud_rate;             // Modbus baud rate (default 9600)
    uint8_t  slave_addr;            // Modbus slave address of ICS 2 (1-247)
    uint8_t  num_zones;             // Number of zones to poll (1-12)
    uint32_t poll_interval_ms;      // Poll period in milliseconds
    uint32_t response_timeout_ms;   // Modbus response timeout
    lk_ics2_update_cb_t update_cb;  // Called after each zone update
    void    *update_cb_ctx;         // User context passed to update_cb
} lk_ics2_config_t;

/**
 * @brief  Probe the ICS 2 for the number of configured zones.
 *
 * Must be called before lk_ics2_init(). Initialises the UART and performs a
 * single Modbus read of LK_REG_NUM_ZONES (0x0031). Falls back to a
 * NVS-cached value from the last successful probe if the ICS 2 is unreachable,
 * then to cfg->num_zones as a last resort.
 *
 * The result should be used as the num_zones field for both the subsequent
 * lk_ics2_init() call and for Matter endpoint creation so that exactly the
 * right number of thermostat endpoints is exposed to Home Assistant.
 *
 * @param  cfg    Driver hardware config (uart/pin/baud used; num_zones is fallback).
 * @param  count  Output: zone count. Always written, never 0.
 * @return ESP_OK if the ICS 2 responded; ESP_FAIL if a fallback was used.
 */
esp_err_t lk_ics2_probe_num_zones(const lk_ics2_config_t *cfg, uint8_t *count);

/**
 * @brief  Initialize the LK ICS 2 Modbus driver and start the poll task.
 *
 * @param  cfg  Driver configuration. Caller owns the struct (copied internally).
 * @return ESP_OK on success.
 */
esp_err_t lk_ics2_init(const lk_ics2_config_t *cfg);

/**
 * @brief  Get a snapshot of the latest zone data.
 *
 * @param  zone  Zone index (0-based, 0 to num_zones-1).
 * @param  out   Output struct filled with the latest values.
 * @return ESP_OK if data is valid, ESP_ERR_INVALID_STATE if not yet polled.
 */
esp_err_t lk_ics2_get_zone(uint8_t zone, lk_ics2_zone_t *out);

/**
 * @brief  Write a new heating setpoint for a zone.
 *
 * @param  zone         Zone index (0-based).
 * @param  setpoint_cdeg  Desired setpoint in centidegrees (e.g. 2200 = 22.00°C).
 * @return ESP_OK on success.
 */
esp_err_t lk_ics2_set_setpoint(uint8_t zone, int16_t setpoint_cdeg);

/**
 * @brief  Write a new mode for a zone.
 *
 * @param  zone  Zone index (0-based).
 * @param  mode  Desired zone mode.
 * @return ESP_OK on success.
 */
esp_err_t lk_ics2_set_zone_mode(uint8_t zone, lk_zone_mode_t mode);

/**
 * @brief  Set the global system mode.
 *
 * @param  mode  Desired global mode.
 * @return ESP_OK on success.
 */
esp_err_t lk_ics2_set_global_mode(lk_zone_mode_t mode);

#ifdef __cplusplus
}
#endif

#include "lk_ics2.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "lk_ics2";

// ============================================================
// Internal state
// ============================================================

static lk_ics2_config_t  s_cfg;
static lk_ics2_zone_t    s_zones[LK_ICS2_MAX_ZONES];
static SemaphoreHandle_t s_zone_mutex;     // protects s_zones[] cache
static SemaphoreHandle_t s_bus_mutex;      // serialises all RS485/Modbus transactions
static TaskHandle_t      s_poll_task;

// ============================================================
// Modbus RTU helpers
// ============================================================

// CRC-16/IBM (Modbus)
static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Enable RS485 transmitter (DE high)
static inline void rs485_tx_enable(void)
{
    gpio_set_level((gpio_num_t)s_cfg.de_pin, 1);
}

// Enable RS485 receiver (DE low)
static inline void rs485_rx_enable(void)
{
    gpio_set_level((gpio_num_t)s_cfg.de_pin, 0);
}

// Flush UART RX buffer
static void uart_flush_rx(void)
{
    uart_flush_input((uart_port_t)s_cfg.uart_port);
}

/**
 * Send a Modbus RTU request and receive the response.
 *
 * @param  req      Request bytes (without CRC).
 * @param  req_len  Length of request without CRC.
 * @param  rsp      Buffer for response (caller provides).
 * @param  rsp_max  Maximum bytes to read.
 * @param  rsp_len  Actual bytes received.
 * @return ESP_OK on success, error otherwise.
 */
static esp_err_t modbus_transaction(const uint8_t *req, size_t req_len,
                                     uint8_t *rsp, size_t rsp_max, size_t *rsp_len)
{
    // Guard against calls before lk_ics2_init() has created the mutex.
    if (!s_bus_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    // Serialise all bus access. The poll task and the Matter attribute write
    // callback both call this function from different RTOS tasks; without this
    // lock a poll read and a setpoint write can overlap, causing CRC failures,
    // half-duplex direction glitches, or mixed frames on the RS485 bus.
    xSemaphoreTake(s_bus_mutex, portMAX_DELAY);

    // All Modbus requests are 6 bytes; +2 for CRC = 8 bytes max.
    // Fixed-size avoids the VLA extension (not standard C++).
    uint8_t frame[8];
    if (req_len + 2u > sizeof(frame)) {
        xSemaphoreGive(s_bus_mutex);
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(frame, req, req_len);
    uint16_t crc = modbus_crc16(req, req_len);
    frame[req_len]     = (uint8_t)(crc & 0xFF);
    frame[req_len + 1] = (uint8_t)(crc >> 8);

    uart_port_t port = (uart_port_t)s_cfg.uart_port;

    // Flush any stale bytes
    uart_flush_rx();

    // Enable TX, send frame
    rs485_tx_enable();
    int written = uart_write_bytes(port, (const char *)frame, sizeof(frame));
    // Wait for frame to be clocked out before switching to RX
    uart_wait_tx_done(port, pdMS_TO_TICKS(100));
    rs485_rx_enable();

    esp_err_t ret = ESP_OK;

    if (written != (int)sizeof(frame)) {
        ESP_LOGE(TAG, "UART write failed (%d of %zu bytes)", written, sizeof(frame));
        ret = ESP_FAIL;
        goto done;
    }

    {
        // Read response
        int received = uart_read_bytes(port, rsp, rsp_max,
                                       pdMS_TO_TICKS(s_cfg.response_timeout_ms));
        if (received <= 0) {
            ESP_LOGW(TAG, "Modbus timeout - no response from slave %d", s_cfg.slave_addr);
            ret = ESP_ERR_TIMEOUT;
            goto done;
        }

        *rsp_len = (size_t)received;

        if (*rsp_len < 4) {
            ESP_LOGE(TAG, "Response too short (%zu bytes)", *rsp_len);
            ret = ESP_ERR_INVALID_RESPONSE;
            goto done;
        }
        uint16_t resp_crc = modbus_crc16(rsp, *rsp_len - 2);
        uint16_t rcv_crc  = (uint16_t)(rsp[*rsp_len - 2]) | ((uint16_t)(rsp[*rsp_len - 1]) << 8);
        if (resp_crc != rcv_crc) {
            ESP_LOGE(TAG, "CRC mismatch (calc=0x%04X, rcvd=0x%04X)", resp_crc, rcv_crc);
            ret = ESP_ERR_INVALID_CRC;
            goto done;
        }
        if (rsp[0] != s_cfg.slave_addr) {
            ESP_LOGE(TAG, "Slave addr mismatch (exp=%d, got=%d)", s_cfg.slave_addr, rsp[0]);
            ret = ESP_FAIL;
            goto done;
        }
        if (rsp[1] & 0x80) {
            ESP_LOGE(TAG, "Modbus exception: code=0x%02X", rsp[2]);
            ret = ESP_ERR_INVALID_RESPONSE;
            goto done;
        }
    }

done:
    xSemaphoreGive(s_bus_mutex);
    return ret;
}

/**
 * Modbus FC=03 Read Holding Registers.
 *
 * @param  start_addr  First register address (0-based).
 * @param  count       Number of registers.
 * @param  values      Output array of uint16_t (count elements).
 */
static esp_err_t modbus_read_holding(uint16_t start_addr, uint16_t count, uint16_t *values)
{
    uint8_t req[6] = {
        s_cfg.slave_addr,
        0x03,                            // FC3: Read Holding Registers
        (uint8_t)(start_addr >> 8),
        (uint8_t)(start_addr & 0xFF),
        (uint8_t)(count >> 8),
        (uint8_t)(count & 0xFF),
    };

    if (count > LK_ICS2_MAX_ZONES) return ESP_ERR_INVALID_SIZE;
    // 3-byte header + count×2 data + 2-byte CRC; fixed-size avoids VLA.
    uint8_t rsp[3 + LK_ICS2_MAX_ZONES * 2 + 2];
    size_t  rsp_len = 0;
    esp_err_t ret = modbus_transaction(req, sizeof(req), rsp, sizeof(rsp), &rsp_len);
    if (ret != ESP_OK) return ret;

    // Validate function code and exact frame length before reading data bytes.
    // Byte count alone is not sufficient: a response with the right payload size
    // but the wrong function code (possible on a noisy bus) would pass silently.
    size_t expected_len = 3u + (size_t)count * 2u + 2u; // hdr(3) + data + CRC(2)
    if (rsp_len != expected_len || rsp[1] != 0x03) {
        ESP_LOGE(TAG, "FC03 response invalid (len=%u exp=%u fc=0x%02X)",
                 (unsigned)rsp_len, (unsigned)expected_len, rsp[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t byte_count = rsp[2];
    if (byte_count != count * 2) {
        ESP_LOGE(TAG, "FC03 byte count mismatch (exp=%d, got=%d)", count * 2, byte_count);
        return ESP_ERR_INVALID_RESPONSE;
    }

    for (uint16_t i = 0; i < count; i++) {
        values[i] = ((uint16_t)rsp[3 + i * 2] << 8) | rsp[4 + i * 2];
    }
    return ESP_OK;
}

/**
 * Modbus FC=04 Read Input Registers.
 */
static esp_err_t modbus_read_input(uint16_t start_addr, uint16_t count, uint16_t *values)
{
    uint8_t req[6] = {
        s_cfg.slave_addr,
        0x04,                            // FC4: Read Input Registers
        (uint8_t)(start_addr >> 8),
        (uint8_t)(start_addr & 0xFF),
        (uint8_t)(count >> 8),
        (uint8_t)(count & 0xFF),
    };

    if (count > LK_ICS2_MAX_ZONES) return ESP_ERR_INVALID_SIZE;
    uint8_t rsp[3 + LK_ICS2_MAX_ZONES * 2 + 2];
    size_t  rsp_len = 0;
    esp_err_t ret = modbus_transaction(req, sizeof(req), rsp, sizeof(rsp), &rsp_len);
    if (ret != ESP_OK) return ret;

    size_t expected_len = 3u + (size_t)count * 2u + 2u;
    if (rsp_len != expected_len || rsp[1] != 0x04) {
        ESP_LOGE(TAG, "FC04 response invalid (len=%u exp=%u fc=0x%02X)",
                 (unsigned)rsp_len, (unsigned)expected_len, rsp[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t byte_count = rsp[2];
    if (byte_count != count * 2) {
        ESP_LOGE(TAG, "FC04 byte count mismatch (exp=%d, got=%d)", count * 2, byte_count);
        return ESP_ERR_INVALID_RESPONSE;
    }

    for (uint16_t i = 0; i < count; i++) {
        values[i] = ((uint16_t)rsp[3 + i * 2] << 8) | rsp[4 + i * 2];
    }
    return ESP_OK;
}

/**
 * Modbus FC=06 Write Single Holding Register.
 */
static esp_err_t modbus_write_single(uint16_t addr, uint16_t value)
{
    uint8_t req[6] = {
        s_cfg.slave_addr,
        0x06,                           // FC6: Write Single Register
        (uint8_t)(addr >> 8),
        (uint8_t)(addr & 0xFF),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF),
    };

    uint8_t rsp[8];
    size_t  rsp_len = 0;
    esp_err_t ret = modbus_transaction(req, sizeof(req), rsp, sizeof(rsp), &rsp_len);
    if (ret != ESP_OK) return ret;

    // FC06 normal response echoes the request exactly (function code, register
    // address, and written value).  Validate this so a corrupted or stale
    // response on a noisy RS485 bus is not silently treated as success.
    if (rsp_len != 8 || rsp[1] != 0x06 ||
        rsp[2] != req[2] || rsp[3] != req[3] ||
        rsp[4] != req[4] || rsp[5] != req[5]) {
        ESP_LOGE(TAG, "FC06 echo mismatch (len=%u fc=0x%02X)", (unsigned)rsp_len, rsp[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

// ============================================================
// Zone polling logic
// ============================================================

// Convert LK ICS 2 temperature value (0.1°C, signed) to centidegrees (°C x 100)
static inline int16_t lk_temp_to_cdeg(int16_t lk_val)
{
    return (int16_t)(lk_val * 10);   // x0.1°C → x0.01°C (centidegrees)
}

// Convert centidegrees to LK ICS 2 temperature value (0.1°C)
static inline int16_t cdeg_to_lk_temp(int16_t cdeg)
{
    return (int16_t)(cdeg / 10);     // x0.01°C → x0.1°C
}

// Read all zones in 5 batched Modbus transactions (one per register block)
// instead of 5 transactions per zone. For N zones this cuts bus traffic from
// 5N to 5 requests per poll cycle, reducing contention with write operations
// and improving latency for all zones.
static void poll_all_zones(void)
{
    uint8_t n = s_cfg.num_zones;
    uint16_t raw[LK_ICS2_MAX_ZONES];

    // Room temperature is mandatory. If this batch fails, skip the whole
    // cycle to avoid any partial commit.
    if (modbus_read_input(LK_REG_ROOM_TEMP_BASE, n, raw) != ESP_OK) {
        ESP_LOGW(TAG, "Batch room temp read failed; retaining cached data for all zones");
        return;
    }

    // Snapshot the current state before any field writes so we can detect
    // which zones actually changed and avoid pushing unchanged zones into Matter.
    lk_ics2_zone_t before[LK_ICS2_MAX_ZONES];
    xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
    for (uint8_t z = 0; z < n; z++) before[z] = s_zones[z];
    xSemaphoreGive(s_zone_mutex);

    // Update s_zones[] directly, one field at a time, rather than buffering
    // in a snapshot and committing the whole struct at the end.  The snapshot
    // pattern creates a race: a write-through (lk_ics2_set_setpoint / _zone_mode)
    // that lands between the initial snapshot and the final commit gets silently
    // overwritten by the stale snapshot value, defeating write-through caching.
    // Writing only freshly-read fields under the mutex prevents that race.
    xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
    for (uint8_t z = 0; z < n; z++) {
        s_zones[z].room_temp_cdeg = lk_temp_to_cdeg((int16_t)raw[z]);
        s_zones[z].valid = true;
    }
    xSemaphoreGive(s_zone_mutex);

    // Optional batches: only update the specific field when the read succeeds;
    // leave the previous value (including any write-through) intact on failure.
    if (modbus_read_input(LK_REG_FLOOR_TEMP_BASE, n, raw) == ESP_OK) {
        xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
        for (uint8_t z = 0; z < n; z++)
            s_zones[z].floor_temp_cdeg = lk_temp_to_cdeg((int16_t)raw[z]);
        xSemaphoreGive(s_zone_mutex);
    }

    if (modbus_read_input(LK_REG_ACTUATOR_BASE, n, raw) == ESP_OK) {
        xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
        for (uint8_t z = 0; z < n; z++)
            s_zones[z].actuator_pct = (uint8_t)(raw[z] & 0xFF);
        xSemaphoreGive(s_zone_mutex);
    }

    if (modbus_read_holding(LK_REG_SETPOINT_BASE, n, raw) == ESP_OK) {
        xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
        for (uint8_t z = 0; z < n; z++)
            s_zones[z].setpoint_cdeg = lk_temp_to_cdeg((int16_t)raw[z]);
        xSemaphoreGive(s_zone_mutex);
    }

    if (modbus_read_holding(LK_REG_ZONE_MODE_BASE, n, raw) == ESP_OK) {
        xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
        for (uint8_t z = 0; z < n; z++)
            s_zones[z].mode = (lk_zone_mode_t)(raw[z] & 0x03);
        xSemaphoreGive(s_zone_mutex);
    }

    // Snapshot for callbacks taken after all writes so it reflects the
    // freshest state, including any concurrent write-throughs.
    if (s_cfg.update_cb) {
        lk_ics2_zone_t after[LK_ICS2_MAX_ZONES];
        xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
        for (uint8_t z = 0; z < n; z++) after[z] = s_zones[z];
        xSemaphoreGive(s_zone_mutex);

        for (uint8_t z = 0; z < n; z++) {
            // Only push zones whose Matter-visible fields changed.  This avoids
            // per-cycle heap allocation, CHIP task scheduling, and attribute
            // writes for zones that are in steady state.
            bool changed = !before[z].valid                             ||
                           before[z].room_temp_cdeg != after[z].room_temp_cdeg ||
                           before[z].setpoint_cdeg  != after[z].setpoint_cdeg  ||
                           before[z].mode           != after[z].mode;
            if (!changed) continue;

            s_cfg.update_cb(z, &after[z], s_cfg.update_cb_ctx);
            ESP_LOGD(TAG, "Zone %d: room=%.1f°C floor=%.1f°C sp=%.1f°C act=%d%% mode=%d",
                     z,
                     after[z].room_temp_cdeg  / 100.0f,
                     after[z].floor_temp_cdeg / 100.0f,
                     after[z].setpoint_cdeg   / 100.0f,
                     after[z].actuator_pct,
                     after[z].mode);
        }
    }
}

static void poll_task(void *arg)
{
    ESP_LOGI(TAG, "Poll task started - %d zones, interval=%dms",
             s_cfg.num_zones, (int)s_cfg.poll_interval_ms);

    while (true) {
        poll_all_zones();
        vTaskDelay(pdMS_TO_TICKS(s_cfg.poll_interval_ms));
    }
}

// ============================================================
// UART / RS485 initialisation
// ============================================================

static esp_err_t uart_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate  = s_cfg.baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_port_t port = (uart_port_t)s_cfg.uart_port;

    ESP_RETURN_ON_ERROR(uart_param_config(port, &uart_cfg), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(port,
                                     s_cfg.tx_pin,
                                     s_cfg.rx_pin,
                                     UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");

    // Install driver with 256-byte RX buffer, no TX buffer (we write synchronously)
    ESP_RETURN_ON_ERROR(uart_driver_install(port, 256, 0, 0, NULL, 0),
                        TAG, "uart_driver_install failed");

    // Set RS485 half-duplex mode
    ESP_RETURN_ON_ERROR(uart_set_mode(port, UART_MODE_RS485_HALF_DUPLEX),
                        TAG, "uart_set_mode RS485 failed");

    // Configure DE/RE GPIO
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << s_cfg.de_pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&gpio_cfg), TAG, "gpio_config DE pin failed");
    rs485_rx_enable();   // Start in receive mode

    ESP_LOGI(TAG, "RS485 UART%d init: baud=%d TX=%d RX=%d DE=%d",
             s_cfg.uart_port, s_cfg.baud_rate,
             s_cfg.tx_pin, s_cfg.rx_pin, s_cfg.de_pin);
    return ESP_OK;
}

// ============================================================
// Public API
// ============================================================

esp_err_t lk_ics2_init(const lk_ics2_config_t *cfg)
{
    if (!cfg || cfg->num_zones == 0 || cfg->num_zones > LK_ICS2_MAX_ZONES) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_cfg, cfg, sizeof(s_cfg));
    memset(s_zones, 0, sizeof(s_zones));

    s_zone_mutex = xSemaphoreCreateMutex();
    s_bus_mutex  = xSemaphoreCreateMutex();
    if (!s_zone_mutex || !s_bus_mutex) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(uart_init(), TAG, "UART init failed");

    // Validate configured zone count against the controller at startup.
    // LK_REG_NUM_ZONES (0x0031) is an input register that reports how many
    // zones the ICS 2 is configured with.  A mismatch means the bridge will
    // either expose phantom zones or silently miss real ones.
    {
        uint16_t ctrl_zones = 0;
        esp_err_t zret = modbus_read_input(LK_REG_NUM_ZONES, 1, &ctrl_zones);
        if (zret != ESP_OK) {
            ESP_LOGW(TAG, "Could not read zone count from ICS 2 (err=%s) - proceeding with configured value %d",
                     esp_err_to_name(zret), s_cfg.num_zones);
        } else if (ctrl_zones != s_cfg.num_zones) {
            ESP_LOGW(TAG, "Zone count mismatch: ICS 2 reports %u zones, bridge configured for %u. "
                     "Update CONFIG_LK_ICS2_NUM_ZONES in menuconfig to match.",
                     (unsigned)ctrl_zones, (unsigned)s_cfg.num_zones);
        } else {
            ESP_LOGI(TAG, "Zone count confirmed: %u zones", (unsigned)ctrl_zones);
        }
    }

    // Start poll task on core 1 (or 0 for unicore), low priority
    BaseType_t ret = xTaskCreate(poll_task, "lk_ics2_poll",
                                 4096, NULL, 5, &s_poll_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create poll task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "LK ICS 2 driver initialized (%d zones, slave=%d)",
             s_cfg.num_zones, s_cfg.slave_addr);
    return ESP_OK;
}

esp_err_t lk_ics2_get_zone(uint8_t zone, lk_ics2_zone_t *out)
{
    if (zone >= s_cfg.num_zones || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
    *out = s_zones[zone];
    xSemaphoreGive(s_zone_mutex);

    return out->valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t lk_ics2_set_setpoint(uint8_t zone, int16_t setpoint_cdeg)
{
    if (zone >= s_cfg.num_zones) {
        return ESP_ERR_INVALID_ARG;
    }

    // Clamp to reasonable floor heating range: 5°C - 40°C
    if (setpoint_cdeg < 500)  setpoint_cdeg = 500;
    if (setpoint_cdeg > 4000) setpoint_cdeg = 4000;

    int16_t lk_val = cdeg_to_lk_temp(setpoint_cdeg);
    uint16_t reg_addr = LK_REG_SETPOINT_BASE + zone;

    ESP_LOGI(TAG, "Zone %d: writing setpoint %.1f°C (reg=0x%04X val=%d)",
             zone, setpoint_cdeg / 100.0f, reg_addr, lk_val);

    esp_err_t err = modbus_write_single(reg_addr, (uint16_t)lk_val);
    if (err != ESP_OK) return err;

    // Write-through: update the cache so a subsequent failed poll does not
    // re-publish the old setpoint back to Matter/Home Assistant.
    xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
    s_zones[zone].setpoint_cdeg = setpoint_cdeg;
    xSemaphoreGive(s_zone_mutex);
    return ESP_OK;
}

esp_err_t lk_ics2_set_zone_mode(uint8_t zone, lk_zone_mode_t mode)
{
    if (zone >= s_cfg.num_zones) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t reg_addr = LK_REG_ZONE_MODE_BASE + zone;
    ESP_LOGI(TAG, "Zone %d: writing mode %d (reg=0x%04X)", zone, mode, reg_addr);

    esp_err_t err = modbus_write_single(reg_addr, (uint16_t)mode);
    if (err != ESP_OK) return err;

    // Write-through: update the cache so a subsequent failed poll does not
    // re-publish the old mode back to Matter/Home Assistant.
    xSemaphoreTake(s_zone_mutex, portMAX_DELAY);
    s_zones[zone].mode = mode;
    xSemaphoreGive(s_zone_mutex);
    return ESP_OK;
}

esp_err_t lk_ics2_set_global_mode(lk_zone_mode_t mode)
{
    ESP_LOGI(TAG, "Global: writing mode %d (reg=0x%04X)", mode, LK_REG_GLOBAL_MODE);
    return modbus_write_single(LK_REG_GLOBAL_MODE, (uint16_t)mode);
}

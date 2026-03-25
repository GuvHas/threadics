#include "matter_device.h"
#include "lk_ics2.h"

#include <string.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>

// esp-matter SDK headers
#include <esp_matter.h>
#include <esp_matter_core.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_attribute.h>
#include <esp_matter_cluster.h>
#include <esp_matter_console.h>

// Matter cluster IDs used in this device
#include <app/clusters/thermostat-server/thermostat-server.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static const char *TAG = "matter_dev";

// ============================================================
// Internal state
// ============================================================

// Map from zone index -> Matter endpoint_id
static uint16_t s_zone_endpoint_id[LK_ICS2_MAX_ZONES];
static uint8_t  s_num_zones = 0;

// ============================================================
// Matter system mode <-> LK mode conversion
//
// Matter Thermostat SystemMode attribute values (Thermostat.SystemMode):
//   0 = Off
//   1 = Auto
//   3 = Cool
//   4 = Heat
//   5 = Emergency Heating
//   7 = Fan Only
//   8 = Dry
//
// LK ICS 2 mode values:
//   0 = Off, 1 = Heat, 2 = Cool, 3 = Auto
// ============================================================

static uint8_t lk_mode_to_matter(lk_zone_mode_t lk_mode)
{
    switch (lk_mode) {
        case LK_ZONE_MODE_OFF:  return 0;   // Off
        case LK_ZONE_MODE_HEAT: return 4;   // Heat
        case LK_ZONE_MODE_COOL: return 3;   // Cool
        case LK_ZONE_MODE_AUTO: return 1;   // Auto
        default:                return 0;
    }
}

static lk_zone_mode_t matter_mode_to_lk(uint8_t matter_mode)
{
    switch (matter_mode) {
        case 0:  return LK_ZONE_MODE_OFF;
        case 1:  return LK_ZONE_MODE_AUTO;
        case 3:  return LK_ZONE_MODE_COOL;
        case 4:  return LK_ZONE_MODE_HEAT;
        default: return LK_ZONE_MODE_AUTO;
    }
}

// ============================================================
// Matter attribute write callback
//
// Called by the Matter stack whenever a controller writes an attribute.
// We intercept setpoint and system-mode writes and forward them to ICS 2.
// ============================================================

static esp_err_t attribute_update_cb(attribute::callback_type_t type,
                                      uint16_t endpoint_id,
                                      uint32_t cluster_id,
                                      uint32_t attribute_id,
                                      esp_matter_attr_val_t *val,
                                      void *priv_data)
{
    if (type != PRE_UPDATE) {
        return ESP_OK;
    }

    // Find which zone this endpoint belongs to
    uint8_t zone = 0xFF;
    for (uint8_t z = 0; z < s_num_zones; z++) {
        if (s_zone_endpoint_id[z] == endpoint_id) {
            zone = z;
            break;
        }
    }

    if (zone == 0xFF) {
        return ESP_OK;   // Not one of our zone endpoints
    }

    // Thermostat cluster writes
    if (cluster_id == Thermostat::Id) {
        if (attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id) {
            // Matter setpoint is in centidegrees (x100 °C), signed int16
            int16_t setpoint_cdeg = val->val.i16;
            ESP_LOGI(TAG, "Zone %d: HA→ICS2 setpoint %.2f°C",
                     zone, setpoint_cdeg / 100.0f);
            lk_ics2_set_setpoint(zone, setpoint_cdeg);
        }
        else if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
            uint8_t matter_mode = val->val.u8;
            lk_zone_mode_t lk_mode = matter_mode_to_lk(matter_mode);
            ESP_LOGI(TAG, "Zone %d: HA→ICS2 mode %d→%d", zone, matter_mode, lk_mode);
            lk_ics2_set_zone_mode(zone, lk_mode);
        }
    }

    return ESP_OK;
}

// ============================================================
// Identification callback (for Matter Identify cluster)
// ============================================================

static esp_err_t identification_cb(identification::callback_type_t type,
                                    uint16_t endpoint_id,
                                    uint8_t effect_id,
                                    uint8_t effect_variant,
                                    void *priv_data)
{
    ESP_LOGI(TAG, "Identify: endpoint=%d effect=%d", endpoint_id, effect_id);
    return ESP_OK;
}

// ============================================================
// Endpoint creation
// ============================================================

static esp_err_t create_thermostat_endpoint(uint8_t zone, uint16_t *endpoint_id_out)
{
    // Thermostat endpoint configuration
    thermostat::config_t therm_cfg;
    memset(&therm_cfg, 0, sizeof(therm_cfg));

    // Thermostat cluster defaults
    therm_cfg.thermostat.local_temperature = 2000;              // 20.00°C
    therm_cfg.thermostat.occupied_heating_setpoint = 2100;      // 21.00°C
    therm_cfg.thermostat.occupied_cooling_setpoint = 2600;      // 26.00°C
    therm_cfg.thermostat.min_heat_setpoint_limit = 500;         //  5.00°C
    therm_cfg.thermostat.max_heat_setpoint_limit = 4000;        // 40.00°C
    therm_cfg.thermostat.min_cool_setpoint_limit = 1600;        // 16.00°C
    therm_cfg.thermostat.max_cool_setpoint_limit = 3200;        // 32.00°C
    therm_cfg.thermostat.system_mode = 4;                       // Heat
    therm_cfg.thermostat.control_sequence_of_operation = 4;     // Heating and Cooling
    therm_cfg.thermostat.feature_map = 1;                       // Bit0=Heating

    // Create the thermostat endpoint
    node_t *node = node::get();
    endpoint_t *ep = thermostat::create(node, &therm_cfg, ENDPOINT_FLAG_NONE, NULL);
    if (!ep) {
        ESP_LOGE(TAG, "Zone %d: failed to create thermostat endpoint", zone);
        return ESP_FAIL;
    }

    *endpoint_id_out = endpoint::get_id(ep);
    ESP_LOGI(TAG, "Zone %d: thermostat endpoint created (id=%d)", zone, *endpoint_id_out);

    return ESP_OK;
}

// ============================================================
// Public API
// ============================================================

esp_err_t matter_device_init(uint8_t num_zones)
{
    if (num_zones == 0 || num_zones > LK_ICS2_MAX_ZONES) {
        return ESP_ERR_INVALID_ARG;
    }

    s_num_zones = num_zones;

    // Create the Matter node (root endpoint 0 is created automatically)
    node::config_t node_cfg;
    memset(&node_cfg, 0, sizeof(node_cfg));
    strncpy(node_cfg.root_node.basic_information.vendor_name,    "threadics",       32);
    strncpy(node_cfg.root_node.basic_information.product_name,   "LK ICS2 Bridge", 32);
    strncpy(node_cfg.root_node.basic_information.hardware_version_string, "1.0",   64);
    strncpy(node_cfg.root_node.basic_information.software_version_string, "1.0.0", 64);

    node_t *node = node::create(&node_cfg, attribute_update_cb, identification_cb);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return ESP_FAIL;
    }

    // Create one thermostat endpoint per zone
    for (uint8_t z = 0; z < num_zones; z++) {
        esp_err_t ret = create_thermostat_endpoint(z, &s_zone_endpoint_id[z]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Zone %d: endpoint creation failed", z);
            return ret;
        }
    }

    ESP_LOGI(TAG, "Matter node initialized with %d thermostat endpoint(s)", num_zones);
    return ESP_OK;
}

esp_err_t matter_device_start(void)
{
    ESP_LOGI(TAG, "Starting Matter stack...");
    esp_err_t ret = esp_matter::start(attribute_update_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_matter::start() failed: %s", esp_err_to_name(ret));
        return ret;
    }

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::init();
#endif

    ESP_LOGI(TAG, "Matter stack started");
    return ESP_OK;
}

void matter_device_update_zone(uint8_t zone, const lk_ics2_zone_t *data)
{
    if (zone >= s_num_zones || !data || !data->valid) {
        return;
    }

    uint16_t ep_id = s_zone_endpoint_id[zone];
    node_t  *node  = node::get();
    endpoint_t *ep = endpoint::get(node, ep_id);
    if (!ep) {
        ESP_LOGW(TAG, "Zone %d: endpoint not found for id=%d", zone, ep_id);
        return;
    }

    cluster_t *therm_cluster = cluster::get(ep, Thermostat::Id);
    if (!therm_cluster) {
        ESP_LOGW(TAG, "Zone %d: Thermostat cluster not found", zone);
        return;
    }

    // Update LocalTemperature (room temperature)
    {
        attribute_t *attr = attribute::get(therm_cluster,
                                           Thermostat::Attributes::LocalTemperature::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_nullable_int16(data->room_temp_cdeg);
            attribute::set_val(attr, &val);
        }
    }

    // Update OccupiedHeatingSetpoint
    {
        attribute_t *attr = attribute::get(therm_cluster,
                                           Thermostat::Attributes::OccupiedHeatingSetpoint::Id);
        if (attr) {
            esp_matter_attr_val_t val = esp_matter_int16(data->setpoint_cdeg);
            attribute::set_val(attr, &val);
        }
    }

    // Update SystemMode
    {
        attribute_t *attr = attribute::get(therm_cluster,
                                           Thermostat::Attributes::SystemMode::Id);
        if (attr) {
            uint8_t matter_mode = lk_mode_to_matter(data->mode);
            esp_matter_attr_val_t val = esp_matter_enum8(matter_mode);
            attribute::set_val(attr, &val);
        }
    }

    ESP_LOGD(TAG, "Zone %d (ep %d): Matter attrs updated - temp=%.2f°C sp=%.2f°C mode=%d",
             zone, ep_id,
             data->room_temp_cdeg  / 100.0f,
             data->setpoint_cdeg   / 100.0f,
             data->mode);
}

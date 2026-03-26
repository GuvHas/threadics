#include "matter_device.h"
#include "lk_ics2.h"

#include <string.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_matter.h>
#include <esp_matter_core.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_attribute.h>
#include <esp_matter_cluster.h>

#include <app/clusters/thermostat-server/thermostat-server.h>

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static const char *TAG = "matter_dev";

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

static uint16_t s_zone_endpoint_id[LK_ICS2_MAX_ZONES];
static uint8_t  s_num_zones = 0;

// ---------------------------------------------------------------------------
// Mode conversion: LK ICS 2 ↔ Matter Thermostat SystemMode
//
// Matter SystemMode: 0=Off, 1=Auto, 3=Cool, 4=Heat
// LK ICS 2 mode:    0=Off, 1=Heat, 2=Cool, 3=Auto
// ---------------------------------------------------------------------------

static uint8_t lk_mode_to_matter(lk_zone_mode_t lk_mode)
{
    switch (lk_mode) {
    case LK_ZONE_MODE_OFF:  return 0;
    case LK_ZONE_MODE_HEAT: return 4;
    case LK_ZONE_MODE_COOL: return 3;
    case LK_ZONE_MODE_AUTO: return 1;
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

// ---------------------------------------------------------------------------
// Matter attribute write callback
//
// Intercepts setpoint and system-mode writes from a controller and forwards
// them to the LK ICS 2 via Modbus.
// ---------------------------------------------------------------------------

static esp_err_t attribute_update_cb(attribute::callback_type_t type,
                                     uint16_t endpoint_id,
                                     uint32_t cluster_id,
                                     uint32_t attribute_id,
                                     esp_matter_attr_val_t *val,
                                     void *priv_data)
{
    if (type != PRE_UPDATE) return ESP_OK;

    uint8_t zone = 0xFF;
    for (uint8_t z = 0; z < s_num_zones; z++) {
        if (s_zone_endpoint_id[z] == endpoint_id) { zone = z; break; }
    }
    if (zone == 0xFF) return ESP_OK;

    if (cluster_id == Thermostat::Id) {
        if (attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id) {
            int16_t sp = val->val.i16;
            ESP_LOGI(TAG, "Zone %d: HA→ICS2 setpoint %.2f°C", zone, sp / 100.0f);
            esp_err_t err = lk_ics2_set_setpoint(zone, sp);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Zone %d: setpoint Modbus write failed: %s", zone, esp_err_to_name(err));
                return err;  // Reject the attribute update so HA stays in sync with the device
            }
        } else if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
            uint8_t m = val->val.u8;
            // Reject modes we cannot map to ICS 2 rather than silently changing behaviour.
            // Valid Matter→LK mappings: 0=Off, 1=Auto, 3=Cool, 4=Heat.
            if (m != 0 && m != 1 && m != 3 && m != 4) {
                ESP_LOGW(TAG, "Zone %d: unsupported SystemMode %d rejected", zone, m);
                return ESP_ERR_NOT_SUPPORTED;
            }
            lk_zone_mode_t lk_mode = matter_mode_to_lk(m);
            ESP_LOGI(TAG, "Zone %d: HA→ICS2 mode %d→%d", zone, m, lk_mode);
            esp_err_t err = lk_ics2_set_zone_mode(zone, lk_mode);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Zone %d: mode Modbus write failed: %s", zone, esp_err_to_name(err));
                return err;
            }
        }
    }
    return ESP_OK;
}

static esp_err_t identification_cb(identification::callback_type_t type,
                                   uint16_t endpoint_id,
                                   uint8_t effect_id,
                                   uint8_t effect_variant,
                                   void *priv_data)
{
    ESP_LOGI(TAG, "Identify: endpoint=%d effect=%d", endpoint_id, effect_id);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t matter_device_init(uint8_t num_zones)
{
    if (num_zones == 0 || num_zones > LK_ICS2_MAX_ZONES) {
        return ESP_ERR_INVALID_ARG;
    }
    s_num_zones = num_zones;

    node::config_t node_cfg{};
    node_t *node = node::create(&node_cfg, attribute_update_cb, identification_cb);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return ESP_FAIL;
    }

    for (uint8_t z = 0; z < num_zones; z++) {
        // esp-matter 1.4: setpoints live inside features.heating/cooling.
        // This is a heating-only device (underfloor heating); cooling fields
        // and control_sequence_of_operation are set accordingly.
        thermostat::config_t therm_cfg{};
        therm_cfg.thermostat.local_temperature                          = 2000;  // 20.00°C
        therm_cfg.thermostat.features.heating.occupied_heating_setpoint = 2100;  // 21.00°C
        therm_cfg.thermostat.system_mode                                =    4;  // Heat
        therm_cfg.thermostat.control_sequence_of_operation              =    2;  // Heating Only
        therm_cfg.thermostat.feature_flags                              =    1;  // Bit0=Heating

        endpoint_t *ep = thermostat::create(node, &therm_cfg, ENDPOINT_FLAG_NONE, nullptr);
        if (!ep) {
            ESP_LOGE(TAG, "Zone %d: failed to create thermostat endpoint", z);
            return ESP_FAIL;
        }
        s_zone_endpoint_id[z] = endpoint::get_id(ep);
        ESP_LOGI(TAG, "Zone %d: thermostat endpoint created (id=%d)", z, s_zone_endpoint_id[z]);
    }

    ESP_LOGI(TAG, "Matter node initialized with %d thermostat endpoint(s)", num_zones);
    return ESP_OK;
}

void matter_device_update_zone(uint8_t zone, const lk_ics2_zone_t *data)
{
    if (zone >= s_num_zones || !data || !data->valid) return;

    uint16_t ep_id = s_zone_endpoint_id[zone];
    node_t  *node  = node::get();
    endpoint_t *ep = endpoint::get(node, ep_id);
    if (!ep) return;

    cluster_t *tc = cluster::get(ep, Thermostat::Id);
    if (!tc) return;

    auto setAttr = [&](uint32_t attr_id, esp_matter_attr_val_t val) {
        attribute_t *a = attribute::get(tc, attr_id);
        if (a) attribute::set_val(a, &val);
    };

    setAttr(Thermostat::Attributes::LocalTemperature::Id,
            esp_matter_nullable_int16(data->room_temp_cdeg));
    setAttr(Thermostat::Attributes::OccupiedHeatingSetpoint::Id,
            esp_matter_int16(data->setpoint_cdeg));
    setAttr(Thermostat::Attributes::SystemMode::Id,
            esp_matter_enum8(lk_mode_to_matter(data->mode)));

    ESP_LOGD(TAG, "Zone %d (ep %d): temp=%.2f°C sp=%.2f°C mode=%d",
             zone, ep_id,
             data->room_temp_cdeg / 100.0f,
             data->setpoint_cdeg  / 100.0f,
             data->mode);
}

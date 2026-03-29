#include <driver/gpio.h>
#include <esp_system.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include <openthread/srp_client.h>
#include <openthread/link.h>
#include <openthread/thread.h>
#include <platform/CommissionableDataProvider.h>
#include <platform/DeviceInstanceInfoProvider.h>
#include <app/server/Server.h>
#include <esp_openthread.h>
#include <platform/ESP32/OpenthreadLauncher.h>
#include <platform/internal/BLEManager.h>

#include "lk_ics2.h"
#include "matter_device.h"

using namespace esp_matter;

namespace {
constexpr const char *kTag = "threadics";

// ---------------------------------------------------------------------------
// BLE shutdown
// ---------------------------------------------------------------------------

void shutdownBLE()
{
    chip::DeviceLayer::Internal::BLEMgr().Shutdown();
    ESP_LOGI(kTag, "BLE stack shut down");
}

// ---------------------------------------------------------------------------
// QR code / manual pairing code helpers (verbatim from reference)
// ---------------------------------------------------------------------------

static void stuffBits(uint8_t *buf, int bitOff, uint32_t val, int numBits)
{
    for (int i = 0; i < numBits; ++i) {
        if ((val >> i) & 1u)
            buf[(bitOff + i) / 8] |= (uint8_t)(1u << ((bitOff + i) % 8));
    }
}

static constexpr char kB38[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-.";
static size_t base38Encode(const uint8_t *src, size_t srcLen, char *dst, size_t dstCap)
{
    size_t di = 0;
    for (size_t si = 0; si < srcLen;) {
        size_t nb = (si + 3 <= srcLen) ? 3u : (srcLen - si);
        size_t nc = (nb == 3) ? 5u : 3u;
        uint32_t v = 0;
        for (size_t k = 0; k < nb; ++k) v |= (uint32_t)src[si + k] << (8 * k);
        si += nb;
        for (size_t k = 0; k < nc && di < dstCap; ++k, v /= 38) dst[di++] = kB38[v % 38];
    }
    return di;
}

static const uint8_t kVD[10][10] = {
    {0,1,2,3,4,5,6,7,8,9},{1,2,3,4,0,6,7,8,9,5},{2,3,4,0,1,7,8,9,5,6},
    {3,4,0,1,2,8,9,5,6,7},{4,0,1,2,3,9,5,6,7,8},{5,9,8,7,6,0,4,3,2,1},
    {6,5,9,8,7,1,0,4,3,2},{7,6,5,9,8,2,1,0,4,3},{8,7,6,5,9,3,2,1,0,4},
    {9,8,7,6,5,4,3,2,1,0},
};
static const uint8_t kVP[8][10] = {
    {0,1,2,3,4,5,6,7,8,9},{1,5,7,6,2,8,3,0,9,4},{5,8,0,3,7,9,6,1,4,2},
    {8,9,1,6,0,4,3,5,2,7},{9,4,5,3,1,2,6,8,7,0},{4,2,8,6,5,7,3,9,0,1},
    {2,7,9,3,8,0,6,4,1,5},{7,0,4,6,9,1,3,2,5,8},
};
static const uint8_t kVInv[10] = {0,4,3,2,1,5,6,7,8,9};

static uint32_t verhoeff10Check(const char *s, size_t len)
{
    int c = 0;
    for (size_t i = len; i > 0; --i) {
        int p = kVP[(len - i + 1) % 8][(uint8_t)(s[i - 1] - '0')];
        c = kVD[c][p];
    }
    return kVInv[c];
}

void printCommissioningCodes()
{
    uint32_t passcode      = 0;
    uint16_t discriminator = 0, vid = 0, pid = 0;

    auto *cdp = chip::DeviceLayer::GetCommissionableDataProvider();
    if (!cdp ||
        cdp->GetSetupPasscode(passcode)          != CHIP_NO_ERROR ||
        cdp->GetSetupDiscriminator(discriminator) != CHIP_NO_ERROR) {
        ESP_LOGE(kTag, "Cannot read setup payload from NVS");
        return;
    }
    auto *diip = chip::DeviceLayer::GetDeviceInstanceInfoProvider();
    if (diip) {
        diip->GetVendorId(vid);
        diip->GetProductId(pid);
    }

    uint8_t payload[11] = {};
    stuffBits(payload,  0, 0,             3);
    stuffBits(payload,  3, vid,          16);
    stuffBits(payload, 19, pid,          16);
    stuffBits(payload, 35, 0,             2);
    stuffBits(payload, 37, 0x02,          8);
    stuffBits(payload, 45, discriminator, 12);
    stuffBits(payload, 57, passcode,      27);

    char qr[24] = "MT:";
    size_t n = base38Encode(payload, sizeof(payload), qr + 3, sizeof(qr) - 4);
    qr[3 + n] = '\0';
    ESP_LOGI(kTag, "SetupQRCode: [%s]", qr);

    uint8_t  sd = (uint8_t)((discriminator >> 8) & 0xF);
    uint32_t c1 = (uint32_t)(sd >> 2);
    uint32_t c2 = ((uint32_t)(sd & 0x3) << 14) | (passcode & 0x3FFFu);
    uint32_t c3 = passcode >> 14;
    char tenDigits[16];
    snprintf(tenDigits, sizeof(tenDigits), "%01u%05u%04u", (unsigned)c1, (unsigned)c2, (unsigned)c3);
    uint32_t checkDigit = verhoeff10Check(tenDigits, 10);
    ESP_LOGI(kTag, "ManualPairingCode: [%s%u]", tenDigits, (unsigned)checkDigit);
}

// ---------------------------------------------------------------------------
// LK ICS 2 zone-update → Matter attribute bridge
//
// The lk_ics2 poll task calls this from a FreeRTOS context.  We marshal the
// update onto the CHIP task via ScheduleWork so that attribute::set_val() runs
// in the correct thread context.
// ---------------------------------------------------------------------------

struct ZoneUpdateWork {
    uint8_t       zone;
    lk_ics2_zone_t data;
};

static void lk_zone_update_cb(uint8_t zone, const lk_ics2_zone_t *data, void *ctx)
{
    auto *work = new (std::nothrow) ZoneUpdateWork{zone, *data};
    if (!work) {
        ESP_LOGE(kTag, "lk_zone_update_cb: OOM, skipping zone %d update", zone);
        return;
    }
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t p) {
        auto *w = reinterpret_cast<ZoneUpdateWork *>(p);
        matter_device_update_zone(w->zone, &w->data);
        delete w;
    }, reinterpret_cast<intptr_t>(work));
}

// ---------------------------------------------------------------------------
// SRP operational advertisement for Matter over Thread
//
// The CHIP SDK's ESP32 DNS-SD implementation publishes _matter._tcp via the
// esp-idf mDNS library, which fails (error 46) when WiFi is disabled.  It does
// NOT fall back to OpenThread SRP.  Without an SRP registration the OTBR has
// no record to proxy to mDNS, so the commissioner cannot locate the device for
// the CASE session and CommissioningComplete is never sent.
// ---------------------------------------------------------------------------

namespace {

struct SrpCtx {
    otSrpClientService svc              = {};
    char               instanceName[34] = {};
    char               hostname[17]     = {};
    otDnsTxtEntry      txt[2]           = {};
    bool               added            = false;
} s_srp;

void setupSrpHost(otInstance *ot)
{
    const otExtAddress *ext = otLinkGetExtendedAddress(ot);
    snprintf(s_srp.hostname, sizeof(s_srp.hostname),
             "%02x%02x%02x%02x%02x%02x%02x%02x",
             ext->m8[0], ext->m8[1], ext->m8[2], ext->m8[3],
             ext->m8[4], ext->m8[5], ext->m8[6], ext->m8[7]);
    otSrpClientSetHostName(ot, s_srp.hostname);
    otSrpClientEnableAutoHostAddress(ot);
    // Enable auto-start so the SRP client discovers the OTBR's SRP server
    // from Thread Network Data and begins sending registrations automatically.
    // Without this the client is configured but never contacts the SRP server,
    // so _matter._tcp is never proxied by the OTBR and the commissioner cannot
    // reconnect for the post-AddNOC CASE session or after any power cycle.
    otSrpClientEnableAutoStartMode(ot, nullptr, nullptr);
}

void trySrpServiceAdd(otInstance *ot,
                      chip::FabricIndex preferIndex = chip::kUndefinedFabricIndex)
{
    if (s_srp.added) return;
    if (otThreadGetDeviceRole(ot) <= OT_DEVICE_ROLE_DETACHED) return;

    const chip::FabricInfo *fabric = nullptr;
    if (preferIndex != chip::kUndefinedFabricIndex) {
        fabric = chip::Server::GetInstance().GetFabricTable().FindFabricWithIndex(preferIndex);
    }
    if (!fabric) {
        for (const auto &f : chip::Server::GetInstance().GetFabricTable()) {
            fabric = &f;
            break;
        }
    }
    if (!fabric) {
        ESP_LOGD(kTag, "SRP service: no fabric yet, will retry on next Thread role change");
        return;
    }

    snprintf(s_srp.instanceName, sizeof(s_srp.instanceName),
             "%016llX-%016llX",
             (unsigned long long)fabric->GetCompressedFabricId(),
             (unsigned long long)fabric->GetNodeId());

    static const uint8_t kSII[] = "5000";
    static const uint8_t kSAI[] = "300";
    s_srp.txt[0].mKey         = "SII";
    s_srp.txt[0].mValue       = kSII;
    s_srp.txt[0].mValueLength = 4;
    s_srp.txt[1].mKey         = "SAI";
    s_srp.txt[1].mValue       = kSAI;
    s_srp.txt[1].mValueLength = 3;

    s_srp.svc = {};
    s_srp.svc.mInstanceName  = s_srp.instanceName;
    s_srp.svc.mName          = "_matter._tcp";
    s_srp.svc.mSubTypeLabels = nullptr;
    s_srp.svc.mTxtEntries    = s_srp.txt;
    s_srp.svc.mNumTxtEntries = 2;
    s_srp.svc.mPort          = 5540;
    s_srp.svc.mPriority      = 0;
    s_srp.svc.mWeight        = 0;

    otError err = otSrpClientAddService(ot, &s_srp.svc);
    if (err == OT_ERROR_NONE || err == OT_ERROR_ALREADY) {
        s_srp.added = true;
        ESP_LOGI(kTag, "SRP: queued _matter._tcp service as '%s'", s_srp.instanceName);
    } else {
        ESP_LOGE(kTag, "SRP: otSrpClientAddService => %d (will retry)", (int)err);
    }
}

void srpServiceRemove(otInstance *ot)
{
    if (!s_srp.added) return;

    bool hasFabric = false;
    for (const auto &f : chip::Server::GetInstance().GetFabricTable()) {
        (void)f;
        hasFabric = true;
        break;
    }
    if (hasFabric) {
        ESP_LOGI(kTag, "SRP: kFabricRemoved with committed fabric still present, keeping SRP record");
        return;
    }

    s_srp.added = false;
    otSrpClientClearHostAndServices(ot);
    ESP_LOGI(kTag, "SRP: cleared _matter._tcp service '%s' (decommissioned)", s_srp.instanceName);
}

class SrpFabricDelegate : public chip::FabricTable::Delegate
{
public:
    void OnFabricCommitted(const chip::FabricTable &, chip::FabricIndex fabricIndex) override
    {
        otInstance *ot = esp_openthread_get_instance();
        if (!ot) return;

        if (s_srp.added) {
            s_srp.added = false;
            otSrpClientClearHostAndServices(ot);
            ESP_LOGI(kTag, "SRP: cleared old record for fabric update");
        }

        setupSrpHost(ot);
        trySrpServiceAdd(ot, fabricIndex);
    }
};

static SrpFabricDelegate s_fabricDelegate;

void onThreadStateChanged(uint32_t flags, void *ctx)
{
    // Trigger on role change OR mesh-local address assignment.
    // After an NVS-restored reboot the device may re-join with its previous
    // role unchanged, so OT_CHANGED_THREAD_ROLE never fires; however the
    // mesh-local address is always (re-)assigned, making ML_ADDR the
    // reliable signal that Thread is operational and SRP can be started.
    const uint32_t kTrigger = OT_CHANGED_THREAD_ROLE | OT_CHANGED_THREAD_ML_ADDR;
    if (!(flags & kTrigger)) return;
    auto *ot = static_cast<otInstance *>(ctx);
    chip::DeviceLayer::PlatformMgr().ScheduleWork(
        [](intptr_t p) { trySrpServiceAdd(reinterpret_cast<otInstance *>(p)); },
        reinterpret_cast<intptr_t>(ot));
}

} // anonymous namespace

void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    (void)arg;

    namespace DevEvt = chip::DeviceLayer::DeviceEventType;

    switch (event->Type) {
    case DevEvt::kCommissioningComplete:
        ESP_LOGI(kTag, "Commissioning complete; joined operational fabric");
        shutdownBLE();
        break;

    case DevEvt::kFabricRemoved:
        chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
            otInstance *ot = esp_openthread_get_instance();
            if (ot) srpServiceRemove(ot);
        }, 0);
        ESP_LOGW(kTag, "Fabric removed event (decommission or FailSafe revert)");
        break;

    case DevEvt::kCHIPoBLEAdvertisingChange:
        if (event->CHIPoBLEAdvertisingChange.Result == chip::DeviceLayer::kActivity_Started) {
            ESP_LOGI(kTag, "BLE commissioning window opened");
            printCommissioningCodes();
        } else {
            ESP_LOGI(kTag, "BLE commissioning window closed");
        }
        break;

    default:
        break;
    }
}

} // namespace

extern "C" void app_main()
{
    // Factory reset via BOOT button (GPIO9, active-low).
    // Hold for 5 s at startup to clear fabric/config state and reboot.
    // chip-factory namespace is preserved so device can re-commission.
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_9, GPIO_PULLUP_ONLY);
    if (gpio_get_level(GPIO_NUM_9) == 0) {
        ESP_LOGW(kTag, "BOOT held — factory reset in 5 s, release to cancel");
        for (int i = 5; i > 0; --i) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (gpio_get_level(GPIO_NUM_9) != 0) {
                ESP_LOGI(kTag, "Factory reset cancelled");
                goto skip_factory_reset;
            }
            ESP_LOGW(kTag, "Factory reset in %d s…", i - 1);
        }
        ESP_LOGW(kTag, "Clearing fabric/config state — preserving chip-factory");
        nvs_flash_init_partition("nvs_matter");
        static const char *kClearNs[] = { "chip-config", "chip-counters" };
        for (const char *ns : kClearNs) {
            nvs_handle_t h;
            if (nvs_open_from_partition("nvs_matter", ns, NVS_READWRITE, &h) == ESP_OK) {
                nvs_erase_all(h);
                nvs_commit(h);
                nvs_close(h);
                ESP_LOGI(kTag, "Factory reset: cleared '%s'", ns);
            }
        }
        esp_restart();
    }
    skip_factory_reset:

    // General NVS: auto-erase on corruption (no Matter state here).
    esp_err_t nvs_err = nvs_flash_init_partition("nvs");
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(kTag, "General NVS unrecoverable (%s), erasing", esp_err_to_name(nvs_err));
        ESP_ERROR_CHECK(nvs_flash_erase_partition("nvs"));
        nvs_err = nvs_flash_init_partition("nvs");
    }
    ESP_ERROR_CHECK(nvs_err);

    // Matter credentials NVS: NEVER auto-erase — losing this requires full re-flash.
    esp_err_t nvs_matter_err = nvs_flash_init_partition("nvs_matter");
    if (nvs_matter_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_matter_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(kTag, "Matter NVS unrecoverable (%s) — manual factory reset required",
                 esp_err_to_name(nvs_matter_err));
        abort();
    }
    ESP_ERROR_CHECK(nvs_matter_err);

    // Probe the ICS 2 for its actual zone count before creating Matter endpoints.
    // The UART is initialised here; lk_ics2_init() (called after Matter starts)
    // will reuse it.  Falls back to NVS-cached count then CONFIG default if the
    // ICS 2 is unreachable at boot time.
    lk_ics2_config_t lk_cfg = {
        .uart_port           = CONFIG_LK_ICS2_UART_PORT,
        .tx_pin              = CONFIG_LK_ICS2_UART_TX_PIN,
        .rx_pin              = CONFIG_LK_ICS2_UART_RX_PIN,
        .de_pin              = CONFIG_LK_ICS2_RS485_DE_PIN,
        .baud_rate           = CONFIG_LK_ICS2_MODBUS_BAUD_RATE,
        .slave_addr          = (uint8_t)CONFIG_LK_ICS2_MODBUS_SLAVE_ADDR,
        .num_zones           = (uint8_t)CONFIG_LK_ICS2_NUM_ZONES,
        .poll_interval_ms    = CONFIG_LK_ICS2_POLL_INTERVAL_MS,
        .response_timeout_ms = CONFIG_LK_ICS2_MODBUS_TIMEOUT_MS,
        .update_cb           = lk_zone_update_cb,
        .update_cb_ctx       = nullptr,
    };
    uint8_t num_zones = CONFIG_LK_ICS2_NUM_ZONES;
    lk_ics2_probe_num_zones(&lk_cfg, &num_zones);
    lk_cfg.num_zones = num_zones;

    // Create Matter node with one thermostat endpoint per heating zone.
    // The attribute callback (setpoint/mode writes from HA → ICS 2) is
    // registered here via matter_device_init().
    ESP_ERROR_CHECK(matter_device_init(num_zones));

    // OpenThread native radio (802.15.4 built into ESP32-C6).
    static esp_openthread_platform_config_t ot_platform_config = {
        .radio_config = {
            .radio_mode = RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = HOST_CONNECTION_MODE_NONE,
        },
        .port_config = {
            .storage_partition_name = "nvs",
            .netif_queue_size       = 10,
            .task_queue_size        = 10,
        },
    };
    ESP_ERROR_CHECK(set_openthread_platform_config(&ot_platform_config));

    ESP_LOGI(kTag, "Starting Matter stack (BLE commissioning + Thread FTD)");
    ESP_ERROR_CHECK(start(app_event_cb));

    // Start the LK ICS 2 Modbus poll task now that the CHIP task is running
    // and ScheduleWork() is available for the zone-update callback.
    // lk_cfg was built and probed before Matter init above.
    ESP_ERROR_CHECK(lk_ics2_init(&lk_cfg));
    ESP_LOGI(kTag, "LK ICS 2 Modbus driver started (%d zones, slave=%d)",
             lk_cfg.num_zones, lk_cfg.slave_addr);

    // SRP work-around: the CHIP SDK's ESP32 DNS-SD layer uses esp-idf mDNS,
    // which fails when WiFi is disabled.  Manually manage _matter._tcp via the
    // OpenThread SRP client so the OTBR can proxy the record to mDNS and the
    // commissioner can open a CASE session after commissioning completes.
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
        otInstance *instance = esp_openthread_get_instance();
        if (instance == nullptr) {
            ESP_LOGW(kTag, "SRP setup: OpenThread instance unavailable");
            return;
        }

        setupSrpHost(instance);
        ESP_LOGI(kTag, "SRP: hostname '%s', auto-address enabled", s_srp.hostname);

        chip::Server::GetInstance().GetFabricTable().AddFabricDelegate(&s_fabricDelegate);
        otSetStateChangedCallback(instance, onThreadStateChanged, instance);
        trySrpServiceAdd(instance);
    }, 0);

    // Release app_main stack — Matter and LK ICS 2 poll tasks run independently.
    vTaskDelete(nullptr);
}

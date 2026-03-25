# threadics — LK ICS 2 Matter/Thread Bridge

An ESP32-C6 firmware that bridges an **LK Systems ICS 2** floor heating
controller to **Matter over Thread**, making all zones natively available
in **Home Assistant** (and any other Matter controller).

```
[LK ICS 2] ──RS485──> [ESP32-C6] ──Thread──> [Thread Border Router] ──> [Home Assistant]
```

---

## Features

- Up to **12 independent heating zones** exposed as Matter Thermostat devices
- Reads **room temperature**, **floor temperature** and **actuator position** per zone
- Writes **setpoint** and **mode** (Off / Heat / Cool / Auto) back to the ICS 2
- Native **Matter 1.3** over **Thread** — no cloud, no custom integrations
- OTA firmware update support
- Commissioning via **BLE** (standard Matter QR code / PIN flow)

---

## Hardware

### Bill of Materials

| Component | Notes |
|-----------|-------|
| ESP32-C6 development board | Any board with exposed GPIO. Tested on ESP32-C6-DevKitC-1 |
| RS485 half-duplex transceiver | e.g. MAX485, SN75176, THVD1500. 3.3 V compatible |
| 120 Ω termination resistor | Solder across A/B at the far end of the RS485 bus |
| 5 V / 1 A power supply | For ESP32-C6 board |
| Dupont jumper wires | For prototyping |

### Wiring

```
ESP32-C6 GPIO         MAX485 / RS485 module         LK ICS 2
─────────────────     ──────────────────────         ──────────────────────
GPIO 6  (TX / DI) ──> DI (Data In)
GPIO 7  (RX / RO) <── RO (Receiver Out)
GPIO 10 (DE)      ──> DE + /RE (tied together)
GND               ──> GND
3.3 V             ──> VCC
                       A  ──────────────────────>   RS485-A (terminal)
                       B  ──────────────────────>   RS485-B (terminal)
```

> **Note:** The LK ICS 2 RS485 terminals are labelled on the controller PCB.
> Consult the ICS 2 installation manual for the exact terminal numbers.
> Add a 120 Ω resistor between A and B at the ICS 2 end if the cable is
> longer than ~5 m.

Default GPIO assignments (configurable via `menuconfig`):

| Signal | Default GPIO |
|--------|-------------|
| UART TX (→ DI) | GPIO 6 |
| UART RX (← RO) | GPIO 7 |
| RS485 DE/RE | GPIO 10 |

---

## Software Prerequisites

1. **ESP-IDF v5.3** or later
   ```bash
   git clone --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
   cd ~/esp/esp-idf && git checkout v5.3
   ./install.sh esp32c6
   source export.sh
   ```

2. **esp-matter** (Espressif's Matter SDK)
   ```bash
   git clone --recursive https://github.com/espressif/esp-matter.git ~/esp/esp-matter
   cd ~/esp/esp-matter
   ./install.sh
   source ./export.sh
   ```

3. **Thread Border Router** — required for Matter over Thread connectivity.
   - Recommended: [Home Assistant Thread integration](https://www.home-assistant.io/integrations/thread/)
     with a compatible USB dongle (e.g. Silicon Labs EFR32MG21 based).
   - Alternative: Raspberry Pi + OpenThread Border Router Docker image.

---

## Build & Flash

```bash
# 1. Clone the repository
git clone https://github.com/guvhas/threadics.git
cd threadics

# 2. Set the target
idf.py set-target esp32c6

# 3. Configure (set GPIO pins, zone count, Modbus address, etc.)
idf.py menuconfig
#  -> LK ICS2 Matter Bridge Configuration

# 4. Build
idf.py build

# 5. Flash (replace /dev/ttyUSB0 with your serial port)
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Configuration

All options are available through `idf.py menuconfig` under
**"LK ICS2 Matter Bridge Configuration"**:

| Option | Default | Description |
|--------|---------|-------------|
| `LK_ICS2_UART_PORT` | 1 | UART port (use 1 on ESP32-C6, UART0 is the console) |
| `LK_ICS2_UART_TX_PIN` | 6 | GPIO for RS485 TX |
| `LK_ICS2_UART_RX_PIN` | 7 | GPIO for RS485 RX |
| `LK_ICS2_RS485_DE_PIN` | 10 | GPIO for RS485 DE/RE |
| `LK_ICS2_MODBUS_BAUD_RATE` | 9600 | Modbus baud rate |
| `LK_ICS2_MODBUS_SLAVE_ADDR` | 1 | ICS 2 Modbus slave address |
| `LK_ICS2_NUM_ZONES` | 6 | Number of zones to expose (1–12) |
| `LK_ICS2_POLL_INTERVAL_MS` | 5000 | How often to read the ICS 2 (ms) |
| `MATTER_DISCRIMINATOR` | 0xF00 | 12-bit Matter discriminator |
| `MATTER_PASSCODE` | 20202021 | Matter setup passcode |

---

## LK ICS 2 Modbus Register Map

The firmware uses Modbus RTU over RS485. The register layout used:

### Input Registers (FC=04, read-only)

| Address | Content | Unit |
|---------|---------|------|
| `0x0000 + zone` | Room temperature | 0.1 °C (signed) |
| `0x0010 + zone` | Floor temperature | 0.1 °C (signed) |
| `0x0020 + zone` | Actuator position | % (0–100) |
| `0x0030` | Controller firmware version | — |
| `0x0031` | Number of configured zones | — |

### Holding Registers (FC=03 read / FC=06 write)

| Address | Content | Unit |
|---------|---------|------|
| `0x0100 + zone` | Heating setpoint | 0.1 °C (signed) |
| `0x0110 + zone` | Zone mode (0=Off,1=Heat,2=Cool,3=Auto) | — |
| `0x0120` | Global system mode | — |

> **Important:** These addresses match common LK Systems Modbus firmware.
> If your ICS 2 uses a different register map, update `lk_ics2.h` (the
> `LK_REG_*` defines) to match the values in your ICS 2 Modbus manual.

---

## Commissioning into Home Assistant

### Step 1 — Prerequisites

- Home Assistant 2023.1 or later with the **Matter** integration enabled
- A **Thread Border Router** paired to HA (Settings → System → Matter/Thread)

### Step 2 — Power on the bridge

Power the ESP32-C6. It will:
1. Start the RS485 driver and begin polling the ICS 2
2. Open a BLE commissioning window
3. Print the QR code payload to the serial console

```
I (1234) chip[SVR]: Manual pairing code: 3497-989-5049
I (1234) chip[SVR]: SetupQRCode:         MT:Y3.13OTB00KA0648G00
```

### Step 3 — Add device in Home Assistant

1. In HA, go to **Settings → Devices & Services → Add Integration → Matter**
2. Choose **Add Matter device**
3. Scan the QR code shown in the serial console, or enter the manual code
4. HA will commission the device onto your Thread network

### Step 4 — Verify

After commissioning, HA creates one **Climate entity** per zone:
- `climate.lk_ics2_zone_1` … `climate.lk_ics2_zone_N`

Each entity exposes:
- Current temperature (room sensor)
- Target temperature (setpoint)
- HVAC mode (Off / Heat / Cool / Auto)

---

## Home Assistant Automation Example

```yaml
# Automatically lower setpoint at night
automation:
  - alias: "Floor heating night setback"
    trigger:
      - platform: time
        at: "22:00:00"
    action:
      - service: climate.set_temperature
        target:
          entity_id: climate.lk_ics2_zone_1
        data:
          temperature: 18
      - service: climate.set_hvac_mode
        target:
          entity_id: climate.lk_ics2_zone_1
        data:
          hvac_mode: heat
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     ESP32-C6                            │
│                                                         │
│  ┌──────────────┐    ┌────────────────────────────────┐ │
│  │ lk_ics2.cpp  │    │       matter_device.cpp        │ │
│  │              │    │                                │ │
│  │ Modbus RTU   │───>│ Matter Thermostat Endpoints    │ │
│  │ poll task    │    │ (one per zone, ep 1..N)        │ │
│  │ (5 s period) │    │                                │ │
│  │              │<───│ Attribute write callback       │ │
│  │ UART1/RS485  │    │ (setpoint / mode → Modbus)     │ │
│  └──────┬───────┘    └────────────────────────────────┘ │
│         │                         │                      │
└─────────┼─────────────────────────┼──────────────────────┘
          │ RS485                   │ Thread (IEEE 802.15.4)
          │                         │
     ┌────▼────┐             ┌──────▼──────┐
     │ LK ICS 2│             │Thread Border│
     │ (Modbus │             │   Router    │
     │  slave) │             └──────┬──────┘
     └─────────┘                    │ IP
                              ┌─────▼──────┐
                              │    Home    │
                              │  Assistant │
                              └────────────┘
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `Modbus timeout` in logs | Wrong slave address, wiring, or baud rate | Check `menuconfig` and RS485 wiring; verify A/B polarity |
| `CRC mismatch` | Electrical noise or wrong baud rate | Add termination resistor; verify baud rate |
| Device not appearing in HA | Matter not commissioned | Run `idf.py monitor` and follow the QR code instructions |
| All zones show 0°C | Registers read correctly but ICS 2 returns 0 | Verify register map against your ICS 2 firmware version |
| HA shows "unavailable" | Thread network issue | Verify Thread Border Router is online and reachable |

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

## Contributing

Pull requests welcome. When changing the Modbus register map, please
document which ICS 2 firmware version it applies to.

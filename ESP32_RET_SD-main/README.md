# ESP32 CAN → MQTT Telemetry

CAN bus frames are decoded into named engineering values on the ESP32 and
published as batched JSON to an MQTT broker over Wi-Fi, via a 4G router module
that provides the internet uplink. No SD card, no SavvyCAN bridge.

Built on top of `ESP32_RET_SD` (Collin Kidder's A0RET lineage), with the SD
logging and SavvyCAN bridge stripped out. When you need a logger for
reverse-engineering a bus, flash the stock
[ESP32_RET_SD](https://github.com/MotorvateDIY/ESP32_RET_SD) instead — keeping
both halves in one binary meant carrying ~1,900 lines and 6 kB of RAM that this
firmware never executes.

---

## 0. Scope vs. the system block diagram

This firmware currently implements the **single-CAN + Wi-Fi + MQTT** path. The
block diagram for the installed logger is larger than that, and the gaps are
deliberate, not oversights:

| Block diagram element | Status |
| --- | --- |
| CAN1 → BCU / odometer / CAB500 | **implemented** |
| Energy consumption calculation | **implemented** |
| CAN2, CAN3 | **not supported on a classic ESP32** — see below |
| 4G module on UART1 | **not implemented** — firmware assumes Wi-Fi, see below |
| GPS (UART2) | not implemented |
| IMU (I²C) — MPU6500 | **implemented** |
| Temp + CT sensors via ADS1115 (I²C) | **implemented** |
| CT sensors on SPI | not implemented — use the ADS1115 channels instead |
| Key switch input | supported but disabled (`TELEM_KEY_SWITCH_PIN`) |
| GNSS / RTC real time | not implemented — timestamps are uptime-relative |

### The three-CAN problem

A classic ESP32 has **one** TWAI controller. It physically cannot drive CAN1,
CAN2 and CAN3. Options, roughly in order of effort:

1. **Put everything on one bus.** If the BCU, odometer and CAB500 already share
   a bus, nothing else is needed — this is the common case, and the diagram
   shows all three on CAN1 already.
2. **ESP32-S3 / C6** — two TWAI controllers. `esp32_can` exposes `CAN1`
   automatically on those parts; you would add a second drain in `loop()`.
3. **External MCP2515 or MCP2517FD on SPI** for the third bus. Note the block
   diagram already uses SPI for the CT sensors, so that bus would be shared.
4. **A different MCU.** An STM32 or a dual-MCU split if you genuinely need three
   independent buses at high load.

### The 4G module

The scoping conversation established the module presents a **Wi-Fi hotspot** that
the ESP32 joins as a station, and the firmware is built that way. The block
diagram instead shows it on **UART1**, which implies an AT-command modem
(SIM7600/A7670/EC200-class) rather than a router.

Those are completely different transports. If it is UART, `mqtt_manager` needs
`TinyGSM` to bring up the PPP context and `PubSubClient` runs over
`TinyGsmClient` instead of `WiFiClient` — a contained change, since everything
above the client object stays the same. **Confirm which it is before wiring.**

---

## 1. Build settings

| Setting | Value |
| --- | --- |
| Board | ESP32 Dev Module |
| **Partition Scheme** | **Minimal SPIFFS (1.9MB APP with OTA / 190KB SPIFFS)** |
| Flash Size | 4MB (32Mb) |
| Upload Speed | 921600 |
| Core Debug Level | None |

> **The partition scheme is not optional.** On the default 1.2 MB app partition
> this firmware occupies **99%** of the available flash, which leaves no room for
> an OTA image to be staged. With `min_spiffs` it sits at **66%** of a 1.9 MB
> partition, and OTA has the headroom it needs.

Verified with arduino-cli 1.2.0, ESP32 core 3.3.10:

```
1,287,128 bytes (65% of 1,966,080)   RAM 58,424 (17%)
```

### Sketch folder name

Arduino requires the folder name to match the `.ino` name. Rename
`ESP32_RET_SD-main` to **`ESP32_RET_SD`** or the IDE will refuse to open it.

---

## 2. Libraries

| Library | Version tested | Source |
| --- | --- | --- |
| `esp32_can` | 0.3.1 | https://github.com/collin80/esp32_can |
| `can_common` | 0.4.0 | https://github.com/collin80/can_common |
| `PubSubClient` | 2.8 | Library Manager (Nick O'Leary) |
| `ArduinoJson` | 7.4.3 | Library Manager (Benoit Blanchon) |

`WiFi`, `Preferences`, `HTTPUpdate`, `Update` and `SPI` ship with the ESP32 core.

> **Note on `esp32_can`:** the current version only declares `CAN1` on SoCs with
> two TWAI controllers. The classic ESP32 has one, which is why the stock GVRET
> sources no longer compile against it. This firmware only ever touches `CAN0`,
> so the problem does not arise.

---

## 3. Wiring

Nothing new is needed on the ESP32 side for the 4G link: the module presents a
Wi-Fi hotspot and the ESP32 joins it as a station.

### CAN transceiver (SN65HVD230 — 3.3 V, recommended)

| ESP32 | Transceiver | Notes |
| --- | --- | --- |
| GPIO 4 | `D` / `CTX` / TXD | CAN transmit |
| GPIO 5 | `R` / `CRX` / RXD | CAN receive |
| 3V3 | `VCC` | **3.3 V only** on the HVD230 |
| GND | `GND` | common ground |
| — | `Rs` | tie to GND for high-speed mode |
| — | `CANH` → OBD-II pin 6 | |
| — | `CANL` → OBD-II pin 14 | |

Using a **TJA1050 or MCP2551 instead?** Those are 5 V parts. Their RX output
swings to 5 V and will damage the ESP32 — put a divider or level shifter on the
RX line, or just use the SN65HVD230 / TJA1042 / SN65HVD232.

**Do not fit a 120 Ω termination resistor** when tapping an existing vehicle bus.
The bus is already terminated at both ends; adding a third resistor drops the
effective impedance and can stop the bus working. Many breakout boards ship with
termination fitted — check for it and remove it.

### Power

| OBD-II | Goes to |
| --- | --- |
| Pin 16 (+12 V battery) | 12 V → 5 V buck converter → ESP32 `VIN` |
| Pins 4 / 5 (ground) | buck GND → ESP32 `GND` |

Do not feed 12 V into `VIN` directly. OBD-II pin 16 is unswitched, so the board
stays powered with the ignition off — budget for that or add a switched feed.

### Other pins

| GPIO | Use |
| --- | --- |
| 21 | I²C SDA — MPU6500 + ADS1115 |
| 22 | I²C SCL — MPU6500 + ADS1115 |
| 2 | Built-in LED — toggles every 250 CAN frames as a bus-alive indicator |
| 5, 18, 19, 23 | Previously the SD card (VSPI). **Unused — free for your own use.** |

### I²C sensors

Both parts share the bus at 400 kHz. Fit **4.7 kΩ pull-ups to 3.3 V** on SDA and
SCL if your breakout boards don't already have them — most MPU6500 modules do,
most bare ADS1115 boards do too, but two sets in parallel is fine.

| Part | Address | Set by |
| --- | --- | --- |
| MPU6500 | `0x68` | AD0 low (`0x69` if high) |
| ADS1115 | `0x48` | ADDR→GND (`0x49` VDD, `0x4A` SDA, `0x4B` SCL) |

Power both from **3.3 V**. The MPU6500 is not 5 V tolerant.

> **ADS1115 input limit:** no input may exceed VDD + 0.3 V — about 3.6 V on a
> 3.3 V rail — regardless of the PGA range you select. The ±6.144 V and
> ±4.096 V settings do not raise that ceiling; they only reduce resolution.
> Anything above 3.3 V needs a divider in front of the pin.

Avoid GPIO 6–11 (SPI flash). GPIO 0, 2, 12 and 15 are strapping pins — do not
pull them at boot.

---

## 4. Configuration

Everything is settable at runtime over USB serial at **115200 baud**. Type
`help`. Settings persist in NVS (namespace `telem`), so credentials survive a
reflash and are not baked into the binary.

```
set ssid   MyRouter_4G
set pass   hunter2
set host   192.168.1.100
set port   1883
set topic  vehicle
set interval 1000
save
reboot
```

Other commands: `show`, `status`, `signals`, `trip`, `imu`, `adc`, `i2cscan`,
`imucal`, `counters`, `defaults`, `ota [url]`, `reboot`.

Compile-time defaults live in `config.h`.

---

## 5. Defining your signals

`signal_db.cpp` holds the dictionary. The rows shipped are **placeholders** —
replace them with your own. Nothing else needs editing; the decoder, the JSON
payload and the `signals` console command all derive from this table.

```c
//   name        id     ext    start  len  endian        signed  scale   offset  unit
{ "soc",       0x100, false,   7,    16,  SIG_MOTOROLA, false,  0.01f,  0.0f,   "%"    },
{ "packVolt",  0x101, false,   7,    16,  SIG_MOTOROLA, false,  0.1f,   0.0f,   "V"    },
{ "hvCurr",    0x3C0, false,   7,    32,  SIG_MOTOROLA, false,  0.001f, -2147483.648f, "A" },
```

### Names the energy meter depends on

`energy_meter.cpp` looks these up **by name** at boot. Keep the spellings exactly
or the consumption calculation silently finds nothing and reports zero — the
console prints a specific error at startup for each one that is missing.

| Name | Required? | Used for |
| --- | --- | --- |
| `packVolt` | **yes** | power = V × I |
| `hvCurr` *or* `packCurr` | **yes** (one of them) | power = V × I |
| `odo` *or* `speed` | **yes** (one of them) | distance, and therefore Wh/km |
| `soc`, `soh` | no | reported, not used in the maths |

Everything else in the table is published but not otherwise interpreted, so add
cell voltages, temperatures and status bits freely.

`startBit` uses standard DBC numbering, so values can be lifted straight out of a
`.dbc` file or SavvyCAN:

- **`SIG_INTEL`** (little-endian) — `startBit` is the signal's *least* significant bit.
- **`SIG_MOTOROLA`** (big-endian) — `startBit` is the *most* significant bit, in
  the usual sawtooth numbering where bit 7 is the MSB of byte 0.

Physical value = `raw × scale + offset`, with two's-complement sign extension
applied before scaling when `signed` is true.

Use `signals` on the console to watch live decoded values and confirm your bit
positions before pointing the device at a broker.

---

## 6. MQTT interface

| Topic | Direction | Payload |
| --- | --- | --- |
| `<base>/<device>/data` | publish | batched signal values |
| `<base>/<device>/status` | publish, retained | birth / last-will / diagnostics |
| `<base>/<device>/cmd` | subscribe | commands |

`<device>` is derived from the Wi-Fi MAC, e.g. `canlog-3C71BF`.

**Data payload:**

```json
{"dev":"canlog-3C71BF","seq":412,"up":1837,
 "sig":{"soc":64.5,"soh":97.2,"packVolt":392.4,"hvCurr":-88.1,"speed":63.2},
 "energy":{"kW":-34.57,"km":42.183,"whOut":7104.2,"whIn":612.8,
           "whPerKm":153.9,"recent":148.2,"regenPc":8.6,"tripS":2714,
           "lifeKm":1832.6,"lifeWhKm":161.4,"ok":true},
 "imu":{"ax":0.021,"ay":-0.135,"az":0.991,"gx":0.12,"gy":-0.34,"gz":2.81,
        "pitch":2.14,"roll":-0.87,"grade":3.74,"temp":38.6,"ok":true},
 "adc":{"auxTemp":24.8,"ctA":12.35,"ctB":0.42}}
```

`sig` is measured off the CAN bus, `imu` and `adc` are measured from the I²C
sensors, and `energy` is derived on the device. They are kept in separate objects
so the server can always tell which is which. `energy.ok` goes false when the
voltage or current input has gone stale, meaning the figures are being held
rather than updated.

The `imu` and `adc` objects are **omitted entirely** when the part isn't
detected, rather than sent as zeroes your server would have to learn to ignore.
An ADC channel with an empty name in the table is skipped the same way.

Only signals that received a fresh frame since the last batch are included. Every
30 s a heartbeat batch carries the full set plus `"hb":true` and `"rssi"`, so the
server can distinguish an idle bus from a dead device. Set `set all 1` to include
every signal in every batch.

**Commands:**

```json
{"cmd":"status"}
{"cmd":"reboot"}
{"cmd":"interval","ms":500}
{"cmd":"trip_reset"}
{"cmd":"ota"}
{"cmd":"ota","url":"http://192.168.1.100/firmware/canlog.bin"}
```

---

## 6a. Energy meter

Power is sampled every 100 ms and integrated. Discharge and regeneration are
accumulated **separately** rather than as a single signed total — net energy
tells you range, but the regen-to-discharge ratio tells you how the car is being
driven and whether recuperation is working.

| Field | Meaning |
| --- | --- |
| `kW` | instantaneous power, positive = discharge |
| `whOut` / `whIn` | trip energy drawn from / returned to the pack |
| `whPerKm` | **average consumption rate** — net Wh ÷ trip km |
| `recent` | rolling Wh/km over the last 1 km |
| `regenPc` | regen as a share of gross discharge |
| `lifeKm` / `lifeWhKm` | lifetime distance and average, persisted in NVS |

Console: `trip` shows all of it, `trip reset` zeroes the trip, `trip resetlife`
zeroes the lifetime totals.

### Three things to verify before trusting the numbers

**1. Current polarity.** `set cursign 1` or `-1` decides which direction counts
as discharge. Get it backwards and consumption and regen swap places. Check it
on the bench: drive gently forward and confirm `trip` shows **positive** kW.

**2. Current source.** `set hvcurr 1` integrates the CAB500 transducer, `0` uses
the BMS's own `packCurr` estimate. Prefer the CAB500 — sensor accuracy goes
straight into your Wh/km figure, and a BMS current estimate is often coarse.

**3. Distance source.** `set useodo 1` uses odometer deltas, `0` integrates
speed. Odometer is better: integrating speed accumulates error and misses
whatever happened between samples. Deltas that are negative or larger than 5 km
in one 100 ms tick are rejected as counter resets or decode errors.

Integration stops when voltage or current has not been refreshed for 2 s, so a
sensor dropping off the bus freezes the totals instead of quietly accumulating a
stale value forever. Distance keeps accruing in that case, and `ok` goes false.

Lifetime totals are written to NVS once a minute and on key-off.

---

## 6b. I²C sensors

### MPU6500 IMU

Sampled at 50 Hz with a 41 Hz internal low-pass, which rejects engine and road
vibration while passing real vehicle motion (all below a few Hz). Defaults are
±4 g and ±500 °/s — a car rarely exceeds 1 g or a few tens of °/s, so this keeps
resolution high with headroom for potholes.

Run **`imucal`** once with the vehicle completely stationary. It averages 512
samples and stores the gyro zero in NVS, so it survives reflashing. Without it
the gyro reads a constant few °/s of bias and any integration drifts.

`i2cscan` is the first thing to run if a part doesn't appear — it separates
"wrong address" from "not wired / no pull-ups" in one command.

**Mounting convention** for the pitch/roll maths: **+X forward, +Y left, +Z up**.
Mounted differently, `ax`/`ay`/`az`/`gx`/`gy`/`gz` are still correct — only
`pitch`, `roll` and `grade` need their axes swapped in `imu_mpu6500.cpp`.

> **On `grade`:** road grade from a bare accelerometer is an *estimate*, not a
> measurement. The sensor cannot separate the gravity component of a slope from
> longitudinal acceleration — braking looks like an uphill. The heavy filter
> makes it usable over a sustained climb, but it will lie during hard
> acceleration. Treat it as context for consumption analysis, not truth. Fusing
> it with GPS altitude would fix this, which is one reason to wire up the GPS.

`temp` is the **die** temperature — the silicon, which runs well above ambient.
It's useful for spotting a sensor cooking in the sun, not for cabin temperature.

### ADS1115 ADC

Four single-ended channels, round-robined one at a time. A conversion at 128 SPS
takes ~8 ms; rather than stall the CAN loop for 32 ms per sweep, the driver is a
state machine — it starts a conversion, returns, and collects the result later.

Channels are defined in `adc_ads1115.cpp` in the same data-driven style as the
CAN signal table, with `value = volts × scale + offset`:

```c
//   name        gain              scale     offset   unit
{ "auxTemp", ADS_GAIN_4V096, 100.0f,   0.0f, "degC" },   // LM35, 10 mV/degC
{ "ctA",     ADS_GAIN_4V096, 100.0f,   0.0f, "A" },      // 0-1 V = 0-100 A
{ "",        ADS_GAIN_4V096,   1.0f,   0.0f, "V" },      // "" disables it
```

The shipped scales are **placeholders** — set them from your sensor datasheets.
A bidirectional CT centred on 1 V for ±100 A would be `scale 200, offset -100`.

---

## 7. OTA updates

Push-style `ArduinoOTA` cannot work here — the board sits behind carrier-grade
NAT with no inbound reachability and no mDNS across the cellular link. This
firmware **pulls** instead: it downloads a `.bin` over HTTP when told to, writes
it to the inactive OTA partition, verifies, and reboots.

1. `Sketch → Export Compiled Binary` in the Arduino IDE.
2. Put the `.bin` on any HTTP server.
3. Trigger it: `ota http://host/canlog.bin` on the console, or publish
   `{"cmd":"ota","url":"..."}` to the command topic.

A failed download leaves the running firmware untouched — the bootloader only
switches partitions after a complete, verified write.

---

## 8. How it handles bus load

A 500 kbit/s bus in a running vehicle produces roughly 2,000–4,000 frames per
second. Forwarding those one-for-one over cellular is not possible, so frames are
never queued for the network. Each frame is decoded on arrival and only the
**most recent value per signal** is kept; the publisher transmits that small
table on a timer.

The consequences are worth being explicit about:

- **Bandwidth is set by the publish interval, not by bus load.** A busier bus
  overwrites the same table more often and costs nothing extra on the SIM.
- **There is no queue to overflow.** The classic failure mode — a frame buffer
  that backs up during a network stall and then silently drops data — cannot
  occur.
- **Transient values between publishes are not transmitted.** If you need every
  occurrence of a fast-changing signal rather than a periodic sample, this
  architecture is the wrong one; you would want on-change publishing or an SD
  capture alongside it.

Threading: `loop()` runs on **core 1** and only drains CAN and services the
console. All networking — association, MQTT, OTA — runs in a task pinned to
**core 0**, alongside the Wi-Fi driver. A cellular link that stalls for seconds
therefore cannot delay CAN reception.

---

## 9. Safety

`TELEM_CAN_LISTEN_ONLY` defaults to **true**. In listen-only mode the controller
never transmits and never ACKs, so it cannot disturb the vehicle bus or push a
real ECU into a bus-off state. Leave it on unless you specifically need to
transmit, and think carefully before you do so on a moving vehicle.

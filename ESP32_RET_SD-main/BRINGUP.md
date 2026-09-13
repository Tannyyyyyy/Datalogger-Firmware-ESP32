# Bring-up: compile, flash, test

Work through these in order. Each phase has a **checkpoint** — don't move on
until it passes, because a failure later is much harder to diagnose when you
don't know which earlier stage was actually good.

Phases 1–5 need **no vehicle**. Do all of them on the bench first.

---

## Phase 0 — One-time setup

Already done on this machine, listed so you can reproduce it elsewhere.

**Libraries** (in `C:\Users\sawit\OneDrive\Documents\Arduino\libraries`):

| Library | Version | Source |
| --- | --- | --- |
| ArduinoJson | 7.4.3 | Library Manager |
| PubSubClient | 2.8 | Library Manager |
| esp32_can | 0.3.1 | `git clone https://github.com/collin80/esp32_can` |
| can_common | 0.4.0 | `git clone https://github.com/collin80/can_common` |

**ESP32 core:** 3.3.10, already installed.

### Rename the sketch folder

Arduino requires the folder name to match the `.ino` name. Rename:

```
C:\Users\sawit\Downloads\ESP32_RET_SD-main\ESP32_RET_SD-main\
                                        -> ...\ESP32_RET_SD\
```

Without this the IDE refuses to open the sketch. After renaming, the absolute
path inside `.vscode\c_cpp_properties.json` goes stale — IntelliSense still works
through the `${workspaceFolder}/**` entry, but regenerate it when convenient.

---

## Phase 1 — Compile

### Option A: Arduino IDE

**Tools menu — these must be set:**

| Setting | Value |
| --- | --- |
| Board | ESP32 Dev Module |
| **Partition Scheme** | **Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)** |
| Flash Size | 4MB (32Mb) |
| Upload Speed | 921600 |
| Core Debug Level | None |

The partition scheme is **not optional**. On the default 1.2 MB app partition
this firmware fills 99% of flash and OTA has nowhere to stage an image.

Then **Sketch → Verify/Compile**.

### Option B: command line (PowerShell)

```powershell
$acli = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
$sketch = "C:\Users\sawit\Downloads\ESP32_RET_SD-main\ESP32_RET_SD"
$fqbn = "esp32:esp32:esp32:PartitionScheme=min_spiffs"

& $acli compile --fqbn $fqbn $sketch
```

### ✅ Checkpoint 1

```
Sketch uses 1342108 bytes (68%) of program storage space. Maximum is 1966080 bytes.
Global variables use 66112 bytes (20%) of dynamic memory...
```

The **1966080** maximum is what tells you the partition scheme took. If it says
**1310720**, you're on the default partition — go back and fix it.

---

## Phase 2 — Flash

Connect the board by USB. Find the port:

```powershell
& $acli board list
```

Look for something like `COM5  Serial Port (USB)  ESP32 Dev Module`. If nothing
appears, you're missing the USB-serial driver — **CP210x** (Silicon Labs) or
**CH340**, depending on your board. Check Device Manager for a device with a
warning triangle.

```powershell
& $acli upload -p COM5 --fqbn $fqbn $sketch
```

Some boards can't auto-reset into the bootloader. If you see
`Failed to connect to ESP32: No serial data received`, hold **BOOT** (sometimes
labelled IO0), tap **EN**/**RST**, release **BOOT**, and retry immediately.

### ✅ Checkpoint 2

`Hard resetting via RTS pin...` and no error.

---

## Phase 3 — First boot, serial console

Open the monitor at **115200 baud**:

```powershell
& $acli monitor -p COM5 -c baudrate=115200
```

(Arduino IDE: **Tools → Serial Monitor**, set 115200, line ending "Newline".)

Press **EN/RST** to see the boot log from the start. Expect roughly:

```
ESP32_CAN_MQTT v2.0 on ESP32-D0WD-V3 rev 3

  current settings
  ------------------------------------------------------------
  device id     : canlog-3C71BF        <- note this, it's your MQTT topic
  wifi ssid     : 4G-ROUTER-SSID
  broker        : 192.168.1.100:1883
  ...

signal table ready: 14 signals
energy meter ready (lifetime 0.0 km, 0 Wh)
MPU6500 found at 0x68
ADS1115 ready at 0x48
  A0 -> auxTemp (+/-4.096 V, x100 degC)
  ...
CAN0 enabled at: 500000 bits/sec
connecting to Wi-Fi SSID '4G-ROUTER-SSID'

============================================================
  ESP32_CAN_MQTT v2.0 (telemetry build)
  CAN -> MQTT telemetry console
...
```

Type `help` and press Enter. If you see the menu, the console is alive.

### ✅ Checkpoint 3

The banner appears and `help` responds.

> **Nothing but garbage characters?** Wrong baud rate — it must be 115200.
> **Nothing at all?** Wrong COM port, or the monitor is still held open by
> another program (the IDE's Serial Monitor and `arduino-cli monitor` cannot both
> have the port).

---

## Phase 4 — I²C sensors

Still on the bench, no vehicle.

```
i2cscan
```

Expect:

```
  scanning I2C on SDA 21 / SCL 22 ...
    0x48  <- ADS1115
    0x68  <- MPU6500 (or an RTC)
  2 device(s) found
```

**Nothing found?** In order of likelihood: no 4.7 kΩ pull-ups to 3.3 V on SDA and
SCL; SDA and SCL swapped; the part isn't powered; wrong GPIOs. Fix this before
anything else — the scan is the cheapest possible test.

**Only one found?** Check that one's address pin. MPU6500 AD0 low = `0x68`;
ADS1115 ADDR to GND = `0x48`.

### Test the IMU

Lay the board **flat and level**, then:

```
imu
```

With +Z up you should see `az` close to **+1.000** and `ax`/`ay` near zero:

```
  accel (g)     : X  +0.012  Y  -0.008  Z  +0.998
  gyro (dps)    : X  +0.31   Y  +0.14   Z  -0.22
  pitch / roll  : -0.69 deg / -0.46 deg
```

Tilt the board nose-up and re-run — `pitch` should go positive. Roll it right
side down — `roll` goes positive. If the axes don't behave that way, your board
is mounted in a different orientation than the code assumes (+X forward, +Y left,
+Z up); the raw values are still correct, only pitch/roll/grade need remapping.

Now zero the gyro, **with the board completely still**:

```
imucal
```

```
averaging 512 samples - keep the vehicle completely still...
gyro bias = 0.312 / 0.145 / -0.221 dps (saved)
```

Re-run `imu` — the gyro axes should now sit near zero. This is stored in NVS and
survives reflashing, so you only do it once (or after remounting).

### Test the ADC

```
adc
```

```
  ch  name        volts       value       range
  A0  auxTemp     +0.24810 V    +24.810 degC  +/-4.096 V
  A1  ctA         +0.00012 V     +0.012 A     +/-4.096 V
```

Short a channel to GND — it should read ~0 V. Tie it to 3.3 V — about 3.3 V.
If a floating channel shows drifting noise, that's normal and expected; unused
channels should be disabled (empty name) or tied to GND.

### ✅ Checkpoint 4

Both devices found, `az ≈ +1 g` when level, gyro zeroed, ADC channels tracking
what you apply to them.

---

## Phase 5 — Network and MQTT

### Configure

```
set ssid   YourRouterSSID
set pass   YourRouterPassword
set host   192.168.1.100
set port   1883
set topic  vehicle
save
reboot
```

### Check the link

```
status
```

```
  wifi          : connected
  ip / rssi     : 192.168.1.57 / -54 dBm
  mqtt          : connected (state 0)
  data topic    : vehicle/canlog-3C71BF/data
  published     : 43 ok, 0 failed, 1 reconnects
```

MQTT `state 0` means connected. Other values worth knowing:

| state | Meaning |
| --- | --- |
| `-2` | TCP connect failed — wrong IP/port, or the broker isn't running |
| `-4` | Timeout — broker reachable but not answering |
| `4` | Bad username/password |
| `5` | Not authorised — broker ACL is rejecting the client |

### Watch the data

You have no MQTT client installed. Easiest options:

**MQTT Explorer** (GUI, recommended for a first look) — download from
mqtt-explorer.com, connect to your broker, and browse the `vehicle/` tree.

**Mosquitto CLI:**

```powershell
winget install EclipseFoundation.Mosquitto
mosquitto_sub -h 192.168.1.100 -t "vehicle/#" -v
```

**No broker yet?** Run one on your PC for testing — install Mosquitto as above,
then point the device at your PC's LAN IP (`ipconfig` to find it). You may need
`listener 1883` and `allow_anonymous true` in `mosquitto.conf`.

Expect a message every second:

```json
{"dev":"canlog-3C71BF","seq":12,"up":37,
 "energy":{"kW":0,"km":0,"whPerKm":0,"ok":false},
 "imu":{"ax":0.012,"az":0.998,"pitch":-0.69,"ok":true},
 "adc":{"auxTemp":24.81,"ctA":0.012}}
```

On the bench there's no `sig` object and `energy.ok` is `false` — correct, since
no CAN frames are arriving. The IMU and ADC data proves the whole pipeline works.

### ✅ Checkpoint 5

Messages arriving with live `imu` values that change when you move the board.

---

## Phase 6 — CAN

### On the bench

You need something to talk to. Options:

- A **USB-CAN adapter** (CANable, PCAN, etc.) sending frames at 500 kbit/s.
- A **second ESP32 + transceiver** transmitting test frames.
- Nothing — skip to the vehicle, but then Phase 6 and 7 happen at the same time,
  which makes a failure harder to place.

Wire CANH↔CANH, CANL↔CANL, and fit **one 120 Ω terminator at each end** of the
bench link (two total). This is the opposite of the vehicle case.

Send a frame matching an ID in your table, then:

```
counters
signals
```

`frames seen` should climb. `matched` climbs only when an ID in your table
arrives. If `seen` climbs but `matched` stays 0, the IDs in `signal_db.cpp` don't
match what's on the wire.

### In the vehicle

**Before you plug in:** confirm listen-only is on.

```
show
```

```
  can mode      : listen-only (silent)
```

In this mode the controller never transmits and never ACKs, so it cannot disturb
the vehicle bus or push a real ECU into a bus-off state. Leave it on.

Plug into OBD-II, ignition on, then:

```
counters
```

`frames seen` should climb fast — thousands per second on a live bus. The
built-in LED also toggles every 250 frames, so a steadily blinking LED means the
bus is alive.

**`frames seen` stays 0?** In order: CANH/CANL swapped; wrong bitrate (try 250000
with `set canspeed 250000`, `save`, `reboot`); transceiver not powered; you fitted
a 120 Ω terminator (don't — the vehicle bus is already terminated at both ends).

### ✅ Checkpoint 6

`frames seen` climbing.

---

## Phase 7 — Signals and the energy meter

```
signals
```

Every row shipped in `signal_db.cpp` is a **placeholder**. Expect most to say
`never seen` on a real vehicle until you replace them with your car's real IDs.

**To find the real ones:** flash the stock
[ESP32_RET_SD](https://github.com/MotorvateDIY/ESP32_RET_SD) and use the board as
a SavvyCAN / SD logger to capture and reverse-engineer the bus. Then put the IDs
and bit positions into `signal_db.cpp` and flash this firmware again.

Once `packVolt` and a current signal are live:

```
trip
```

```
  integrating   : yes
  pack          : 392.4 V   88.10 A   34.57 kW
```

### The three things to verify

**1. Polarity.** Drive gently forward and check `trip`. Power must read
**positive** while drawing from the pack. If it's negative, consumption and regen
are swapped:

```
set cursign -1
save
```

**2. Current source.** `set hvcurr 1` uses the CAB500 transducer, `0` uses the
BMS estimate. Prefer the CAB500 — its accuracy goes straight into your Wh/km.

**3. Distance.** After a few km, check that `trip distance` matches the car's own
trip odometer. If it doesn't, either the `odo` scale is wrong or you're falling
back to integrating speed (`set useodo 1` to force odometer deltas).

### ✅ Checkpoint 7

`whPerKm` shows a plausible figure. A passenger EV is typically **130–200
Wh/km**. Ten times that, or a negative number, means a scale or sign is wrong —
go back to the three checks above rather than trusting the number.

---

## Phase 8 — OTA

Test this **before** the device is somewhere inconvenient to reach.

1. Arduino IDE: **Sketch → Export Compiled Binary**, or from the command line the
   `.bin` is in your build folder.
2. Serve it over HTTP. Quickest:
   ```powershell
   cd <folder containing the .bin>
   python -m http.server 8080
   ```
3. Trigger it from the console:
   ```
   ota http://192.168.1.50:8080/ESP32_RET_SD.ino.bin
   ```
   or publish `{"cmd":"ota","url":"http://..."}` to `vehicle/<device>/cmd`.

Expect progress lines every 10%, then a reboot into the new firmware.

A failed download leaves the running firmware untouched — the bootloader only
switches partitions after a complete, verified write.

### ✅ Checkpoint 8

The board reboots and the banner shows your new build.

---

## Troubleshooting quick reference

| Symptom | Most likely cause |
| --- | --- |
| Compile: `Maximum is 1310720` | Wrong partition scheme |
| Upload: `No serial data received` | Hold BOOT, tap EN, release BOOT |
| Serial shows garbage | Baud isn't 115200 |
| `i2cscan` finds nothing | No pull-ups, or SDA/SCL swapped |
| `az` reads ~0 and `ax` ~1 | Board mounted on a different axis |
| MQTT state `-2` | Broker unreachable — wrong IP/port or not running |
| MQTT state `4`/`5` | Credentials or broker ACL |
| `frames seen` = 0 | CANH/CANL swapped, wrong bitrate, or you added a terminator |
| `matched` = 0 but `seen` climbing | Signal table IDs don't match this vehicle |
| `energy.ok` false | `packVolt` or current signal missing/stale |
| `whPerKm` wildly wrong | Scale or `cursign` wrong — see Phase 7 |
| Consumption negative while driving | `set cursign -1` |

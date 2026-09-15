/*
 * config.h
 *
 * Every user-tunable knob for the CAN -> MQTT telemetry firmware.
 *
 * These are COMPILE-TIME DEFAULTS. On first boot they are written into NVS
 * (Preferences, namespace "telem"); after that NVS wins and you change settings
 * at runtime over the USB serial console (type "help" at 115200 baud). Use
 * "defaults" on the console to wipe NVS back to the values below.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <Arduino.h>

#define CFG_VERSION "ESP32_CAN_MQTT v2.0"

// ---------------------------------------------------------------------------
// PINS
// ---------------------------------------------------------------------------
#define GPIO_CAN_TX     GPIO_NUM_4
#define GPIO_CAN_RX     GPIO_NUM_5

// Built-in LED, toggled every CAN_RX_LED_TOGGLE frames as an "the bus is alive"
// indication you can see through the case.
#define LED_PIN             2
#define CAN_RX_LED_TOGGLE   250

// I2C for the MPU6500 and ADS1115. GPIO 21/22 are the ESP32 defaults.
#define TELEM_I2C_SDA   21
#define TELEM_I2C_SCL   22
#define TELEM_I2C_HZ    400000   // both parts are rated for 400 kHz

// Optional ignition / key-switch input. A trip starts on key-on and the totals
// are flushed to NVS on key-off. -1 disables it and trips run purely on the
// console / MQTT command.
// NOTE: must be an isolated, level-shifted input - do NOT connect a 12 V
// key-switch line straight to a GPIO.
#define TELEM_KEY_SWITCH_PIN         -1
#define TELEM_KEY_SWITCH_ACTIVE_HIGH true

// ---------------------------------------------------------------------------
// NETWORK - the Wi-Fi hotspot published by your 4G router module
// ---------------------------------------------------------------------------
#define TELEM_WIFI_SSID     "4G-ROUTER-SSID"     // <-- CHANGE ME
#define TELEM_WIFI_PASS     "4G-ROUTER-PASSWORD" // <-- CHANGE ME

// How long to wait for an association before retrying (ms).
#define TELEM_WIFI_TIMEOUT_MS   15000

// ---------------------------------------------------------------------------
// MQTT BROKER (plain TCP, no TLS - as scoped)
// ---------------------------------------------------------------------------
#define TELEM_MQTT_HOST     "192.168.1.100"      // <-- CHANGE ME (IP or hostname)
#define TELEM_MQTT_PORT     1883
#define TELEM_MQTT_USER     ""                   // "" = anonymous
#define TELEM_MQTT_PASS     ""

// Base topic. The device id is appended automatically, giving e.g.
//   vehicle/canlog-3C71BF/data    <- batched signal payloads
//   vehicle/canlog-3C71BF/status  <- birth / last-will
//   vehicle/canlog-3C71BF/cmd     <- inbound commands (subscribed)
#define TELEM_BASE_TOPIC    "vehicle"

// Keep this comfortably below any NAT / idle timeout your carrier enforces,
// otherwise the socket dies silently.
#define TELEM_MQTT_KEEPALIVE    30

// PubSubClient defaults to 256 bytes, which one batch of signals overruns
// instantly - we raise it explicitly at startup.
#define TELEM_MQTT_BUFFER_SIZE  2048

// ---------------------------------------------------------------------------
// PUBLISH CADENCE
// ---------------------------------------------------------------------------
// One batched JSON message every N milliseconds. This is the single knob that
// determines your cellular data usage: bus load does NOT affect it, because we
// only ever transmit the latest decoded value per signal.
#define TELEM_PUBLISH_INTERVAL_MS   1000

// If true, every signal is in every batch. If false (default) only signals that
// received a fresh CAN frame since the last batch are included, which is
// materially cheaper on a metered SIM.
#define TELEM_PUBLISH_UNCHANGED     false

// Even with nothing changing, emit a batch at least this often so the server can
// tell the difference between "idle" and "dead".
#define TELEM_HEARTBEAT_MS          30000

// ---------------------------------------------------------------------------
// CAN
// ---------------------------------------------------------------------------
#define TELEM_CAN_SPEED         500000

// Listen-only (silent / passive) mode. STRONGLY recommended on a live vehicle:
// the controller never transmits, never ACKs, and therefore cannot disturb the
// bus or bus-off a real ECU.
#define TELEM_CAN_LISTEN_ONLY   true

// ---------------------------------------------------------------------------
// ENERGY METER (EV consumption accounting)
// ---------------------------------------------------------------------------
// How often power is integrated. 100 ms is far faster than the signals change
// and keeps integration error negligible, while costing almost nothing.
#define ENERGY_UPDATE_MS            100

// If the voltage or current signal has not refreshed within this window we stop
// integrating rather than keep applying a stale value. A sensor that drops off
// the bus must not quietly keep accumulating energy.
#define ENERGY_SIGNAL_TIMEOUT_MS    2000

// Below this trip distance the Wh/km average is meaningless, so it reads as 0.
#define ENERGY_MIN_TRIP_KM          0.05f

// Trip and lifetime totals are written to NVS this often, and on key-off. NVS
// has a finite erase budget, so do not make this aggressive.
#define ENERGY_NVS_SAVE_MS          60000

// Window for the "recent" consumption figure, i.e. rolling Wh/km over the last
// stretch of driving rather than the whole trip.
#define ENERGY_RECENT_WINDOW_KM     1.0f

// Defaults for the runtime-settable energy options (see struct Config).
#define ENERGY_USE_HV_CURRENT   true    // CAB500 transducer rather than the BMS
#define ENERGY_CURRENT_SIGN     1       // +1 if positive current = discharge
#define ENERGY_USE_ODOMETER     true    // odometer deltas rather than speed

// ---------------------------------------------------------------------------
// MPU6500 6-axis IMU
// ---------------------------------------------------------------------------
#define IMU_ENABLED         true
#define IMU_I2C_ADDR        0x68     // 0x69 if AD0 is tied high
#define IMU_SAMPLE_MS       20       // 50 Hz - plenty for vehicle dynamics

// A car rarely exceeds 1 g laterally or longitudinally, so +/-4 g keeps
// resolution high while leaving headroom for potholes. +/-500 dps covers any
// real yaw rate.
#define IMU_ACCEL_RANGE_G   4        // 2, 4, 8 or 16
#define IMU_GYRO_RANGE_DPS  500      // 250, 500, 1000 or 2000

// Digital low-pass filter. 41 Hz rejects engine / road vibration while passing
// real vehicle motion, which is all below a few Hz.
#define IMU_DLPF_CFG        3        // 0..6, 3 = 41 Hz

// Samples averaged when zeroing the gyro. The vehicle MUST be stationary.
#define IMU_CAL_SAMPLES     512

// Smoothing for the pitch/roll estimate. Closer to 1 is smoother but laggier.
#define IMU_TILT_FILTER     0.98f

// ---------------------------------------------------------------------------
// ADS1115 16-bit ADC
// ---------------------------------------------------------------------------
#define ADS_ENABLED         true
#define ADS_I2C_ADDR        0x48     // 0x49 / 0x4A / 0x4B via the ADDR pin

// Milliseconds between channel conversions. The driver round-robins one enabled
// channel at a time, so a full sweep of 4 channels takes 4x this.
#define ADS_SAMPLE_MS       100

// PGA full-scale range indices used by the channel table in adc_ads1115.cpp.
//
// IMPORTANT: no input may exceed VDD + 0.3 V regardless of the PGA setting. On a
// 3.3 V supply that caps you at ~3.6 V, so ranges 0 and 1 cannot actually be
// driven to full scale - they only reduce resolution. Use a divider above 3.3 V.
#define ADS_GAIN_6V144      0
#define ADS_GAIN_4V096      1
#define ADS_GAIN_2V048      2
#define ADS_GAIN_1V024      3
#define ADS_GAIN_0V512      4
#define ADS_GAIN_0V256      5

// ---------------------------------------------------------------------------
// OTA (pull-based, over HTTP)
// ---------------------------------------------------------------------------
// The board sits behind carrier-grade NAT, so push-style ArduinoOTA cannot reach
// it. Instead the firmware fetches an image itself, either on an MQTT command
// ({"cmd":"ota"}) or from this default URL.
#define TELEM_OTA_URL       "http://192.168.1.100/firmware/canlog.bin"

// ---------------------------------------------------------------------------
// RUNTIME SETTINGS (mirrored into NVS)
// ---------------------------------------------------------------------------
struct Config {
    char     wifiSSID[33];
    char     wifiPass[65];
    char     brokerHost[64];
    uint16_t brokerPort;
    char     mqttUser[33];
    char     mqttPass[65];
    char     baseTopic[48];
    uint16_t publishIntervalMs;
    uint32_t canSpeed;
    bool     canListenOnly;
    bool     publishUnchanged;
    char     otaUrl[160];

    // --- energy meter ---
    // Which signal feeds the integrator: true = "hvCurr" (the CAB500
    // transducer, more accurate), false = "packCurr" (the BMS estimate).
    bool     useHvCurrent;

    // Multiplier applied to the raw current so that POSITIVE means DISCHARGE.
    // Set to -1 if your sensor or BMS reports discharge as negative. Getting
    // this backwards inverts consumption and regen, so verify it on the bench:
    // drive gently and confirm "power" reads positive.
    int8_t   currentSign;

    // Prefer odometer deltas for distance. Falls back to integrating speed
    // automatically if the odometer signal never appears.
    bool     useOdometer;
};

extern Config cfg;

// Unique per-board id derived from the Wi-Fi MAC, e.g. "canlog-3C71BF".
extern char deviceId[24];

void cfgLoad();     // NVS -> cfg (seeds from the defaults above)
void cfgSave();     // cfg -> NVS
void cfgReset();    // wipe NVS namespace, reload compile-time defaults
void cfgPrint();    // dump current settings to Serial

#endif /* CONFIG_H_ */

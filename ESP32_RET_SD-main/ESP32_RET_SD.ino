/*
 * ESP32 CAN -> MQTT vehicle telemetry.
 *
 * Decodes CAN frames into named signals, accumulates EV energy/consumption, and
 * publishes a batched JSON payload over MQTT through a 4G router's Wi-Fi.
 *
 * Originally ESP32_RET_SD by MotorvateDIY, based on Collin Kidder's A0RET. The
 * SD-logging / SavvyCAN-bridge half of that firmware has been removed - flash the
 * stock ESP32_RET_SD (https://github.com/MotorvateDIY/ESP32_RET_SD) when you need
 * a logger for reverse-engineering a bus.
 *
 * CORE SPLIT
 * ----------
 * core 1 (this loop) : drain CAN, decode, integrate energy, read I2C sensors
 * core 0 (telem_net) : Wi-Fi, MQTT, OTA - see mqtt_manager.h
 *
 * Network calls block for seconds when a cellular link stalls; keeping them off
 * core 1 means CAN reception never pauses and frames are never missed.
 *
 * MIT licensed - see LICENSE.
 */

#include <Arduino.h>
#include <Wire.h>
#include "esp32_can.h"

#include "config.h"
#include "log.h"
#include "signal_db.h"
#include "energy_meter.h"
#include "imu_mpu6500.h"
#include "adc_ads1115.h"
#include "mqtt_manager.h"
#include "console.h"

// At 500 kbit/s a fully loaded bus delivers roughly 4 frames per millisecond, so
// 64 per pass leaves a wide margin while still yielding to the idle task.
static const int MAX_FRAMES_PER_PASS = 64;

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    Serial.printf("\n\n%s on %s rev %d\n", CFG_VERSION, ESP.getChipModel(), ESP.getChipRevision());

    // Broker / Wi-Fi / cadence settings, and the device id, come out of NVS
    // before anything else uses them.
    cfgLoad();
    cfgPrint();

    // Validate the signal dictionary up front. A bad row here would otherwise
    // surface as a nonsense value on a dashboard days later.
    if (!signalDB.begin()) {
        logError("signal table failed validation - fix signal_db.cpp and reflash");
    }

    // Resolves the signal indices the consumption maths needs and restores the
    // lifetime energy totals from NVS.
    energyMeter.begin();

    // I2C sensors. Both are optional - if a part does not answer we log it and
    // carry on, because losing the IMU should never stop CAN telemetry.
    Wire.begin(TELEM_I2C_SDA, TELEM_I2C_SCL, TELEM_I2C_HZ);
#if IMU_ENABLED
    imu.begin();
#endif
#if ADS_ENABLED
    adc.begin();
#endif

    CAN0.setCANPins(GPIO_CAN_RX, GPIO_CAN_TX);
    CAN0.enable();
    CAN0.begin(cfg.canSpeed, 255);
    CAN0.setListenOnlyMode(cfg.canListenOnly);
    CAN0.watchFor();
    logInfo("CAN0 at %lu bps, %s", (unsigned long)cfg.canSpeed,
            cfg.canListenOnly ? "listen-only" : "normal (will ACK)");

    // Let the CAN controller settle before the radio starts pulling current.
    delay(250);

    // Brings up Wi-Fi and spawns the MQTT/OTA task on core 0. Non-blocking.
    if (!mqttManager.begin()) {
        logError("telemetry network layer failed to start");
    }

    console.begin();
}

void loop()
{
    // --- CAN: empty the driver queue into the latest-value signal table ------
    // Nothing is buffered and nothing is queued for the network, so a busy bus
    // costs CPU but never memory. The pass is bounded so a saturated bus cannot
    // starve the console.
    static uint32_t frameCount = 0;
    CAN_FRAME incoming;

    for (int n = 0; n < MAX_FRAMES_PER_PASS && CAN0.available() > 0; n++) {
        CAN0.read(incoming);
        signalDB.decodeFrame(incoming);

        // Heartbeat on the built-in LED: one toggle per CAN_RX_LED_TOGGLE frames
        // is an at-a-glance "the bus is alive" indication through the case.
        if (++frameCount >= CAN_RX_LED_TOGGLE) {
            frameCount = 0;
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
    }

    // Integrate power into energy. Deliberately on this core, next to CAN, so
    // consumption accounting keeps running through network outages, broker
    // downtime and OTA downloads. Internally rate-limited to ENERGY_UPDATE_MS.
    energyMeter.update();

    // Both sensors are non-blocking and internally rate-limited: the IMU does one
    // 14-byte burst read every IMU_SAMPLE_MS, and the ADC advances a conversion
    // state machine rather than waiting out its ~8 ms conversion.
#if IMU_ENABLED
    imu.update();
#endif
#if ADS_ENABLED
    adc.update();
#endif

    // Line-oriented config console - type 'help' at 115200 baud.
    for (int n = 0; n < 32 && Serial.available() > 0; n++) {
        console.processByte((uint8_t)Serial.read());
    }

    // Yield a tick so the idle task runs and the task watchdog stays satisfied.
    vTaskDelay(1);
}

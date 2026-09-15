/*
 * mqtt_manager.cpp
 *
 * See mqtt_manager.h for the threading model.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "mqtt_manager.h"
#include "config.h"
#include "signal_db.h"
#include "energy_meter.h"
#include "imu_mpu6500.h"
#include "adc_ads1115.h"
#include "ota_manager.h"
#include "log.h"

MqttManager mqttManager;

// PubSubClient's callback is a bare function pointer with no user-data argument,
// so we need a file-scope handle back to the instance.
static MqttManager *s_instance = nullptr;

// Reconnect backoff bounds. Starting at one second keeps a brief blip cheap;
// capping at thirty stops a long outage from hammering the broker (and the
// cellular data plan) with connect attempts.
static const uint16_t BACKOFF_MIN_MS = 1000;
static const uint16_t BACKOFF_MAX_MS = 30000;

MqttManager::MqttManager()
    : _mqtt(_net),
      _mqttConnected(false),
      _paused(false),
      _publishCount(0),
      _publishFailures(0),
      _mqttReconnects(0),
      _lastPublishMs(0),
      _lastHeartbeatMs(0),
      _nextMqttAttemptMs(0),
      _backoffMs(BACKOFF_MIN_MS),
      _seq(0)
{
    _topicData[0] = _topicStatus[0] = _topicCmd[0] = '\0';
}

void MqttManager::buildTopics()
{
    snprintf(_topicData,   sizeof(_topicData),   "%s/%s/data",   cfg.baseTopic, deviceId);
    snprintf(_topicStatus, sizeof(_topicStatus), "%s/%s/status", cfg.baseTopic, deviceId);
    snprintf(_topicCmd,    sizeof(_topicCmd),    "%s/%s/cmd",    cfg.baseTopic, deviceId);
}

bool MqttManager::begin()
{
    s_instance = this;
    buildTopics();

    WiFi.persistent(false);          // don't wear out flash rewriting credentials
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);            // modem sleep adds latency and buys us little
                                     // on a permanently powered vehicle install

    _mqtt.setServer(cfg.brokerHost, cfg.brokerPort);
    _mqtt.setKeepAlive(TELEM_MQTT_KEEPALIVE);
    _mqtt.setCallback(mqttCallbackEntry);

    // Without this, PubSubClient silently drops any payload over 256 bytes.
    if (!_mqtt.setBufferSize(TELEM_MQTT_BUFFER_SIZE)) {
        logError("could not allocate a %u byte MQTT buffer", TELEM_MQTT_BUFFER_SIZE);
        return false;
    }

    logInfo("connecting to Wi-Fi SSID '%s'", cfg.wifiSSID);
    WiFi.begin(cfg.wifiSSID, cfg.wifiPass);

    // Pin the network task to core 0, alongside the Wi-Fi driver, leaving core 1
    // free for CAN. 6 kB of stack covers TLS-free MQTT plus the HTTP OTA client.
    const BaseType_t ok = xTaskCreatePinnedToCore(
        networkTaskEntry,
        "telem_net",
        6144,
        this,
        1,            // one below the Arduino loop task so CAN keeps priority
        nullptr,
        0);

    if (ok != pdPASS) {
        logError("failed to start the network task");
        return false;
    }

    return true;
}

bool MqttManager::isWifiConnected() const
{
    return WiFi.status() == WL_CONNECTED;
}

// ---------------------------------------------------------------------------
// Network task
// ---------------------------------------------------------------------------

void MqttManager::networkTaskEntry(void *arg)
{
    static_cast<MqttManager *>(arg)->networkTask();
}

void MqttManager::networkTask()
{
    for (;;) {
        ensureWifi();

        if (isWifiConnected()) {
            ensureMqtt();

            if (_mqtt.connected()) {
                _mqtt.loop();               // services keepalives and inbound cmds

                const uint32_t now = millis();
                if (!_paused && (now - _lastPublishMs >= cfg.publishIntervalMs)) {
                    _lastPublishMs = now;
                    publishBatch();
                }
            }
        }

        // 10 ms cadence is far finer than the publish interval and keeps the
        // task responsive to inbound commands without busy-waiting.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void MqttManager::ensureWifi()
{
    static uint32_t lastAttempt = 0;
    static bool wasConnected = false;

    if (isWifiConnected()) {
        if (!wasConnected) {
            wasConnected = true;
            logInfo("Wi-Fi connected, IP %s, RSSI %d dBm",
                         WiFi.localIP().toString().c_str(), WiFi.RSSI());
        }
        return;
    }

    if (wasConnected) {
        wasConnected = false;
        _mqttConnected = false;
        logWarn("Wi-Fi lost - the 4G router may have dropped or rebooted");
    }

    // WiFi.setAutoReconnect handles most cases, but a router that vanished for a
    // long stretch sometimes needs an explicit kick.
    const uint32_t now = millis();
    if (now - lastAttempt >= TELEM_WIFI_TIMEOUT_MS) {
        lastAttempt = now;
        logInfo("retrying Wi-Fi association...");
        WiFi.disconnect();
        WiFi.begin(cfg.wifiSSID, cfg.wifiPass);
    }
}

void MqttManager::ensureMqtt()
{
    if (_mqtt.connected()) {
        _mqttConnected = true;
        return;
    }

    _mqttConnected = false;

    const uint32_t now = millis();
    if (now < _nextMqttAttemptMs) return;

    logInfo("connecting to MQTT broker %s:%u as %s",
                 cfg.brokerHost, cfg.brokerPort, deviceId);

    // Last will: if we drop off without saying goodbye, the broker publishes
    // "offline" on our behalf so the server can tell a dead link from a quiet bus.
    const char *user = strlen(cfg.mqttUser) ? cfg.mqttUser : nullptr;
    const char *pass = strlen(cfg.mqttPass) ? cfg.mqttPass : nullptr;

    const bool ok = _mqtt.connect(
        deviceId,
        user, pass,
        _topicStatus,      // will topic
        1,                 // will QoS
        true,              // will retained
        "{\"state\":\"offline\"}");

    if (ok) {
        _mqttConnected = true;
        _mqttReconnects++;
        _backoffMs = BACKOFF_MIN_MS;

        _mqtt.subscribe(_topicCmd, 1);
        publishStatus("online");

        logInfo("MQTT connected. data=%s cmd=%s", _topicData, _topicCmd);
    } else {
        // rc -2 is a TCP-level failure, -4 timeout, 4/5 bad credentials.
        logWarn("MQTT connect failed, rc=%d - retrying in %u ms",
                     _mqtt.state(), _backoffMs);
        _nextMqttAttemptMs = now + _backoffMs;
        _backoffMs = (_backoffMs >= BACKOFF_MAX_MS / 2) ? BACKOFF_MAX_MS
                                                        : (uint16_t)(_backoffMs * 2);
    }
}

// ---------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------

void MqttManager::publishBatch()
{
    SignalValue snap[MAX_SIGNALS];
    const size_t n = signalDB.snapshot(snap, MAX_SIGNALS, true);
    if (n == 0) return;

    const uint32_t now = millis();
    const bool heartbeatDue = (now - _lastHeartbeatMs >= TELEM_HEARTBEAT_MS);

    JsonDocument doc;
    doc["dev"] = deviceId;
    doc["seq"] = _seq;
    doc["up"]  = now / 1000;          // uptime in seconds

    JsonObject sig = doc["sig"].to<JsonObject>();

    size_t included = 0;
    for (size_t i = 0; i < n; i++) {
        if (!snap[i].everSeen) continue;

        // Default policy: only send what actually changed since the last batch.
        // A heartbeat batch overrides that so the server sees a full picture
        // periodically even on a static bus.
        if (!cfg.publishUnchanged && !heartbeatDue &&
            snap[i].updatesSincePublish == 0) {
            continue;
        }

        const SignalDef *def = signalDB.definition(i);
        if (def == nullptr) continue;

        // Plain float assignment - ArduinoJson emits the shortest representation
        // that round-trips, which is both correct and the fewest bytes on the wire.
        sig[def->name] = snap[i].value;
        included++;
    }

    // No fresh signals and no heartbeat due - the bus is quiet or the car is
    // off. Stay silent rather than spending cellular data restating a static
    // picture. (The energy block is built after this check, not before, so a
    // suppressed batch costs nothing.)
    if (included == 0 && !heartbeatDue) return;

    // Derived energy figures. These are computed on core 1, not read off the
    // bus, so they go in their own object rather than being mixed in with raw
    // decoded signals - the server should be able to tell measured from derived.
    EnergySnapshot e;
    energyMeter.snapshot(e);

    JsonObject en = doc["energy"].to<JsonObject>();
    en["kW"]      = roundf(e.power * 100.0f) / 100.0f;
    en["km"]      = roundf(e.tripDistanceKm * 1000.0f) / 1000.0f;
    en["whOut"]   = roundf(e.tripEnergyOutWh * 10.0f) / 10.0f;
    en["whIn"]    = roundf(e.tripEnergyInWh * 10.0f) / 10.0f;
    en["whPerKm"] = roundf(e.tripAvgWhPerKm * 10.0f) / 10.0f;   // the headline figure
    en["recent"]  = roundf(e.recentWhPerKm * 10.0f) / 10.0f;
    en["regenPc"] = roundf(e.tripRegenPercent * 10.0f) / 10.0f;
    en["tripS"]   = e.tripSeconds;
    en["lifeKm"]  = roundf(e.lifeDistanceKm * 10.0f) / 10.0f;
    en["lifeWhKm"] = roundf(e.lifeAvgWhPerKm * 10.0f) / 10.0f;
    en["ok"]      = e.integrating;   // false = inputs stale, treat figures as held

#if IMU_ENABLED
    // Vehicle dynamics. Only published when the part is actually present, so a
    // board built without the IMU simply omits the object rather than sending
    // a block of zeroes the server would have to learn to ignore.
    if (imu.present()) {
        ImuSample m;
        imu.snapshot(m);
        JsonObject im = doc["imu"].to<JsonObject>();
        im["ax"]    = roundf(m.ax * 1000.0f) / 1000.0f;
        im["ay"]    = roundf(m.ay * 1000.0f) / 1000.0f;
        im["az"]    = roundf(m.az * 1000.0f) / 1000.0f;
        im["gx"]    = roundf(m.gx * 100.0f) / 100.0f;
        im["gy"]    = roundf(m.gy * 100.0f) / 100.0f;
        im["gz"]    = roundf(m.gz * 100.0f) / 100.0f;
        im["pitch"] = roundf(m.pitchDeg * 100.0f) / 100.0f;
        im["roll"]  = roundf(m.rollDeg * 100.0f) / 100.0f;
        im["grade"] = roundf(m.gradePercent * 100.0f) / 100.0f;
        im["temp"]  = roundf(m.tempC * 10.0f) / 10.0f;
        im["ok"]    = m.valid;
    }
#endif

#if ADS_ENABLED
    // Analog channels, keyed by the names in the ADC channel table so the
    // server sees "auxTemp" rather than "A0".
    if (adc.present()) {
        AdcSample a;
        adc.snapshot(a);
        JsonObject an = doc["adc"].to<JsonObject>();
        for (uint8_t c = 0; c < ADS_NUM_CHANNELS; c++) {
            const AdcChannelDef &d = ADC_CHANNELS[c];
            if (d.name == nullptr || d.name[0] == '\0') continue;
            if (!a.fresh[c]) continue;
            an[d.name] = roundf(a.value[c] * 1000.0f) / 1000.0f;
        }
    }
#endif

    if (heartbeatDue) {
        _lastHeartbeatMs = now;
        doc["hb"] = true;
        doc["rssi"] = WiFi.RSSI();
    }

    char payload[TELEM_MQTT_BUFFER_SIZE];
    const size_t len = serializeJson(doc, payload, sizeof(payload));

    if (len == 0 || len >= sizeof(payload)) {
        // Truncation would produce invalid JSON, so drop the batch rather than
        // send something the server cannot parse.
        _publishFailures++;
        logError("batch too large for the %u byte buffer - raise "
                      "TELEM_MQTT_BUFFER_SIZE or publish fewer signals",
                      TELEM_MQTT_BUFFER_SIZE);
        return;
    }

    if (_mqtt.publish(_topicData, (const uint8_t *)payload, len, false)) {
        _publishCount++;
        _seq++;
    } else {
        _publishFailures++;
        logWarn("publish failed (%u bytes), mqtt state %d", (unsigned)len, _mqtt.state());
    }
}

void MqttManager::publishStatus(const char *state)
{
    if (!_mqtt.connected()) return;

    JsonDocument doc;
    doc["state"]    = state;
    doc["dev"]      = deviceId;
    doc["fw"]       = CFG_VERSION;
    doc["ip"]       = WiFi.localIP().toString();
    doc["rssi"]     = WiFi.RSSI();
    doc["up"]       = millis() / 1000;
    doc["heap"]     = ESP.getFreeHeap();
    doc["frames"]   = signalDB.framesSeen();
    doc["matched"]  = signalDB.framesMatched();
    doc["pub"]      = _publishCount;
    doc["pubFail"]  = _publishFailures;

    char payload[512];
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len > 0 && len < sizeof(payload)) {
        // Retained, so a server connecting later immediately learns our state.
        _mqtt.publish(_topicStatus, (const uint8_t *)payload, len, true);
    }
}

// ---------------------------------------------------------------------------
// Inbound commands
// ---------------------------------------------------------------------------

void MqttManager::mqttCallbackEntry(char *topic, uint8_t *payload, unsigned int length)
{
    (void)topic;   // we only ever subscribe to one topic
    if (s_instance != nullptr) {
        s_instance->handleCommand((const char *)payload, length);
    }
}

/*
 * Supported commands (published to <base>/<device>/cmd):
 *   {"cmd":"status"}                 - publish a status message now
 *   {"cmd":"reboot"}                 - restart the board
 *   {"cmd":"interval","ms":500}      - change publish cadence, persisted to NVS
 *   {"cmd":"trip_reset"}             - zero the trip distance/energy counters
 *   {"cmd":"ota"}                    - update from the stored OTA URL
 *   {"cmd":"ota","url":"http://..."} - update from an explicit URL
 */
void MqttManager::handleCommand(const char *json, unsigned int length)
{
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, json, length);
    if (err) {
        logWarn("ignoring malformed command payload: %s", err.c_str());
        return;
    }

    const char *cmd = doc["cmd"] | "";
    logInfo("command received: %s", cmd);

    if (strcmp(cmd, "status") == 0) {
        publishStatus("online");

    } else if (strcmp(cmd, "reboot") == 0) {
        publishStatus("rebooting");
        _mqtt.loop();
        delay(200);              // let the packet actually leave before we reset
        ESP.restart();

    } else if (strcmp(cmd, "interval") == 0) {
        const uint16_t ms = doc["ms"] | 0;
        if (ms >= 50) {
            cfg.publishIntervalMs = ms;
            cfgSave();
            logInfo("publish interval is now %u ms", ms);
        } else {
            logWarn("rejected interval %u ms - the minimum is 50", ms);
        }

    } else if (strcmp(cmd, "trip_reset") == 0) {
        energyMeter.resetTrip();
        publishStatus("online");

    } else if (strcmp(cmd, "ota") == 0) {
        const char *url = doc["url"] | cfg.otaUrl;
        publishStatus("updating");
        _mqtt.loop();
        // Runs on this task, on core 0, so CAN reception on core 1 continues
        // right up until the device reboots into the new image.
        otaManager.performUpdate(url);

    } else {
        logWarn("unknown command: %s", cmd);
    }
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

void MqttManager::printStatus()
{
    Serial.println(F("\n  link status"));
    Serial.println(F("  ------------------------------------------------------------"));
    Serial.printf("  wifi          : %s\n", isWifiConnected() ? "connected" : "DISCONNECTED");
    if (isWifiConnected()) {
        Serial.printf("  ip / rssi     : %s / %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        Serial.printf("  gateway       : %s\n", WiFi.gatewayIP().toString().c_str());
    }
    Serial.printf("  mqtt          : %s (state %d)\n",
                  _mqtt.connected() ? "connected" : "DISCONNECTED", _mqtt.state());
    Serial.printf("  data topic    : %s\n", _topicData);
    Serial.printf("  cmd topic     : %s\n", _topicCmd);
    Serial.printf("  published     : %lu ok, %lu failed, %lu reconnects\n",
                  (unsigned long)_publishCount, (unsigned long)_publishFailures,
                  (unsigned long)_mqttReconnects);
    Serial.printf("  free heap     : %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    Serial.println();
}

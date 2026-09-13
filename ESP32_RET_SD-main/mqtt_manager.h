/*
 * mqtt_manager.h
 *
 * Wi-Fi station + MQTT client + batched signal publisher.
 *
 * THREADING MODEL
 * ---------------
 * The Arduino loop() runs pinned to core 1 and does exactly one thing in
 * telemetry mode: drain the CAN controller and decode frames. All networking -
 * association, DNS, TCP, MQTT, OTA - happens in a task pinned to core 0, which
 * is also where the ESP32 Wi-Fi driver itself runs.
 *
 * That split matters. MQTT publishes block, DNS lookups block, and a dropped
 * cellular link can stall a socket write for seconds. Keeping all of it off
 * core 1 means CAN reception never pauses, so we cannot miss frames because the
 * network had a bad moment.
 */

#ifndef MQTT_MANAGER_H_
#define MQTT_MANAGER_H_

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

class MqttManager {
public:
    MqttManager();

    // Brings up Wi-Fi and spawns the network task on core 0. Non-blocking:
    // it returns immediately and connection progress is reported over Serial.
    bool begin();

    // Diagnostics used by the serial console and the status payload.
    bool     isWifiConnected() const;
    bool     isMqttConnected() const   { return _mqttConnected; }
    uint32_t publishCount() const      { return _publishCount; }
    uint32_t publishFailures() const   { return _publishFailures; }
    uint32_t mqttReconnects() const    { return _mqttReconnects; }
    void     printStatus();

    // Publish a one-off status message immediately (console "status" / on demand).
    void publishStatus(const char *state);

    // Set by the OTA handler so we stop transmitting mid-update.
    void setPaused(bool paused)        { _paused = paused; }

private:
    WiFiClient    _net;
    PubSubClient  _mqtt;

    char _topicData[96];
    char _topicStatus[96];
    char _topicCmd[96];

    volatile bool _mqttConnected;
    volatile bool _paused;

    uint32_t _publishCount;
    uint32_t _publishFailures;
    uint32_t _mqttReconnects;
    uint32_t _lastPublishMs;
    uint32_t _lastHeartbeatMs;
    uint32_t _nextMqttAttemptMs;
    uint16_t _backoffMs;
    uint32_t _seq;

    void buildTopics();
    void ensureWifi();
    void ensureMqtt();
    void publishBatch();

    static void networkTaskEntry(void *arg);
    void networkTask();

    // PubSubClient hands inbound messages back through a plain C callback.
    static void mqttCallbackEntry(char *topic, uint8_t *payload, unsigned int length);
    void handleCommand(const char *json, unsigned int length);
};

extern MqttManager mqttManager;

#endif /* MQTT_MANAGER_H_ */

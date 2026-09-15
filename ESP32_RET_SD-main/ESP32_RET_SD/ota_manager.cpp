/*
 * ota_manager.cpp
 *
 * See ota_manager.h for why this is pull-based rather than ArduinoOTA.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPUpdate.h>
#include "ota_manager.h"
#include "mqtt_manager.h"
#include "log.h"

OtaManager otaManager;

OtaManager::OtaManager()
    : _updating(false), _lastResult(0)
{
}

void OtaManager::onStart()
{
    logInfo("OTA: download started");
}

void OtaManager::onEnd()
{
    logInfo("OTA: image written, rebooting");
}

void OtaManager::onProgress(int current, int total)
{
    // One line per 10% keeps the serial log readable over a slow cellular link.
    static int lastDecile = -1;
    if (total <= 0) return;
    const int decile = (current * 10) / total;
    if (decile != lastDecile) {
        lastDecile = decile;
        logInfo("OTA: %d%% (%d / %d bytes)", decile * 10, current, total);
    }
}

void OtaManager::onError(int err)
{
    logError("OTA: error %d", err);
}

bool OtaManager::performUpdate(const char *url)
{
    if (url == nullptr || strlen(url) == 0) {
        logError("OTA: no URL configured");
        return false;
    }

    if (WiFi.status() != WL_CONNECTED) {
        logError("OTA: no network");
        return false;
    }

    if (_updating) {
        logWarn("OTA: an update is already in progress");
        return false;
    }

    _updating = true;

    // Stop publishing for the duration. The CAN task keeps running on core 1 and
    // keeps the signal table current, we simply stop putting bytes on the wire so
    // the whole cellular link is available to the download.
    mqttManager.setPaused(true);

    logInfo("OTA: fetching %s", url);

    httpUpdate.onStart(onStart);
    httpUpdate.onEnd(onEnd);
    httpUpdate.onProgress(onProgress);
    httpUpdate.onError(onError);

    // Reboot ourselves after a successful write so we control the timing rather
    // than being restarted from inside the HTTP callback.
    httpUpdate.rebootOnUpdate(false);

    // A separate client from the MQTT one - reusing that socket would tear down
    // the broker connection mid-transfer.
    WiFiClient otaClient;
    const t_httpUpdate_return ret = httpUpdate.update(otaClient, url);

    _lastResult = (uint32_t)ret;
    _updating = false;

    switch (ret) {
        case HTTP_UPDATE_OK:
            logInfo("OTA: success - restarting into the new firmware");
            delay(500);
            ESP.restart();
            return true;   // not reached

        case HTTP_UPDATE_NO_UPDATES:
            // The server answered 304, meaning it considers us already current.
            logInfo("OTA: server reports no update available");
            break;

        case HTTP_UPDATE_FAILED:
        default:
            logError("OTA: failed (%d) %s",
                          httpUpdate.getLastError(),
                          httpUpdate.getLastErrorString().c_str());
            break;
    }

    // Failed or unnecessary - resume normal telemetry on the existing firmware.
    mqttManager.setPaused(false);
    return false;
}

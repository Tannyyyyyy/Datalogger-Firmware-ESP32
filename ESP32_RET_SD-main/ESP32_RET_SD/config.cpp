/*
 * config.cpp
 *
 * Non-volatile settings backing store (NVS via Preferences).
 *
 * On the very first boot the namespace is empty, so every key falls back to the
 * compile-time default in config.h. A factory-fresh board therefore
 * self-populates and subsequent boots are pure NVS reads.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <esp_mac.h>
#include "config.h"

Config cfg;
char deviceId[24] = {0};

static Preferences prefs;
static const char *NS = "telem";

// Copy a String into a fixed char buffer, always NUL terminated.
static void copyStr(char *dest, size_t destSize, const String &src)
{
    strncpy(dest, src.c_str(), destSize - 1);
    dest[destSize - 1] = '\0';
}

void cfgLoad()
{
    // Stable per-board identity from the Wi-Fi MAC. Read straight from efuse
    // rather than via WiFi.macAddress() so this works before the radio starts.
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(deviceId, sizeof(deviceId), "canlog-%02X%02X%02X", mac[3], mac[4], mac[5]);

    prefs.begin(NS, false);   // read/write

    // NVS keys are limited to 15 characters.
    copyStr(cfg.wifiSSID,   sizeof(cfg.wifiSSID),   prefs.getString("ssid",   TELEM_WIFI_SSID));
    copyStr(cfg.wifiPass,   sizeof(cfg.wifiPass),   prefs.getString("pass",   TELEM_WIFI_PASS));
    copyStr(cfg.brokerHost, sizeof(cfg.brokerHost), prefs.getString("host",   TELEM_MQTT_HOST));
    copyStr(cfg.mqttUser,   sizeof(cfg.mqttUser),   prefs.getString("muser",  TELEM_MQTT_USER));
    copyStr(cfg.mqttPass,   sizeof(cfg.mqttPass),   prefs.getString("mpass",  TELEM_MQTT_PASS));
    copyStr(cfg.baseTopic,  sizeof(cfg.baseTopic),  prefs.getString("topic",  TELEM_BASE_TOPIC));
    copyStr(cfg.otaUrl,     sizeof(cfg.otaUrl),     prefs.getString("otaurl", TELEM_OTA_URL));

    cfg.brokerPort        = prefs.getUShort("port",   TELEM_MQTT_PORT);
    cfg.publishIntervalMs = prefs.getUShort("pubms",  TELEM_PUBLISH_INTERVAL_MS);
    cfg.canSpeed          = prefs.getULong("canspd",  TELEM_CAN_SPEED);
    cfg.canListenOnly     = prefs.getBool("lonly",    TELEM_CAN_LISTEN_ONLY);
    cfg.publishUnchanged  = prefs.getBool("unchg",    TELEM_PUBLISH_UNCHANGED);
    cfg.useHvCurrent      = prefs.getBool("hvcurr",   ENERGY_USE_HV_CURRENT);
    cfg.currentSign       = prefs.getChar("cursign",  ENERGY_CURRENT_SIGN);
    cfg.useOdometer       = prefs.getBool("useodo",   ENERGY_USE_ODOMETER);

    prefs.end();

    // Only +1 and -1 are meaningful; a stored 0 would zero out all power.
    cfg.currentSign = (cfg.currentSign >= 0) ? 1 : -1;

    // Guard against a nonsensical interval (a stored 0 would spin the publisher).
    if (cfg.publishIntervalMs < 50) cfg.publishIntervalMs = 50;
}

void cfgSave()
{
    prefs.begin(NS, false);

    prefs.putString("ssid",   cfg.wifiSSID);
    prefs.putString("pass",   cfg.wifiPass);
    prefs.putString("host",   cfg.brokerHost);
    prefs.putString("muser",  cfg.mqttUser);
    prefs.putString("mpass",  cfg.mqttPass);
    prefs.putString("topic",  cfg.baseTopic);
    prefs.putString("otaurl", cfg.otaUrl);

    prefs.putUShort("port",   cfg.brokerPort);
    prefs.putUShort("pubms",  cfg.publishIntervalMs);
    prefs.putULong("canspd",  cfg.canSpeed);
    prefs.putBool("lonly",    cfg.canListenOnly);
    prefs.putBool("unchg",    cfg.publishUnchanged);
    prefs.putBool("hvcurr",   cfg.useHvCurrent);
    prefs.putChar("cursign",  cfg.currentSign);
    prefs.putBool("useodo",   cfg.useOdometer);

    prefs.end();

    Serial.println(F("settings saved to NVS"));
}

void cfgReset()
{
    prefs.begin(NS, false);
    prefs.clear();
    prefs.end();
    cfgLoad();
    Serial.println(F("settings reset to compile-time defaults"));
}

void cfgPrint()
{
    Serial.println(F("\n  current settings"));
    Serial.println(F("  ------------------------------------------------------------"));
    Serial.printf("  device id     : %s\n",    deviceId);
    Serial.printf("  wifi ssid     : %s\n",    cfg.wifiSSID);
    Serial.printf("  wifi pass     : %s\n",    strlen(cfg.wifiPass) ? "(set)" : "(empty)");
    Serial.printf("  broker        : %s:%u\n", cfg.brokerHost, cfg.brokerPort);
    Serial.printf("  mqtt user     : %s\n",    strlen(cfg.mqttUser) ? cfg.mqttUser : "(anonymous)");
    Serial.printf("  base topic    : %s\n",    cfg.baseTopic);
    Serial.printf("  publish every : %u ms\n", cfg.publishIntervalMs);
    Serial.printf("  publish all   : %s\n",    cfg.publishUnchanged ? "yes (every signal, every batch)"
                                                                    : "no (changed signals only)");
    Serial.printf("  can speed     : %lu bps\n", (unsigned long)cfg.canSpeed);
    Serial.printf("  can mode      : %s\n",    cfg.canListenOnly ? "listen-only (silent)" : "normal (will ACK)");
    Serial.printf("  ota url       : %s\n",    cfg.otaUrl);
    Serial.println(F("  --- energy meter ---"));
    Serial.printf("  current src   : %s\n",    cfg.useHvCurrent ? "hvCurr (CAB500)" : "packCurr (BMS)");
    Serial.printf("  current sign  : %+d (%s current means discharge)\n",
                  (int)cfg.currentSign, cfg.currentSign > 0 ? "positive" : "negative");
    Serial.printf("  distance src  : %s\n",    cfg.useOdometer ? "odometer deltas" : "integrated speed");
    Serial.println();
}

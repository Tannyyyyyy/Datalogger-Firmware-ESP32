/*
 * console.cpp
 */

#include <Arduino.h>
#include "console.h"
#include "config.h"
#include "mqtt_manager.h"
#include <Wire.h>
#include "signal_db.h"
#include "energy_meter.h"
#include "imu_mpu6500.h"
#include "adc_ads1115.h"
#include "ota_manager.h"

/*
 * Walk the 7-bit address space looking for anything that ACKs. This is the
 * first thing worth running when a sensor does not appear: it separates "wrong
 * address" from "not wired / no pull-ups" in one command.
 */
static void i2cScan()
{
    Serial.printf("\n  scanning I2C on SDA %d / SCL %d ...\n",
                  TELEM_I2C_SDA, TELEM_I2C_SCL);

    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            const char *hint = "";
            if (addr == 0x68 || addr == 0x69) hint = "  <- MPU6500 (or an RTC)";
            if (addr >= 0x48 && addr <= 0x4B) hint = "  <- ADS1115";
            Serial.printf("    0x%02X%s\n", addr, hint);
            found++;
        }
    }

    if (found == 0) {
        Serial.println(F("    nothing responded."));
        Serial.println(F("    check: 3.3 V and GND to both parts, SDA/SCL not swapped,"));
        Serial.println(F("    and 4.7k pull-ups to 3.3 V on both lines."));
    } else {
        Serial.printf("  %d device(s) found\n", found);
    }
    Serial.println();
}

Console console;

Console::Console()
    : _len(0)
{
    _buf[0] = '\0';
}

void Console::begin()
{
    printMenu();
}

void Console::printMenu()
{
    Serial.println(F("\n============================================================"));
    Serial.print  (F("  "));
    Serial.println(F(CFG_VERSION));
    Serial.println(F("  CAN -> MQTT telemetry console"));
    Serial.println(F("============================================================"));
    Serial.println(F("  help              this menu"));
    Serial.println(F("  show              print current settings"));
    Serial.println(F("  status            print wifi / mqtt link status"));
    Serial.println(F("  signals           print the live decoded signal table"));
    Serial.println(F("  trip              print energy / consumption figures"));
    Serial.println(F("  trip reset        zero the trip distance and energy"));
    Serial.println(F("  trip resetlife    zero the LIFETIME totals (irreversible)"));
    Serial.println(F("  imu               print IMU readings (MPU6500)"));
    Serial.println(F("  imucal            zero the gyro - VEHICLE MUST BE STILL"));
    Serial.println(F("  adc               print ADC channels (ADS1115)"));
    Serial.println(F("  i2cscan           list devices responding on the I2C bus"));
    Serial.println(F("  counters          reset the frame / publish counters"));
    Serial.println(F("  save              write settings to NVS"));
    Serial.println(F("  defaults          erase NVS, restore compile-time values"));
    Serial.println(F("  ota [url]         update firmware now (stored url if omitted)"));
    Serial.println(F("  reboot            restart the board"));
    Serial.println();
    Serial.println(F("  set ssid     <text>    4G router SSID"));
    Serial.println(F("  set pass     <text>    4G router password"));
    Serial.println(F("  set host     <text>    MQTT broker IP or hostname"));
    Serial.println(F("  set port     <num>     MQTT broker port (default 1883)"));
    Serial.println(F("  set user     <text>    MQTT username (blank for anonymous)"));
    Serial.println(F("  set mpass    <text>    MQTT password"));
    Serial.println(F("  set topic    <text>    base topic, device id is appended"));
    Serial.println(F("  set interval <num>     publish period in ms (min 50)"));
    Serial.println(F("  set canspeed <num>     CAN bitrate, e.g. 500000"));
    Serial.println(F("  set listen   <0|1>     1 = listen-only, never ACK (recommended)"));
    Serial.println(F("  set all      <0|1>     1 = send every signal in every batch"));
    Serial.println(F("  set otaurl   <text>    firmware image URL"));
    Serial.println();
    Serial.println(F("  set hvcurr   <0|1>     1 = integrate CAB500 'hvCurr', 0 = BMS 'packCurr'"));
    Serial.println(F("  set cursign  <1|-1>    sign that makes DISCHARGE positive"));
    Serial.println(F("  set useodo   <0|1>     1 = odometer deltas, 0 = integrate speed"));
    Serial.println();
    Serial.println(F("  Settings take effect after 'save' + 'reboot', except"));
    Serial.println(F("  'interval' and 'all', which apply immediately."));
    Serial.println(F("============================================================\n"));
}

bool Console::processByte(uint8_t c)
{
    // Accept both CR and LF so any terminal works.
    if (c == '\r' || c == '\n') {
        if (_len == 0) return false;
        _buf[_len] = '\0';
        Serial.println();
        handleLine();
        _len = 0;
        return true;
    }

    // Backspace / delete.
    if (c == 0x08 || c == 0x7F) {
        if (_len > 0) {
            _len--;
            Serial.print(F("\b \b"));
        }
        return false;
    }

    if (_len < CMD_BUF_LEN - 1 && c >= 0x20) {
        _buf[_len++] = (char)c;
        Serial.write(c);          // local echo
    }
    return false;
}

void Console::handleLine()
{
    // Split the leading verb from the rest of the line.
    char *line = _buf;
    while (*line == ' ') line++;

    char *args = strchr(line, ' ');
    if (args != nullptr) {
        *args = '\0';
        args++;
        while (*args == ' ') args++;
    }

    if (strcasecmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        printMenu();

    } else if (strcasecmp(line, "show") == 0) {
        cfgPrint();

    } else if (strcasecmp(line, "status") == 0) {
        mqttManager.printStatus();

    } else if (strcasecmp(line, "signals") == 0) {
        signalDB.dumpToSerial();

    } else if (strcasecmp(line, "trip") == 0) {
        if (args == nullptr || *args == '\0') {
            energyMeter.printToSerial();
        } else if (strcasecmp(args, "reset") == 0) {
            energyMeter.resetTrip();
        } else if (strcasecmp(args, "resetlife") == 0) {
            energyMeter.resetLifetime();
        } else {
            Serial.println(F("usage: trip | trip reset | trip resetlife"));
        }

    } else if (strcasecmp(line, "imu") == 0) {
        imu.printToSerial();

    } else if (strcasecmp(line, "imucal") == 0) {
        imu.calibrateGyro();

    } else if (strcasecmp(line, "adc") == 0) {
        adc.printToSerial();

    } else if (strcasecmp(line, "i2cscan") == 0) {
        i2cScan();

    } else if (strcasecmp(line, "counters") == 0) {
        signalDB.resetCounters();
        Serial.println(F("counters reset"));

    } else if (strcasecmp(line, "save") == 0) {
        cfgSave();

    } else if (strcasecmp(line, "defaults") == 0) {
        cfgReset();

    } else if (strcasecmp(line, "ota") == 0) {
        const char *url = (args != nullptr && *args) ? args : cfg.otaUrl;
        Serial.printf("starting OTA from %s\n", url);
        otaManager.performUpdate(url);

    } else if (strcasecmp(line, "reboot") == 0) {
        Serial.println(F("rebooting..."));
        delay(100);
        ESP.restart();

    } else if (strcasecmp(line, "set") == 0) {
        if (args == nullptr) {
            Serial.println(F("usage: set <key> <value>   (see 'help')"));
        } else {
            handleSet(args);
        }

    } else {
        Serial.printf("unknown command '%s' - type 'help'\n", line);
    }
}

void Console::handleSet(char *args)
{
    char *key = args;
    char *val = strchr(args, ' ');
    if (val != nullptr) {
        *val = '\0';
        val++;
        while (*val == ' ') val++;
    } else {
        // Allow an empty value so credentials can be cleared: "set user"
        val = (char *)"";
    }

    if (strcasecmp(key, "ssid") == 0) {
        strncpy(cfg.wifiSSID, val, sizeof(cfg.wifiSSID) - 1);
        cfg.wifiSSID[sizeof(cfg.wifiSSID) - 1] = '\0';
        Serial.printf("ssid = %s\n", cfg.wifiSSID);

    } else if (strcasecmp(key, "pass") == 0) {
        strncpy(cfg.wifiPass, val, sizeof(cfg.wifiPass) - 1);
        cfg.wifiPass[sizeof(cfg.wifiPass) - 1] = '\0';
        Serial.println(F("wifi password updated"));

    } else if (strcasecmp(key, "host") == 0) {
        strncpy(cfg.brokerHost, val, sizeof(cfg.brokerHost) - 1);
        cfg.brokerHost[sizeof(cfg.brokerHost) - 1] = '\0';
        Serial.printf("host = %s\n", cfg.brokerHost);

    } else if (strcasecmp(key, "port") == 0) {
        const long p = strtol(val, nullptr, 10);
        if (p > 0 && p < 65536) {
            cfg.brokerPort = (uint16_t)p;
            Serial.printf("port = %u\n", cfg.brokerPort);
        } else {
            Serial.println(F("port must be 1..65535"));
        }

    } else if (strcasecmp(key, "user") == 0) {
        strncpy(cfg.mqttUser, val, sizeof(cfg.mqttUser) - 1);
        cfg.mqttUser[sizeof(cfg.mqttUser) - 1] = '\0';
        Serial.printf("mqtt user = %s\n", cfg.mqttUser);

    } else if (strcasecmp(key, "mpass") == 0) {
        strncpy(cfg.mqttPass, val, sizeof(cfg.mqttPass) - 1);
        cfg.mqttPass[sizeof(cfg.mqttPass) - 1] = '\0';
        Serial.println(F("mqtt password updated"));

    } else if (strcasecmp(key, "topic") == 0) {
        strncpy(cfg.baseTopic, val, sizeof(cfg.baseTopic) - 1);
        cfg.baseTopic[sizeof(cfg.baseTopic) - 1] = '\0';
        Serial.printf("base topic = %s\n", cfg.baseTopic);

    } else if (strcasecmp(key, "interval") == 0) {
        const long ms = strtol(val, nullptr, 10);
        if (ms >= 50 && ms <= 65535) {
            cfg.publishIntervalMs = (uint16_t)ms;
            Serial.printf("publish interval = %u ms (live)\n", cfg.publishIntervalMs);
        } else {
            Serial.println(F("interval must be 50..65535 ms"));
        }

    } else if (strcasecmp(key, "canspeed") == 0) {
        const long bps = strtol(val, nullptr, 10);
        if (bps > 0) {
            cfg.canSpeed = (uint32_t)bps;
            Serial.printf("can speed = %lu bps (applies after reboot)\n",
                          (unsigned long)cfg.canSpeed);
        } else {
            Serial.println(F("canspeed must be a positive bitrate"));
        }

    } else if (strcasecmp(key, "listen") == 0) {
        cfg.canListenOnly = (strtol(val, nullptr, 10) != 0);
        Serial.printf("listen-only = %s (applies after reboot)\n",
                      cfg.canListenOnly ? "yes" : "no");

    } else if (strcasecmp(key, "all") == 0) {
        cfg.publishUnchanged = (strtol(val, nullptr, 10) != 0);
        Serial.printf("publish every signal = %s (live)\n",
                      cfg.publishUnchanged ? "yes" : "no");

    } else if (strcasecmp(key, "hvcurr") == 0) {
        cfg.useHvCurrent = (strtol(val, nullptr, 10) != 0);
        Serial.printf("energy current source = %s (applies after reboot)\n",
                      cfg.useHvCurrent ? "hvCurr (CAB500)" : "packCurr (BMS)");

    } else if (strcasecmp(key, "cursign") == 0) {
        cfg.currentSign = (strtol(val, nullptr, 10) < 0) ? -1 : 1;
        Serial.printf("current sign = %+d (%s current means discharge, live)\n",
                      (int)cfg.currentSign,
                      cfg.currentSign > 0 ? "positive" : "negative");

    } else if (strcasecmp(key, "useodo") == 0) {
        cfg.useOdometer = (strtol(val, nullptr, 10) != 0);
        Serial.printf("distance source = %s (live)\n",
                      cfg.useOdometer ? "odometer deltas" : "integrated speed");

    } else if (strcasecmp(key, "otaurl") == 0) {
        strncpy(cfg.otaUrl, val, sizeof(cfg.otaUrl) - 1);
        cfg.otaUrl[sizeof(cfg.otaUrl) - 1] = '\0';
        Serial.printf("ota url = %s\n", cfg.otaUrl);

    } else {
        Serial.printf("unknown setting '%s' - type 'help'\n", key);
        return;
    }

    Serial.println(F("(remember to 'save')"));
}

/*
 * ota_manager.h
 *
 * Pull-based HTTP OTA.
 *
 * WHY NOT ArduinoOTA?
 * -------------------
 * The board reaches the internet through a 4G module, which means it sits behind
 * carrier-grade NAT with no inbound reachability and no mDNS across the link.
 * ArduinoOTA is push-based - your PC opens a connection to the device - so it
 * cannot work here. Instead the device pulls: it fetches a .bin over HTTP when
 * told to, verifies and writes it to the inactive OTA partition, and reboots.
 *
 * Trigger it either with an MQTT command ({"cmd":"ota"}) or from the serial
 * console ("ota [url]").
 */

#ifndef OTA_MANAGER_H_
#define OTA_MANAGER_H_

#include <Arduino.h>

class OtaManager {
public:
    OtaManager();

    // Downloads and installs the image at url, then reboots on success.
    // Blocking - call it from the network task, never from the CAN loop.
    // Returns false if the update did not happen (the device keeps running the
    // current firmware; a failed flash never leaves a half-written partition
    // active, because the bootloader only switches over after a full verify).
    bool performUpdate(const char *url);

    bool     isUpdating() const     { return _updating; }
    uint32_t lastResult() const     { return _lastResult; }

private:
    volatile bool _updating;
    uint32_t      _lastResult;

    static void onStart();
    static void onEnd();
    static void onProgress(int current, int total);
    static void onError(int err);
};

extern OtaManager otaManager;

#endif /* OTA_MANAGER_H_ */

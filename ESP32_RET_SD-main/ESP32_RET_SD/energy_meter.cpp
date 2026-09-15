/*
 * energy_meter.cpp
 *
 * See energy_meter.h for the design rationale.
 */

#include <Arduino.h>
#include <Preferences.h>
#include "energy_meter.h"
#include "signal_db.h"
#include "config.h"
#include "log.h"

EnergyMeter energyMeter;

static Preferences energyPrefs;
static const char *ENERGY_NS = "energy";

EnergyMeter::EnergyMeter()
    : _idxVolt(-1), _idxPackCurr(-1), _idxHvCurr(-1), _idxSpeed(-1), _idxOdo(-1),
      _resolved(false), _mutex(nullptr),
      _volt(0), _current(0), _powerKw(0), _speed(0), _integrating(false),
      _tripDistKm(0), _tripOutWh(0), _tripInWh(0), _tripStartMs(0),
      _lifeDistKm(0), _lifeOutWh(0), _lifeInWh(0),
      _windowDistKm(0), _windowNetWh(0), _recentWhPerKm(0),
      _lastOdo(0), _haveLastOdo(false),
      _lastUpdateMs(0), _lastSaveMs(0),
      _keyPin(TELEM_KEY_SWITCH_PIN), _keyLast(false), _keyDebounceMs(0)
{
}

bool EnergyMeter::begin()
{
    _mutex = xSemaphoreCreateMutex();
    if (_mutex == nullptr) {
        logError("failed to create energy meter mutex");
        return false;
    }

    // Resolve the signal names once. Every one of these is a string compare
    // through the whole table, so it happens here and never again.
    _idxVolt     = signalDB.findIndex("packVolt");
    _idxPackCurr = signalDB.findIndex("packCurr");
    _idxHvCurr   = signalDB.findIndex("hvCurr");
    _idxSpeed    = signalDB.findIndex("speed");
    _idxOdo      = signalDB.findIndex("odo");

    // Report exactly what is missing rather than silently reading zero forever.
    if (_idxVolt < 0) {
        logError("energy meter: no 'packVolt' signal in the table - "
                      "consumption cannot be calculated");
    }
    if (_idxPackCurr < 0 && _idxHvCurr < 0) {
        logError("energy meter: neither 'hvCurr' nor 'packCurr' is in the "
                      "table - consumption cannot be calculated");
    }
    if (_idxOdo < 0 && _idxSpeed < 0) {
        logError("energy meter: neither 'odo' nor 'speed' is in the table - "
                      "distance cannot be measured, so Wh/km is unavailable");
    }
    if (cfg.useHvCurrent && _idxHvCurr < 0 && _idxPackCurr >= 0) {
        logWarn("energy meter: 'hvCurr' selected but not in the table - "
                     "falling back to 'packCurr'");
    }
    if (_idxOdo < 0 && _idxSpeed >= 0) {
        logInfo("energy meter: no odometer signal, integrating speed for distance");
    }

    _resolved = (_idxVolt >= 0) && (_idxPackCurr >= 0 || _idxHvCurr >= 0);

    load();

    _tripStartMs = millis();
    _lastUpdateMs = millis();
    _lastSaveMs = millis();

    if (_keyPin >= 0) {
        pinMode(_keyPin, INPUT);
        _keyLast = (digitalRead(_keyPin) == (TELEM_KEY_SWITCH_ACTIVE_HIGH ? HIGH : LOW));
        logInfo("energy meter: key switch on GPIO %d", _keyPin);
    }

    logInfo("energy meter ready (lifetime %.1f km, %.0f Wh)",
                 _lifeDistKm, _lifeOutWh);
    return true;
}

float EnergyMeter::safeDiv(float numerator, float denominator, float minDenominator)
{
    if (denominator < minDenominator) return 0.0f;
    return numerator / denominator;
}

void EnergyMeter::serviceKeySwitch()
{
    if (_keyPin < 0) return;

    const uint32_t now = millis();
    if (now - _keyDebounceMs < 50) return;
    _keyDebounceMs = now;

    const bool keyNow = (digitalRead(_keyPin) == (TELEM_KEY_SWITCH_ACTIVE_HIGH ? HIGH : LOW));
    if (keyNow == _keyLast) return;

    _keyLast = keyNow;
    if (keyNow) {
        logInfo("key on - starting a new trip");
        resetTrip();
    } else {
        logInfo("key off - saving totals");
        save();
    }
}

void EnergyMeter::update()
{
    serviceKeySwitch();

    const uint32_t now = millis();
    const uint32_t elapsed = now - _lastUpdateMs;
    if (elapsed < ENERGY_UPDATE_MS) return;
    _lastUpdateMs = now;

    if (!_resolved) return;

    // --- gather inputs, rejecting anything stale -------------------------
    float volt = 0, curr = 0, speed = 0, odo = 0;
    uint32_t ageV = 0, ageI = 0, ageS = 0, ageO = 0;

    const bool haveV = signalDB.getValue(_idxVolt, volt, ageV) &&
                       ageV <= ENERGY_SIGNAL_TIMEOUT_MS;

    // Prefer the dedicated transducer when it is configured and present.
    int currIdx = -1;
    if (cfg.useHvCurrent && _idxHvCurr >= 0)      currIdx = _idxHvCurr;
    else if (_idxPackCurr >= 0)                             currIdx = _idxPackCurr;
    else                                                    currIdx = _idxHvCurr;

    const bool haveI = (currIdx >= 0) &&
                       signalDB.getValue(currIdx, curr, ageI) &&
                       ageI <= ENERGY_SIGNAL_TIMEOUT_MS;

    const bool haveS = (_idxSpeed >= 0) && signalDB.getValue(_idxSpeed, speed, ageS) &&
                       ageS <= ENERGY_SIGNAL_TIMEOUT_MS;
    const bool haveO = (_idxOdo >= 0) && signalDB.getValue(_idxOdo, odo, ageO) &&
                       ageO <= ENERGY_SIGNAL_TIMEOUT_MS;

    // Normalise so that positive always means "energy leaving the pack".
    curr *= (float)cfg.currentSign;

    const float dtHours = elapsed / 3600000.0f;

    // --- distance ---------------------------------------------------------
    float distKm = 0.0f;
    if (cfg.useOdometer && haveO) {
        if (_haveLastOdo) {
            const float delta = odo - _lastOdo;
            // Reject a negative delta (counter reset) and an implausible jump.
            // 5 km in one 100 ms tick would be 180,000 km/h, so anything near it
            // is a decode error or a rollover, not motion.
            if (delta >= 0.0f && delta < 5.0f) distKm = delta;
        }
        _lastOdo = odo;
        _haveLastOdo = true;
    } else if (haveS) {
        distKm = speed * dtHours;
    }

    // --- power and energy -------------------------------------------------
    float powerW = 0.0f;
    const bool integrating = haveV && haveI;
    if (integrating) {
        powerW = volt * curr;
    }

    const float energyWh = powerW * dtHours;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

    _volt        = haveV ? volt : 0.0f;
    _current     = haveI ? curr : 0.0f;
    _powerKw     = powerW / 1000.0f;
    _speed       = haveS ? speed : 0.0f;
    _integrating = integrating;

    if (integrating) {
        if (energyWh >= 0.0f) {
            _tripOutWh += energyWh;
            _lifeOutWh += energyWh;
        } else {
            _tripInWh -= energyWh;   // energyWh is negative during regen
            _lifeInWh -= energyWh;
        }
        _windowNetWh += energyWh;
    }

    _tripDistKm   += distKm;
    _lifeDistKm   += distKm;
    _windowDistKm += distKm;

    // Roll the recent window once it has filled. Restarting from zero rather
    // than keeping a sample buffer costs a little smoothness but no RAM, and
    // for a dashboard figure that is the right trade.
    if (_windowDistKm >= ENERGY_RECENT_WINDOW_KM) {
        _recentWhPerKm = _windowNetWh / _windowDistKm;
        _windowDistKm = 0.0f;
        _windowNetWh = 0.0f;
    }

    xSemaphoreGive(_mutex);

    if (now - _lastSaveMs >= ENERGY_NVS_SAVE_MS) {
        _lastSaveMs = now;
        save();
    }
}

void EnergyMeter::snapshot(EnergySnapshot &out)
{
    memset(&out, 0, sizeof(out));

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    out.packVolt   = _volt;
    out.current    = _current;
    out.power      = _powerKw;
    out.speed      = _speed;

    out.tripDistanceKm  = _tripDistKm;
    out.tripEnergyOutWh = _tripOutWh;
    out.tripEnergyInWh  = _tripInWh;
    out.tripNetWh       = _tripOutWh - _tripInWh;
    out.tripAvgWhPerKm  = safeDiv(out.tripNetWh, _tripDistKm, ENERGY_MIN_TRIP_KM);
    out.tripRegenPercent = safeDiv(_tripInWh * 100.0f, _tripOutWh, 0.001f);

    out.recentWhPerKm = _recentWhPerKm;

    out.lifeDistanceKm  = _lifeDistKm;
    out.lifeEnergyOutWh = _lifeOutWh;
    out.lifeAvgWhPerKm  = safeDiv(_lifeOutWh - _lifeInWh, _lifeDistKm, ENERGY_MIN_TRIP_KM);

    out.tripSeconds  = (millis() - _tripStartMs) / 1000;
    out.integrating  = _integrating;

    xSemaphoreGive(_mutex);
}

void EnergyMeter::resetTrip()
{
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    _tripDistKm = 0.0f;
    _tripOutWh = 0.0f;
    _tripInWh = 0.0f;
    _tripStartMs = millis();
    _windowDistKm = 0.0f;
    _windowNetWh = 0.0f;
    _recentWhPerKm = 0.0f;
    // Forget the odometer reference so the first post-reset tick does not book
    // the distance covered while the meter was stopped.
    _haveLastOdo = false;
    xSemaphoreGive(_mutex);

    logInfo("trip counters reset");
}

void EnergyMeter::resetLifetime()
{
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    _lifeDistKm = 0.0f;
    _lifeOutWh = 0.0f;
    _lifeInWh = 0.0f;
    xSemaphoreGive(_mutex);

    save();
    logInfo("lifetime counters reset");
}

void EnergyMeter::load()
{
    energyPrefs.begin(ENERGY_NS, true);   // read-only
    _lifeDistKm = energyPrefs.getFloat("lifeKm",  0.0f);
    _lifeOutWh  = energyPrefs.getFloat("lifeOut", 0.0f);
    _lifeInWh   = energyPrefs.getFloat("lifeIn",  0.0f);
    energyPrefs.end();
}

void EnergyMeter::save()
{
    float km, out, in;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    km = _lifeDistKm;
    out = _lifeOutWh;
    in = _lifeInWh;
    xSemaphoreGive(_mutex);

    energyPrefs.begin(ENERGY_NS, false);
    energyPrefs.putFloat("lifeKm",  km);
    energyPrefs.putFloat("lifeOut", out);
    energyPrefs.putFloat("lifeIn",  in);
    energyPrefs.end();
}

void EnergyMeter::printToSerial()
{
    EnergySnapshot s;
    snapshot(s);

    Serial.println(F("\n  energy meter"));
    Serial.println(F("  ------------------------------------------------------------"));
    if (!_resolved) {
        Serial.println(F("  NOT RUNNING - required signals are missing from the table"));
        Serial.println(F("  need 'packVolt' plus one of 'hvCurr' / 'packCurr'\n"));
        return;
    }
    Serial.printf("  integrating   : %s\n", s.integrating ? "yes" : "no (signals stale or absent)");
    Serial.printf("  pack          : %.1f V   %.2f A   %.2f kW\n", s.packVolt, s.current, s.power);
    Serial.printf("  speed         : %.1f km/h\n", s.speed);
    Serial.println();
    Serial.printf("  trip time     : %lu s\n", (unsigned long)s.tripSeconds);
    Serial.printf("  trip distance : %.3f km\n", s.tripDistanceKm);
    Serial.printf("  trip drawn    : %.1f Wh\n", s.tripEnergyOutWh);
    Serial.printf("  trip regen    : %.1f Wh (%.1f%% of draw)\n", s.tripEnergyInWh, s.tripRegenPercent);
    Serial.printf("  trip net      : %.1f Wh\n", s.tripNetWh);
    Serial.printf("  TRIP AVERAGE  : %.1f Wh/km\n", s.tripAvgWhPerKm);
    Serial.printf("  recent        : %.1f Wh/km (last %.1f km)\n", s.recentWhPerKm, ENERGY_RECENT_WINDOW_KM);
    Serial.println();
    Serial.printf("  lifetime      : %.1f km, %.0f Wh, avg %.1f Wh/km\n",
                  s.lifeDistanceKm, s.lifeEnergyOutWh, s.lifeAvgWhPerKm);
    Serial.println();
}

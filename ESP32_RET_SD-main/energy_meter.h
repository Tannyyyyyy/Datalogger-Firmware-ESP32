/*
 * energy_meter.h
 *
 * EV energy accounting: instantaneous power, cumulative energy in/out, distance
 * travelled, and average consumption rate in Wh/km.
 *
 * HOW IT WORKS
 * ------------
 * Power is voltage times current, sampled every ENERGY_UPDATE_MS and integrated
 * rectangularly. At a 100 ms step against signals that change on a timescale of
 * hundreds of milliseconds, the integration error is far below the accuracy of
 * the current sensor itself, so a fancier integrator would be false precision.
 *
 * Discharge and regeneration are accumulated separately rather than as one
 * signed total. That distinction is the interesting one on an EV: net energy
 * tells you range, but the ratio of regen to discharge tells you how the car is
 * being driven and how well recuperation is working.
 *
 * Distance comes from odometer deltas when that signal exists, because
 * integrating speed accumulates error and misses whatever happened between
 * samples. Speed integration is the automatic fallback.
 *
 * WHY IT RUNS ON CORE 1
 * ---------------------
 * Integration must not stop because the cellular link stalled. The meter is
 * driven from loop(), on the same core as CAN reception, so energy accounting
 * continues through network outages, broker downtime and OTA downloads.
 */

#ifndef ENERGY_METER_H_
#define ENERGY_METER_H_

#include <Arduino.h>

// A consistent snapshot of the meter, copied out under the mutex.
struct EnergySnapshot {
    float    packVolt;          // V
    float    current;           // A, positive = discharge
    float    power;             // kW, positive = discharge
    float    speed;             // km/h

    float    tripDistanceKm;
    float    tripEnergyOutWh;   // drawn from the pack
    float    tripEnergyInWh;    // returned by regen
    float    tripNetWh;         // out - in
    float    tripAvgWhPerKm;    // the headline consumption rate
    float    tripRegenPercent;  // regen as a share of gross discharge

    float    recentWhPerKm;     // rolling average over the last stretch

    float    lifeDistanceKm;
    float    lifeEnergyOutWh;
    float    lifeAvgWhPerKm;

    uint32_t tripSeconds;
    bool     integrating;       // false when signals are stale or missing
};

class EnergyMeter {
public:
    EnergyMeter();

    // Resolves the signal indices it needs and restores totals from NVS.
    bool begin();

    // Call from loop(). Internally rate-limited to ENERGY_UPDATE_MS, so calling
    // it every pass is fine and costs almost nothing.
    void update();

    // Copy the current state out. Safe to call from any task.
    void snapshot(EnergySnapshot &out);

    // Zero the trip accumulators. Lifetime totals are untouched.
    void resetTrip();

    // Zero the lifetime accumulators too. Requires an explicit confirm because
    // it destroys the odometer-referenced history.
    void resetLifetime();

    // Persist totals now (called on key-off and periodically).
    void save();

    void printToSerial();

    bool signalsResolved() const { return _resolved; }

private:
    // Cached signal table indices, resolved once at begin().
    int _idxVolt;
    int _idxPackCurr;
    int _idxHvCurr;
    int _idxSpeed;
    int _idxOdo;
    bool _resolved;

    SemaphoreHandle_t _mutex;

    // Live values.
    float _volt, _current, _powerKw, _speed;
    bool  _integrating;

    // Trip accumulators.
    float    _tripDistKm;
    float    _tripOutWh;
    float    _tripInWh;
    uint32_t _tripStartMs;

    // Lifetime accumulators.
    float _lifeDistKm;
    float _lifeOutWh;
    float _lifeInWh;

    // Rolling-window state for the "recent" figure.
    float _windowDistKm;
    float _windowNetWh;
    float _recentWhPerKm;

    // Odometer tracking.
    float _lastOdo;
    bool  _haveLastOdo;

    uint32_t _lastUpdateMs;
    uint32_t _lastSaveMs;

    // Key-switch edge detection.
    int  _keyPin;
    bool _keyLast;
    uint32_t _keyDebounceMs;

    void load();
    void serviceKeySwitch();
    static float safeDiv(float numerator, float denominator, float minDenominator);
};

extern EnergyMeter energyMeter;

#endif /* ENERGY_METER_H_ */

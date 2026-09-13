/*
 * signal_db.cpp
 *
 * Signal dictionary + decoder. See signal_db.h for the design rationale.
 */

#include <Arduino.h>
#include "signal_db.h"
#include "log.h"

SignalDB signalDB;

// ===========================================================================
//                          >>> EDIT THIS TABLE <<<
// ===========================================================================
// These rows are PLACEHOLDERS. Replace them with the signals from your own DBC
// or from whatever you reverse-engineered in SavvyCAN. Nothing else in the
// firmware needs to change - the decoder, the JSON payload, and the console all
// derive from this table.
//
//   name          id     ext  start  len  endian        signed  scale     offset  unit
// ---------------------------------------------------------------------------
const SignalDef SIGNAL_TABLE[] = {
    // --- Example: engine data, big-endian, typical of OEM powertrain frames ---
    { "rpm",        0x0C4, false,  7,  16, SIG_MOTOROLA, false,  0.25f,     0.0f,  "rpm"  },
    { "speed",      0x0C4, false, 23,  16, SIG_MOTOROLA, false,  0.01f,     0.0f,  "km/h" },
    { "throttle",   0x0C4, false, 39,   8, SIG_MOTOROLA, false,  0.392157f, 0.0f,  "%"    },

    // --- Example: temperatures, note the classic -40 offset ---
    { "coolant",    0x1D0, false,  7,   8, SIG_MOTOROLA, false,  1.0f,    -40.0f,  "degC" },
    { "intakeAir",  0x1D0, false, 15,   8, SIG_MOTOROLA, false,  1.0f,    -40.0f,  "degC" },

    // --- Example: little-endian battery frame with a signed current ---
    { "packVolt",   0x200, false,  0,  16, SIG_INTEL,    false,  0.01f,     0.0f,  "V"    },
    { "packAmp",    0x200, false, 16,  16, SIG_INTEL,    true,   0.1f,      0.0f,  "A"    },
    { "soc",        0x200, false, 32,   8, SIG_INTEL,    false,  0.5f,      0.0f,  "%"    },

    // --- Example: a single status bit ---
    { "brakeOn",    0x224, false,  0,   1, SIG_INTEL,    false,  1.0f,      0.0f,  ""     },
};

const size_t SIGNAL_TABLE_LEN = sizeof(SIGNAL_TABLE) / sizeof(SIGNAL_TABLE[0]);
// ===========================================================================

SignalDB::SignalDB()
    : _mutex(nullptr), _framesSeen(0), _framesMatched(0), _lockFailures(0)
{
    memset(_values, 0, sizeof(_values));
}

bool SignalDB::begin()
{
    if (SIGNAL_TABLE_LEN > MAX_SIGNALS) {
        logError("SIGNAL_TABLE has %u rows but MAX_SIGNALS is %u - raise the limit",
                 (unsigned)SIGNAL_TABLE_LEN, (unsigned)MAX_SIGNALS);
        return false;
    }

    // Validate the table once at boot rather than discovering a bad row as a
    // garbage value on a dashboard three weeks later.
    for (size_t i = 0; i < SIGNAL_TABLE_LEN; i++) {
        const SignalDef &d = SIGNAL_TABLE[i];
        if (d.bitLength == 0 || d.bitLength > 64) {
            logError("signal %s: bitLength %u is out of range 1..64", d.name, d.bitLength);
            return false;
        }
        if (d.startBit > 63) {
            logError("signal %s: startBit %u is out of range 0..63", d.name, d.startBit);
            return false;
        }
    }

    _mutex = xSemaphoreCreateMutex();
    if (_mutex == nullptr) {
        logError("failed to create signal table mutex");
        return false;
    }

    memset(_values, 0, sizeof(_values));
    logInfo("signal table ready: %u signals", (unsigned)SIGNAL_TABLE_LEN);
    return true;
}

bool SignalDB::decodeFrame(const CAN_FRAME &frame)
{
    _framesSeen++;

    // Remote-transmission requests carry no payload.
    if (frame.rtr) return false;

    bool matched = false;
    const uint32_t nowMs = millis();

    // Linear scan. At a few dozen signals and a few thousand frames per second
    // this is on the order of 100k integer compares per second - negligible on a
    // 240 MHz core. If the table ever grows past a couple of hundred rows,
    // replace this with an id-sorted binary search.
    for (size_t i = 0; i < SIGNAL_TABLE_LEN; i++) {
        const SignalDef &def = SIGNAL_TABLE[i];
        if (def.canId != frame.id || def.extended != (bool)frame.extended) continue;

        // A short frame cannot contain a signal that extends past its payload.
        if (!signalFits(def, frame.length)) continue;

        const float physical =
            applyScale(extractRaw(frame.data.uint8, frame.length, def), def);

        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _values[i].value = physical;
            _values[i].lastUpdateMs = nowMs;
            _values[i].updatesSincePublish++;
            _values[i].everSeen = true;
            xSemaphoreGive(_mutex);
            matched = true;
        } else {
            // The publisher only holds the mutex for a memcpy, so this should
            // never happen. Counted rather than logged to keep the hot path fast.
            _lockFailures++;
        }
    }

    if (matched) _framesMatched++;
    return matched;
}

size_t SignalDB::snapshot(SignalValue *out, size_t maxCount, bool clearCounters)
{
    const size_t n = (SIGNAL_TABLE_LEN < maxCount) ? SIGNAL_TABLE_LEN : maxCount;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        _lockFailures++;
        return 0;
    }

    memcpy(out, _values, n * sizeof(SignalValue));
    if (clearCounters) {
        for (size_t i = 0; i < n; i++) _values[i].updatesSincePublish = 0;
    }
    xSemaphoreGive(_mutex);

    return n;
}

size_t SignalDB::count() const
{
    return SIGNAL_TABLE_LEN;
}

const SignalDef *SignalDB::definition(size_t index) const
{
    if (index >= SIGNAL_TABLE_LEN) return nullptr;
    return &SIGNAL_TABLE[index];
}

int SignalDB::findIndex(const char *name) const
{
    for (size_t i = 0; i < SIGNAL_TABLE_LEN; i++) {
        if (strcmp(SIGNAL_TABLE[i].name, name) == 0) return (int)i;
    }
    return -1;
}

bool SignalDB::getValue(int index, float &valueOut, uint32_t &ageMsOut)
{
    if (index < 0 || (size_t)index >= SIGNAL_TABLE_LEN) return false;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        _lockFailures++;
        return false;
    }
    const SignalValue v = _values[index];
    xSemaphoreGive(_mutex);

    if (!v.everSeen) return false;
    valueOut = v.value;
    ageMsOut = millis() - v.lastUpdateMs;
    return true;
}

void SignalDB::resetCounters()
{
    _framesSeen = 0;
    _framesMatched = 0;
    _lockFailures = 0;
}

void SignalDB::dumpToSerial()
{
    SignalValue snap[MAX_SIGNALS];
    const size_t n = snapshot(snap, MAX_SIGNALS, false);

    Serial.println(F("\n  signal              id      value        unit   age(ms)"));
    Serial.println(F("  ------------------------------------------------------------"));
    for (size_t i = 0; i < n; i++) {
        const SignalDef &d = SIGNAL_TABLE[i];
        if (!snap[i].everSeen) {
            Serial.printf("  %-18s 0x%03X   %-12s %-6s %s\n",
                          d.name, (unsigned)d.canId, "--", d.unit, "never seen");
        } else {
            Serial.printf("  %-18s 0x%03X   %-12.3f %-6s %lu\n",
                          d.name, (unsigned)d.canId, snap[i].value, d.unit,
                          (unsigned long)(millis() - snap[i].lastUpdateMs));
        }
    }
    Serial.printf("\n  frames seen: %lu   matched: %lu   lock failures: %lu\n\n",
                  (unsigned long)framesSeen(), (unsigned long)framesMatched(),
                  (unsigned long)lockFailures());
}

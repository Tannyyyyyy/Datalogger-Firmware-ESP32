/*
 * signal_db.h
 *
 * DBC-style CAN signal decoder plus a "latest value" table.
 *
 * WHY THIS DESIGN
 * ---------------
 * A 500 kbit/s bus in a running vehicle produces roughly 2,000-4,000 frames per
 * second. Forwarding those one-for-one over a cellular link is not possible, so
 * instead of queueing frames we collapse them: each incoming frame is decoded
 * immediately into named engineering values, and only the MOST RECENT value per
 * signal is retained. The publisher then transmits that small table on a timer.
 *
 * The consequence is worth stating plainly: bus load no longer affects
 * bandwidth, RAM, or CPU headroom. A busier bus simply overwrites the same table
 * more often. There is no queue to overflow and no backpressure path.
 *
 * The bit extraction and scaling live in signal_bits.h so they can be tested on
 * a PC; this file is the table, the mutex and the plumbing.
 */

#ifndef SIGNAL_DB_H_
#define SIGNAL_DB_H_

#include <Arduino.h>
#include "esp32_can.h"
#include "signal_bits.h"

// Hard ceiling on table size. Raise if you need more; cost is ~16 bytes of RAM
// per signal plus one comparison per received frame.
#define MAX_SIGNALS 96

// Mutable state per signal. Lives in RAM, written by the CAN task, read by the
// publisher task under a mutex.
struct SignalValue {
    float    value;
    uint32_t lastUpdateMs;
    uint32_t updatesSincePublish;
    bool     everSeen;
};

class SignalDB {
public:
    SignalDB();

    // Allocates the mutex and clears the value table. Call once from setup().
    bool begin();

    // Decode one frame into the value table. Returns true if at least one signal
    // matched. Called from the CAN loop - keep it cheap.
    bool decodeFrame(const CAN_FRAME &frame);

    // Copy the whole value table out under the mutex. Pass clearCounters = true
    // to atomically mark everything as "published". Returns the signal count.
    size_t snapshot(SignalValue *out, size_t maxCount, bool clearCounters);

    size_t count() const;
    const SignalDef *definition(size_t index) const;

    // Look a signal up by name. Returns -1 if it is not in the table. Resolve
    // this ONCE at boot and keep the index - it is a string compare per call.
    int findIndex(const char *name) const;

    // Read one signal. Returns false if the index is bad or no frame carrying
    // that signal has ever arrived. ageMs tells you how stale the value is,
    // which matters when integrating: a sensor that stopped reporting must not
    // keep contributing its last value forever.
    bool getValue(int index, float &valueOut, uint32_t &ageMsOut);

    // Diagnostics, surfaced by the serial console and the status topic.
    uint32_t framesSeen() const      { return _framesSeen; }
    uint32_t framesMatched() const   { return _framesMatched; }
    uint32_t lockFailures() const    { return _lockFailures; }
    void     resetCounters();

    // Pretty-print the live table to Serial (console "signals" command).
    void dumpToSerial();

private:
    SignalValue        _values[MAX_SIGNALS];
    SemaphoreHandle_t  _mutex;
    volatile uint32_t  _framesSeen;
    volatile uint32_t  _framesMatched;
    volatile uint32_t  _lockFailures;
};

extern SignalDB signalDB;

// The dictionary itself lives in signal_db.cpp so you only edit one place.
extern const SignalDef SIGNAL_TABLE[];
extern const size_t    SIGNAL_TABLE_LEN;

#endif /* SIGNAL_DB_H_ */

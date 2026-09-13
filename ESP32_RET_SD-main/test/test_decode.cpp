/*
 * Host-side check for the CAN bit decoder.
 *
 *   g++ -std=c++11 -o test_decode test/test_decode.cpp && ./test_decode
 *
 * This folder is not compiled by the Arduino build (only the sketch root and
 * src/ are), so it costs the firmware nothing.
 *
 * Only signal_bits.h is covered, because that is the one place where a bug
 * produces a plausible wrong number instead of an obvious failure - everything
 * else in the firmware fails loudly or needs real hardware.
 */

#include <cassert>
#include <cmath>
#include <cstdio>

#include "../signal_bits.h"

static bool near(float a, float b) { return std::fabs(a - b) < 1e-4f; }

int main()
{
    const uint8_t data[8] = { 0x12, 0x34, 0xAB, 0xCD, 0x00, 0x7F, 0xFF, 0xFF };

    // --- Motorola (big-endian), the DBC sawtooth convention -----------------
    // startBit 7 = MSB of byte 0, so a 16-bit signal is bytes 0..1.
    SignalDef be16 = { "be16", 0x100, false, 7, 16, SIG_MOTOROLA, false, 1.0f, 0.0f, "" };
    assert(extractRaw(data, 8, be16) == 0x1234);

    // startBit 23 = MSB of byte 2 -> bytes 2..3.
    SignalDef be16b = { "be16b", 0x100, false, 23, 16, SIG_MOTOROLA, false, 1.0f, 0.0f, "" };
    assert(extractRaw(data, 8, be16b) == 0xABCD);

    // 8-bit at byte 5.
    SignalDef be8 = { "be8", 0x100, false, 47, 8, SIG_MOTOROLA, false, 1.0f, 0.0f, "" };
    assert(extractRaw(data, 8, be8) == 0x7F);

    // --- Intel (little-endian) ---------------------------------------------
    // startBit 0, 16 bits over a 2-byte payload: byte 0 is least significant.
    const uint8_t le[2] = { 0x34, 0x12 };
    SignalDef le16 = { "le16", 0x100, false, 0, 16, SIG_INTEL, false, 1.0f, 0.0f, "" };
    assert(extractRaw(le, 2, le16) == 0x1234);

    // Single status bit, bit 1 of byte 0.
    const uint8_t flags[1] = { 0x02 };
    SignalDef bit1 = { "bit1", 0x103, false, 1, 1, SIG_INTEL, false, 1.0f, 0.0f, "" };
    assert(extractRaw(flags, 1, bit1) == 1);
    SignalDef bit0 = { "bit0", 0x103, false, 0, 1, SIG_INTEL, false, 1.0f, 0.0f, "" };
    assert(extractRaw(flags, 1, bit0) == 0);

    // --- scaling and sign extension ----------------------------------------
    SignalDef volts = { "v", 0x101, false, 7, 16, SIG_MOTOROLA, false, 0.1f, 0.0f, "V" };
    assert(near(applyScale(extractRaw(data, 8, volts), volts), 466.0f));   // 0x1234 * 0.1

    // 0xFFFF as a signed 16-bit is -1, not 65535. Getting this wrong turns a
    // trickle of regen into a 6.5 kA discharge.
    const uint8_t neg[2] = { 0xFF, 0xFF };
    SignalDef amps = { "i", 0x101, false, 7, 16, SIG_MOTOROLA, true, 0.1f, 0.0f, "A" };
    assert(near(applyScale(extractRaw(neg, 2, amps), amps), -0.1f));

    SignalDef degC = { "t", 0x102, false, 7, 8, SIG_MOTOROLA, false, 1.0f, -40.0f, "degC" };
    const uint8_t t[1] = { 50 };
    assert(near(applyScale(extractRaw(t, 1, degC), degC), 10.0f));

    // --- short-frame rejection ---------------------------------------------
    // A Motorola signal that straddles the end of a short payload must be
    // rejected, not silently decoded against zero padding.
    assert(signalFits(be8, 6));
    assert(!signalFits(be8, 5));
    assert(signalFits(be16, 2));
    assert(!signalFits(be16, 1));

    SignalDef be32 = { "be32", 0x3C0, false, 7, 32, SIG_MOTOROLA, false, 1.0f, 0.0f, "" };
    assert(signalFits(be32, 4));
    assert(!signalFits(be32, 3));
    assert(extractRaw(data, 4, be32) == 0x1234ABCD);

    std::printf("all decoder checks passed\n");
    return 0;
}

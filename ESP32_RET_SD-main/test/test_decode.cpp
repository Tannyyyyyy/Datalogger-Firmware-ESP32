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

    // --- display a2101.dbc layouts (mirrors SIGNAL_TABLE) -------------------
    // VCU_GW_0 0xA2100: speed=60, power raw 0x8000 -> 0 kW, temps 80-40=40,
    // batt_voltage 400, byte 7 = 0b00001110 -> warning 1, fault 1, state 2.
    const uint8_t gw0[8] = { 0x3C, 0x00, 0x80, 0x50, 0x50, 0x90, 0x01, 0x0E };
    SignalDef sp   = { "speed",         0xA2100, true,  0,  8, SIG_INTEL,    true,  1.0f,     0.0f, "" };
    SignalDef pw   = { "power",         0xA2100, true,  8, 16, SIG_INTEL,    true,  1.0f, 32768.0f, "" };
    SignalDef bt1  = { "batt_temp_1",   0xA2100, true, 31,  8, SIG_MOTOROLA, true,  1.0f,   -40.0f, "" };
    SignalDef mcut = { "mcu_temp",      0xA2100, true, 39,  8, SIG_MOTOROLA, true,  1.0f,   -40.0f, "" };
    SignalDef bv   = { "batt_voltage",  0xA2100, true, 40, 16, SIG_INTEL,    true,  1.0f,     0.0f, "" };
    SignalDef vs   = { "vehicle_state", 0xA2100, true, 57,  2, SIG_MOTOROLA, false, 1.0f,     0.0f, "" };
    SignalDef flt  = { "fault",         0xA2100, true, 58,  1, SIG_MOTOROLA, false, 1.0f,     0.0f, "" };
    SignalDef wrn  = { "warning",       0xA2100, true, 59,  1, SIG_MOTOROLA, false, 1.0f,     0.0f, "" };
    assert(near(applyScale(extractRaw(gw0, 8, sp),   sp),    60.0f));
    assert(near(applyScale(extractRaw(gw0, 8, pw),   pw),     0.0f));
    assert(near(applyScale(extractRaw(gw0, 8, bt1),  bt1),   40.0f));
    assert(near(applyScale(extractRaw(gw0, 8, mcut), mcut),  40.0f));
    assert(near(applyScale(extractRaw(gw0, 8, bv),   bv),   400.0f));
    assert(near(applyScale(extractRaw(gw0, 8, vs),   vs),     2.0f));
    assert(near(applyScale(extractRaw(gw0, 8, flt),  flt),    1.0f));
    assert(near(applyScale(extractRaw(gw0, 8, wrn),  wrn),    1.0f));

    // VCU_GW_1 0xA2101, DLC 5: odo 123456 km, gear 3 (high nibble), mode 2, soc 85.
    const uint8_t gw1[5] = { 0x40, 0xE2, 0x01, 0x32, 0x55 };
    SignalDef odo = { "distance_total", 0xA2101, true,  0, 24, SIG_INTEL,    true,  1.0f, 0.0f, "" };
    SignalDef dm  = { "drive_mode",     0xA2101, true, 27,  4, SIG_MOTOROLA, false, 1.0f, 0.0f, "" };
    SignalDef cg  = { "current_gear",   0xA2101, true, 31,  4, SIG_MOTOROLA, false, 1.0f, 0.0f, "" };
    SignalDef soc = { "soc",            0xA2101, true, 39,  8, SIG_MOTOROLA, true,  1.0f, 0.0f, "" };
    assert(signalFits(soc, 5));
    assert(!signalFits(soc, 4));
    assert(near(applyScale(extractRaw(gw1, 5, odo), odo), 123456.0f));
    assert(near(applyScale(extractRaw(gw1, 5, dm),  dm),       2.0f));
    assert(near(applyScale(extractRaw(gw1, 5, cg),  cg),       3.0f));
    assert(near(applyScale(extractRaw(gw1, 5, soc), soc),     85.0f));

    std::printf("all decoder checks passed\n");
    return 0;
}

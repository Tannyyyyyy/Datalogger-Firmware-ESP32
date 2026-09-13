/*
 * signal_bits.h
 *
 * The bit-twiddling half of the CAN signal decoder: the signal definition, the
 * raw bit-field extraction, and the scaling. Deliberately free of Arduino and
 * FreeRTOS so test/test_decode.cpp can compile and check it on a PC - this is
 * the one piece of the firmware where a bug produces a plausible-looking wrong
 * number rather than an obvious failure.
 *
 * BIT NUMBERING
 * -------------
 * startBit follows the standard DBC convention so numbers can be lifted straight
 * out of a .dbc file or Vector / SavvyCAN:
 *   SIG_INTEL    (little-endian) - startBit is the LEAST significant bit.
 *   SIG_MOTOROLA (big-endian)    - startBit is the MOST significant bit, in the
 *                                  usual "sawtooth" numbering where bit 7 is the
 *                                  MSB of byte 0.
 */

#pragma once

#include <stdint.h>

enum SignalEndian : uint8_t {
    SIG_INTEL    = 0,   // little-endian
    SIG_MOTOROLA = 1    // big-endian
};

// One row of the signal dictionary. Stored in flash (const), never mutated.
struct SignalDef {
    const char   *name;       // JSON key. Keep short - it is sent on every batch.
    uint32_t      canId;      // arbitration id this signal lives in
    bool          extended;   // true = 29-bit id, false = 11-bit
    uint8_t       startBit;   // see bit-numbering note above
    uint8_t       bitLength;  // 1..64
    SignalEndian  endian;
    bool          isSigned;   // two's-complement raw value
    float         scale;      // physical = raw * scale + offset
    float         offset;
    const char   *unit;       // informational only; not transmitted
};

// Index of the signal's LAST bit, counted from the start of the payload. Used to
// reject a signal that would run past the end of a short frame - for Motorola
// layouts it is not enough to check the first byte, because a signal straddling
// the end would otherwise silently decode against zero padding.
inline uint16_t signalEndBit(const SignalDef &def)
{
    if (def.endian == SIG_INTEL) {
        return (uint16_t)(def.startBit + def.bitLength);
    }
    return (uint16_t)(((def.startBit / 8) * 8) + (7 - (def.startBit % 8)) + def.bitLength);
}

inline bool signalFits(const SignalDef &def, uint8_t dlc)
{
    return signalEndBit(def) <= (uint16_t)dlc * 8;
}

/*
 * Pull the raw (pre-scaling) bit field out of the payload.
 *
 * Both layouts are handled by assembling the payload into a single 64-bit word
 * with the appropriate byte order and then doing one shift + mask. That is both
 * shorter and faster than walking bit by bit.
 */
inline uint64_t extractRaw(const uint8_t *data, uint8_t dlc, const SignalDef &def)
{
    uint64_t word = 0;

    if (def.endian == SIG_INTEL) {
        // Little-endian: byte 0 is least significant.
        for (int i = (int)dlc - 1; i >= 0; i--) {
            word = (word << 8) | data[i];
        }
        // Unused high bytes are already zero, so a plain shift is correct.
        if (def.startBit >= 64) return 0;
        word >>= def.startBit;
    } else {
        // Big-endian: byte 0 is most significant, left-aligned in the 64-bit word
        // so bit indices count down from the MSB regardless of DLC.
        for (uint8_t i = 0; i < 8; i++) {
            word = (word << 8) | (uint64_t)(i < dlc ? data[i] : 0x00);
        }
        const uint16_t msbIndex = signalEndBit(def) - def.bitLength;
        if (msbIndex + def.bitLength > 64) return 0;   // signal runs off the end
        word >>= (64 - msbIndex - def.bitLength);
    }

    const uint64_t mask = (def.bitLength >= 64) ? UINT64_MAX
                                                : (((uint64_t)1 << def.bitLength) - 1);
    return word & mask;
}

// Raw bits -> engineering units. Sign-extends first, otherwise negative currents
// read as huge positive ones.
inline float applyScale(uint64_t raw, const SignalDef &def)
{
    if (def.isSigned && def.bitLength < 64 &&
        (raw & ((uint64_t)1 << (def.bitLength - 1)))) {
        const int64_t signedRaw = (int64_t)raw - ((int64_t)1 << def.bitLength);
        return (float)((double)signedRaw * def.scale + def.offset);
    }
    return (float)((double)raw * def.scale + def.offset);
}

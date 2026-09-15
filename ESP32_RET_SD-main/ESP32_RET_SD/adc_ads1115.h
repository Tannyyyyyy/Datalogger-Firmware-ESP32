/*
 * adc_ads1115.h
 *
 * ADS1115 16-bit I2C ADC - four single-ended channels, round-robined.
 *
 * NON-BLOCKING BY DESIGN
 * ----------------------
 * A single-shot conversion at the default 128 SPS takes about 8 ms. Waiting for
 * that inline would stall the CAN drain loop for 32 ms per full sweep of four
 * channels, which is a long time to not be reading a 500 kbit/s bus. So the
 * driver is a state machine: start a conversion, return immediately, and collect
 * the result on a later call once enough time has passed.
 *
 * CHANNEL SCALING
 * ---------------
 * Each channel carries its own PGA range and a linear conversion to engineering
 * units, in the same data-driven spirit as the CAN signal table:
 *
 *     value = volts * scale + offset
 *
 * So a 100 A / 1 V current transformer is scale = 100.0, unit "A"; an LM35 at
 * 10 mV per degree is scale = 100.0, unit "degC"; a raw voltage is scale = 1.0.
 *
 * INPUT RANGE WARNING
 * -------------------
 * No input may exceed VDD + 0.3 V, whatever the PGA says. On a 3.3 V supply the
 * +/-6.144 V and +/-4.096 V settings cannot be reached - they only throw away
 * resolution. Anything above 3.3 V needs a divider in front of the pin.
 */

#ifndef ADC_ADS1115_H_
#define ADC_ADS1115_H_

#include <Arduino.h>

#define ADS_NUM_CHANNELS 4

struct AdcChannelDef {
    const char *name;      // JSON key, "" disables the channel
    uint8_t     gain;      // ADS_GAIN_* index from config.h
    float       scale;     // engineering units per volt
    float       offset;
    const char *unit;
};

struct AdcSample {
    float    volts[ADS_NUM_CHANNELS];
    float    value[ADS_NUM_CHANNELS];   // after scale/offset
    bool     fresh[ADS_NUM_CHANNELS];   // has this channel ever converted
    bool     present;
    uint32_t conversions;
    uint32_t errors;
};

class AdcAds1115 {
public:
    AdcAds1115();

    // Probes the part by reading back the config register. Returns false if it
    // does not answer; the rest of the firmware carries on without it.
    bool begin();

    // Call from loop(). Non-blocking - advances the conversion state machine.
    void update();

    void snapshot(AdcSample &out);

    bool present() const { return _present; }
    void printToSerial();

private:
    enum State { IDLE, CONVERTING };

    bool     _present;
    State    _state;
    uint8_t  _channel;          // channel currently converting
    uint32_t _startedMs;
    uint32_t _nextStartMs;
    uint16_t _convertMs;

    AdcSample _s;
    SemaphoreHandle_t _mutex;

    bool writeReg16(uint8_t reg, uint16_t val);
    bool readReg16(uint8_t reg, uint16_t &val);
    bool startConversion(uint8_t channel);
    int8_t nextEnabledChannel(uint8_t from) const;

    static float fullScaleVolts(uint8_t gain);
};

extern AdcAds1115 adc;
extern const AdcChannelDef ADC_CHANNELS[ADS_NUM_CHANNELS];

#endif /* ADC_ADS1115_H_ */

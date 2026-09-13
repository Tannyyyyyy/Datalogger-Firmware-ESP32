/*
 * adc_ads1115.cpp
 *
 * See adc_ads1115.h for the state-machine rationale and the input range warning.
 */

#include <Arduino.h>
#include <Wire.h>
#include "adc_ads1115.h"
#include "config.h"
#include "log.h"

AdcAds1115 adc;

static const uint8_t REG_CONVERT = 0x00;
static const uint8_t REG_CONFIG  = 0x01;

// ===========================================================================
//                       >>> EDIT THIS CHANNEL TABLE <<<
// ===========================================================================
// Names map to the block diagram's temp sensor and CT sensors. The scale values
// are PLACEHOLDERS - set them from your actual sensor datasheets.
//
// Set a name to "" to disable a channel and skip its conversion entirely.
//
//   name        gain              scale     offset   unit
// ---------------------------------------------------------------------------
const AdcChannelDef ADC_CHANNELS[ADS_NUM_CHANNELS] = {
    // A0 - temperature sensor. Example is an LM35 at 10 mV/degC, so one volt
    // is 100 degC and there is no offset. A TMP36 would be scale 100, offset -50.
    { "auxTemp", ADS_GAIN_4V096, 100.0f,   0.0f, "degC" },

    // A1/A2 - CT current sensors. A unit that outputs 0-1 V for 0-100 A is
    // scale 100. A bidirectional sensor centred on VDD/2 needs the offset set
    // to minus half its span, e.g. scale 200, offset -100 for +/-100 A on
    // 0-2 V centred at 1 V.
    { "ctA",     ADS_GAIN_4V096, 100.0f,   0.0f, "A" },
    { "ctB",     ADS_GAIN_4V096, 100.0f,   0.0f, "A" },

    // A3 - unused. Rename to enable; "" means the driver never converts it.
    { "",        ADS_GAIN_4V096,   1.0f,   0.0f, "V" },
};
// ===========================================================================

AdcAds1115::AdcAds1115()
    : _present(false), _state(IDLE), _channel(0),
      _startedMs(0), _nextStartMs(0), _convertMs(12), _mutex(nullptr)
{
    memset(&_s, 0, sizeof(_s));
}

float AdcAds1115::fullScaleVolts(uint8_t gain)
{
    switch (gain) {
        case ADS_GAIN_6V144: return 6.144f;
        case ADS_GAIN_4V096: return 4.096f;
        case ADS_GAIN_2V048: return 2.048f;
        case ADS_GAIN_1V024: return 1.024f;
        case ADS_GAIN_0V512: return 0.512f;
        case ADS_GAIN_0V256: return 0.256f;
        default:             return 2.048f;   // the part's own default
    }
}

bool AdcAds1115::writeReg16(uint8_t reg, uint16_t val)
{
    Wire.beginTransmission(ADS_I2C_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    return Wire.endTransmission() == 0;
}

bool AdcAds1115::readReg16(uint8_t reg, uint16_t &val)
{
    Wire.beginTransmission(ADS_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;      // repeated start

    if (Wire.requestFrom((uint8_t)ADS_I2C_ADDR, (uint8_t)2) != 2) return false;
    const uint8_t hi = Wire.read();
    const uint8_t lo = Wire.read();
    val = ((uint16_t)hi << 8) | lo;
    return true;
}

int8_t AdcAds1115::nextEnabledChannel(uint8_t from) const
{
    for (uint8_t i = 0; i < ADS_NUM_CHANNELS; i++) {
        const uint8_t c = (uint8_t)((from + i) % ADS_NUM_CHANNELS);
        if (ADC_CHANNELS[c].name != nullptr && ADC_CHANNELS[c].name[0] != '\0') {
            return (int8_t)c;
        }
    }
    return -1;
}

bool AdcAds1115::begin()
{
    _mutex = xSemaphoreCreateMutex();
    if (_mutex == nullptr) return false;

    // The config register reads back a known-shaped value on a live part, which
    // is a more meaningful probe than a bare address ACK.
    uint16_t cfg = 0;
    if (!readReg16(REG_CONFIG, cfg)) {
        logWarn("ADS1115: no response at 0x%02X - check wiring, pull-ups and ADDR",
                     ADS_I2C_ADDR);
        return false;
    }

    if (nextEnabledChannel(0) < 0) {
        logWarn("ADS1115: found at 0x%02X but every channel is disabled",
                     ADS_I2C_ADDR);
        return false;
    }

    // 128 SPS -> ~7.8 ms. Allow generous margin; we are not in a hurry and a
    // premature read returns the previous conversion, which is worse than late.
    _convertMs = 12;

    _present = true;
    _s.present = true;
    _state = IDLE;
    _channel = (uint8_t)nextEnabledChannel(0);
    _nextStartMs = millis();

    logInfo("ADS1115 ready at 0x%02X (config 0x%04X)", ADS_I2C_ADDR, cfg);
    for (uint8_t c = 0; c < ADS_NUM_CHANNELS; c++) {
        if (ADC_CHANNELS[c].name && ADC_CHANNELS[c].name[0]) {
            logInfo("  A%u -> %s (+/-%.3f V, x%.4g %s)", c, ADC_CHANNELS[c].name,
                         fullScaleVolts(ADC_CHANNELS[c].gain),
                         ADC_CHANNELS[c].scale, ADC_CHANNELS[c].unit);
        }
    }
    return true;
}

bool AdcAds1115::startConversion(uint8_t channel)
{
    // OS=1 begin conversion, MUX=100+ch single-ended, MODE=1 single-shot,
    // DR=100 (128 SPS), comparator disabled (COMP_QUE=11).
    const uint16_t cfg = (uint16_t)(0x8000
                       | ((uint16_t)(0x04 | channel) << 12)
                       | ((uint16_t)(ADC_CHANNELS[channel].gain & 0x07) << 9)
                       | (1u << 8)
                       | (4u << 5)
                       | 0x0003);
    return writeReg16(REG_CONFIG, cfg);
}

void AdcAds1115::update()
{
    if (!_present) return;

    const uint32_t now = millis();

    if (_state == IDLE) {
        if ((int32_t)(now - _nextStartMs) < 0) return;

        if (!startConversion(_channel)) {
            if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                _s.errors++;
                xSemaphoreGive(_mutex);
            }
            // Skip this channel rather than jamming on it.
            const int8_t nxt = nextEnabledChannel((uint8_t)(_channel + 1));
            _channel = (nxt < 0) ? 0 : (uint8_t)nxt;
            _nextStartMs = now + ADS_SAMPLE_MS;
            return;
        }

        _startedMs = now;
        _state = CONVERTING;
        return;
    }

    // CONVERTING
    if (now - _startedMs < _convertMs) return;

    uint16_t raw = 0;
    if (readReg16(REG_CONVERT, raw)) {
        const int16_t signedRaw = (int16_t)raw;
        const float fs = fullScaleVolts(ADC_CHANNELS[_channel].gain);
        const float volts = (float)signedRaw * fs / 32768.0f;

        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _s.volts[_channel] = volts;
            _s.value[_channel] = volts * ADC_CHANNELS[_channel].scale
                               + ADC_CHANNELS[_channel].offset;
            _s.fresh[_channel] = true;
            _s.conversions++;
            xSemaphoreGive(_mutex);
        }
    } else {
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _s.errors++;
            xSemaphoreGive(_mutex);
        }
    }

    const int8_t nxt = nextEnabledChannel((uint8_t)(_channel + 1));
    _channel = (nxt < 0) ? 0 : (uint8_t)nxt;
    _nextStartMs = now + ADS_SAMPLE_MS;
    _state = IDLE;
}

void AdcAds1115::snapshot(AdcSample &out)
{
    memset(&out, 0, sizeof(out));
    if (!_present) return;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    out = _s;
    xSemaphoreGive(_mutex);
}

void AdcAds1115::printToSerial()
{
    Serial.println(F("\n  ADC (ADS1115)"));
    Serial.println(F("  ------------------------------------------------------------"));
    if (!_present) {
        Serial.printf("  NOT DETECTED at 0x%02X - check SDA %d / SCL %d, pull-ups, ADDR\n\n",
                      ADS_I2C_ADDR, TELEM_I2C_SDA, TELEM_I2C_SCL);
        return;
    }

    AdcSample s;
    snapshot(s);
    Serial.println(F("  ch  name        volts       value       range"));
    for (uint8_t c = 0; c < ADS_NUM_CHANNELS; c++) {
        const AdcChannelDef &d = ADC_CHANNELS[c];
        if (d.name == nullptr || d.name[0] == '\0') {
            Serial.printf("  A%u  %-11s %s\n", c, "(disabled)", "");
        } else if (!s.fresh[c]) {
            Serial.printf("  A%u  %-11s %-11s %-11s +/-%.3f V\n", c, d.name, "--", "--",
                          AdcAds1115::fullScaleVolts(d.gain));
        } else {
            Serial.printf("  A%u  %-11s %+8.5f V  %+9.3f %-5s +/-%.3f V\n",
                          c, d.name, s.volts[c], s.value[c], d.unit,
                          AdcAds1115::fullScaleVolts(d.gain));
        }
    }
    Serial.printf("\n  conversions/errors : %lu / %lu\n\n",
                  (unsigned long)s.conversions, (unsigned long)s.errors);
}

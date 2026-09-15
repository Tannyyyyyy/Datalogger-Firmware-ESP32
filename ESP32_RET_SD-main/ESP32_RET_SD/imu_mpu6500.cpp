/*
 * imu_mpu6500.cpp
 *
 * See imu_mpu6500.h for the mounting convention and the grade-estimate caveat.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include "imu_mpu6500.h"
#include "config.h"
#include "log.h"

ImuMpu6500 imu;

// --- register map ----------------------------------------------------------
static const uint8_t REG_SMPLRT_DIV    = 0x19;
static const uint8_t REG_CONFIG        = 0x1A;
static const uint8_t REG_GYRO_CONFIG   = 0x1B;
static const uint8_t REG_ACCEL_CONFIG  = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2 = 0x1D;
static const uint8_t REG_ACCEL_XOUT_H  = 0x3B;
static const uint8_t REG_PWR_MGMT_1    = 0x6B;
static const uint8_t REG_PWR_MGMT_2    = 0x6C;
static const uint8_t REG_WHO_AM_I      = 0x75;

// WHO_AM_I values that share this exact register map. The MPU9250/9255 add a
// magnetometer behind an auxiliary bus we do not touch, so they work fine here.
static const uint8_t WHOAMI_MPU6500 = 0x70;
static const uint8_t WHOAMI_MPU9250 = 0x71;
static const uint8_t WHOAMI_MPU6515 = 0x74;
static const uint8_t WHOAMI_MPU9255 = 0x73;

static Preferences imuPrefs;
static const char *IMU_NS = "imu";

ImuMpu6500::ImuMpu6500()
    : _present(false), _accelScale(8192.0f), _gyroScale(65.5f),
      _mutex(nullptr), _lastSampleMs(0), _tiltPrimed(false)
{
    _gyroBias[0] = _gyroBias[1] = _gyroBias[2] = 0.0f;
    memset(&_s, 0, sizeof(_s));
}

bool ImuMpu6500::writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool ImuMpu6500::readRegs(uint8_t reg, uint8_t *buf, size_t len)
{
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    // Repeated start: releasing the bus between address and read lets another
    // master (or a glitch) slip in and corrupt the burst.
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((uint8_t)IMU_I2C_ADDR, (uint8_t)len) != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

bool ImuMpu6500::begin()
{
    _mutex = xSemaphoreCreateMutex();
    if (_mutex == nullptr) return false;

    uint8_t who = 0;
    if (!readRegs(REG_WHO_AM_I, &who, 1)) {
        logWarn("MPU6500: no response at 0x%02X - check wiring, pull-ups and AD0",
                     IMU_I2C_ADDR);
        return false;
    }

    if (who == WHOAMI_MPU6500) {
        logInfo("MPU6500 found at 0x%02X", IMU_I2C_ADDR);
    } else if (who == WHOAMI_MPU9250 || who == WHOAMI_MPU9255 || who == WHOAMI_MPU6515) {
        // Same accel/gyro block, different marketing part number.
        logInfo("MPU-series IMU found at 0x%02X (WHO_AM_I 0x%02X) - register "
                     "map is compatible", IMU_I2C_ADDR, who);
    } else {
        logWarn("MPU6500: unexpected WHO_AM_I 0x%02X - continuing anyway, but "
                     "verify the part (0x68 would mean an MPU6050, which needs "
                     "different scaling)", who);
    }

    // Reset, then wait for the internal registers to reload.
    writeReg(REG_PWR_MGMT_1, 0x80);
    delay(100);

    // Auto-select the best available clock (PLL when the gyro is up). Leaving
    // it on the internal oscillator costs noticeable gyro drift.
    if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;
    delay(10);
    writeReg(REG_PWR_MGMT_2, 0x00);        // all six axes enabled

    writeReg(REG_CONFIG, IMU_DLPF_CFG & 0x07);
    writeReg(REG_SMPLRT_DIV, 4);           // 1 kHz / (1 + 4) = 200 Hz internal

    // Full-scale selection. FCHOICE_B stays 0 so the DLPF above is used.
    uint8_t gyroFs;
    switch (IMU_GYRO_RANGE_DPS) {
        case 250:  gyroFs = 0; _gyroScale = 131.0f; break;
        case 1000: gyroFs = 2; _gyroScale = 32.8f;  break;
        case 2000: gyroFs = 3; _gyroScale = 16.4f;  break;
        case 500:
        default:   gyroFs = 1; _gyroScale = 65.5f;  break;
    }
    writeReg(REG_GYRO_CONFIG, (uint8_t)(gyroFs << 3));

    uint8_t accFs;
    switch (IMU_ACCEL_RANGE_G) {
        case 2:  accFs = 0; _accelScale = 16384.0f; break;
        case 8:  accFs = 2; _accelScale = 4096.0f;  break;
        case 16: accFs = 3; _accelScale = 2048.0f;  break;
        case 4:
        default: accFs = 1; _accelScale = 8192.0f;  break;
    }
    writeReg(REG_ACCEL_CONFIG, (uint8_t)(accFs << 3));
    writeReg(REG_ACCEL_CONFIG2, IMU_DLPF_CFG & 0x07);

    delay(20);

    loadBias();
    _present = true;
    _lastSampleMs = millis();

    logInfo("MPU6500 ready: +/-%d g, +/-%d dps, gyro bias %.2f/%.2f/%.2f dps",
                 IMU_ACCEL_RANGE_G, IMU_GYRO_RANGE_DPS,
                 _gyroBias[0], _gyroBias[1], _gyroBias[2]);
    return true;
}

void ImuMpu6500::update()
{
    if (!_present) return;

    const uint32_t now = millis();
    if (now - _lastSampleMs < IMU_SAMPLE_MS) return;
    _lastSampleMs = now;

    // One burst read keeps all six axes and the temperature time-aligned.
    uint8_t raw[14];
    if (!readRegs(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _s.errors++;
            _s.valid = false;
            xSemaphoreGive(_mutex);
        }
        return;
    }

    // Big-endian signed 16-bit, in the order accel / temp / gyro.
    const int16_t axRaw = (int16_t)((raw[0]  << 8) | raw[1]);
    const int16_t ayRaw = (int16_t)((raw[2]  << 8) | raw[3]);
    const int16_t azRaw = (int16_t)((raw[4]  << 8) | raw[5]);
    const int16_t tRaw  = (int16_t)((raw[6]  << 8) | raw[7]);
    const int16_t gxRaw = (int16_t)((raw[8]  << 8) | raw[9]);
    const int16_t gyRaw = (int16_t)((raw[10] << 8) | raw[11]);
    const int16_t gzRaw = (int16_t)((raw[12] << 8) | raw[13]);

    const float ax = axRaw / _accelScale;
    const float ay = ayRaw / _accelScale;
    const float az = azRaw / _accelScale;

    const float gx = gxRaw / _gyroScale - _gyroBias[0];
    const float gy = gyRaw / _gyroScale - _gyroBias[1];
    const float gz = gzRaw / _gyroScale - _gyroBias[2];

    // Datasheet transfer function for the MPU6500 die temperature sensor. This
    // is the silicon temperature, which runs well above ambient - it is useful
    // for spotting a sensor cooking in the sun, not for cabin temperature.
    const float tempC = (tRaw / 333.87f) + 21.0f;

    // Tilt from gravity. atan2 with the magnitude of the other two axes keeps
    // pitch well behaved through +/-90 degrees.
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    const float roll  = atan2f(ay, az) * 57.29578f;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    _s.ax = ax; _s.ay = ay; _s.az = az;
    _s.gx = gx; _s.gy = gy; _s.gz = gz;
    _s.tempC = tempC;

    if (!_tiltPrimed) {
        // Seed the filter with the first reading so it does not spend several
        // seconds crawling up from zero after boot.
        _s.pitchDeg = pitch;
        _s.rollDeg = roll;
        _tiltPrimed = true;
    } else {
        const float a = IMU_TILT_FILTER;
        _s.pitchDeg = a * _s.pitchDeg + (1.0f - a) * pitch;
        _s.rollDeg  = a * _s.rollDeg  + (1.0f - a) * roll;
    }

    _s.gradePercent = tanf(_s.pitchDeg * 0.01745329f) * 100.0f;
    _s.valid = true;
    _s.reads++;

    xSemaphoreGive(_mutex);
}

void ImuMpu6500::snapshot(ImuSample &out)
{
    memset(&out, 0, sizeof(out));
    if (!_present) return;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    out = _s;
    xSemaphoreGive(_mutex);
}

bool ImuMpu6500::calibrateGyro()
{
    if (!_present) {
        Serial.println(F("IMU not present"));
        return false;
    }

    Serial.printf("averaging %d samples - keep the vehicle completely still...\n",
                  IMU_CAL_SAMPLES);

    double sum[3] = {0, 0, 0};
    int taken = 0;

    for (int i = 0; i < IMU_CAL_SAMPLES; i++) {
        uint8_t raw[14];
        if (readRegs(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
            sum[0] += (int16_t)((raw[8]  << 8) | raw[9]);
            sum[1] += (int16_t)((raw[10] << 8) | raw[11]);
            sum[2] += (int16_t)((raw[12] << 8) | raw[13]);
            taken++;
        }
        delay(2);
    }

    if (taken < IMU_CAL_SAMPLES / 2) {
        Serial.println(F("calibration failed - too many I2C errors"));
        return false;
    }

    _gyroBias[0] = (float)(sum[0] / taken) / _gyroScale;
    _gyroBias[1] = (float)(sum[1] / taken) / _gyroScale;
    _gyroBias[2] = (float)(sum[2] / taken) / _gyroScale;

    saveBias();

    Serial.printf("gyro bias = %.3f / %.3f / %.3f dps (saved)\n",
                  _gyroBias[0], _gyroBias[1], _gyroBias[2]);
    return true;
}

void ImuMpu6500::loadBias()
{
    imuPrefs.begin(IMU_NS, true);
    _gyroBias[0] = imuPrefs.getFloat("gbx", 0.0f);
    _gyroBias[1] = imuPrefs.getFloat("gby", 0.0f);
    _gyroBias[2] = imuPrefs.getFloat("gbz", 0.0f);
    imuPrefs.end();
}

void ImuMpu6500::saveBias()
{
    imuPrefs.begin(IMU_NS, false);
    imuPrefs.putFloat("gbx", _gyroBias[0]);
    imuPrefs.putFloat("gby", _gyroBias[1]);
    imuPrefs.putFloat("gbz", _gyroBias[2]);
    imuPrefs.end();
}

void ImuMpu6500::printToSerial()
{
    Serial.println(F("\n  IMU (MPU6500)"));
    Serial.println(F("  ------------------------------------------------------------"));
    if (!_present) {
        Serial.printf("  NOT DETECTED at 0x%02X - check SDA %d / SCL %d, pull-ups, AD0\n\n",
                      IMU_I2C_ADDR, TELEM_I2C_SDA, TELEM_I2C_SCL);
        return;
    }

    ImuSample s;
    snapshot(s);
    Serial.printf("  accel (g)     : X %+7.3f  Y %+7.3f  Z %+7.3f\n", s.ax, s.ay, s.az);
    Serial.printf("  gyro (dps)    : X %+7.2f  Y %+7.2f  Z %+7.2f\n", s.gx, s.gy, s.gz);
    Serial.printf("  gyro bias     : X %+7.3f  Y %+7.3f  Z %+7.3f\n",
                  _gyroBias[0], _gyroBias[1], _gyroBias[2]);
    Serial.printf("  pitch / roll  : %+.2f deg / %+.2f deg\n", s.pitchDeg, s.rollDeg);
    Serial.printf("  grade est.    : %+.2f %%  (estimate - corrupted by braking/accel)\n",
                  s.gradePercent);
    Serial.printf("  die temp      : %.1f degC (silicon, not ambient)\n", s.tempC);
    Serial.printf("  reads/errors  : %lu / %lu\n\n",
                  (unsigned long)s.reads, (unsigned long)s.errors);
}

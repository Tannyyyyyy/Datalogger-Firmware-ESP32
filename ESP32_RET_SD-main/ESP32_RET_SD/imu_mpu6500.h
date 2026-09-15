/*
 * imu_mpu6500.h
 *
 * Minimal MPU6500 driver - accelerometer, gyroscope, die temperature, plus a
 * filtered pitch/roll estimate.
 *
 * WHY A HAND-ROLLED DRIVER
 * ------------------------
 * The MPU6500 register map is small and completely stable, and we need only a
 * burst read of 14 bytes. A general-purpose library would pull in DMP handling,
 * magnetometer support and calibration machinery none of which applies here, in
 * exchange for a dependency that has to be version-matched. Roughly 150 lines is
 * cheaper than that.
 *
 * MOUNTING CONVENTION
 * -------------------
 * The pitch/roll maths assumes the board is mounted with:
 *   +X pointing FORWARD (direction of travel)
 *   +Y pointing LEFT
 *   +Z pointing UP
 * If yours is mounted differently, the raw ax/ay/az values are still correct -
 * only pitch, roll and grade need their axes swapped. See axes() below.
 *
 * ON GRADE ESTIMATION
 * -------------------
 * Road grade from a bare accelerometer is an ESTIMATE, not a measurement. The
 * sensor cannot distinguish the gravity component of a slope from longitudinal
 * acceleration - braking looks like an uphill. The heavy low-pass filter makes
 * the figure usable over a sustained climb but it will lie during hard
 * acceleration. Treat it as context for consumption analysis, not truth.
 */

#ifndef IMU_MPU6500_H_
#define IMU_MPU6500_H_

#include <Arduino.h>

struct ImuSample {
    float ax, ay, az;      // g
    float gx, gy, gz;      // deg/s, gyro bias removed
    float tempC;           // die temperature, not ambient
    float pitchDeg;        // nose up positive, filtered
    float rollDeg;         // right side down positive, filtered
    float gradePercent;    // tan(pitch) * 100, see the caveat above
    bool  valid;           // false if the part has stopped responding
    uint32_t reads;
    uint32_t errors;
};

class ImuMpu6500 {
public:
    ImuMpu6500();

    // Probes WHO_AM_I, configures the part and restores the stored gyro bias.
    // Returns false if the device does not answer - the rest of the firmware
    // carries on regardless, the IMU is not load-bearing.
    bool begin();

    // Call from loop(). Internally rate-limited to IMU_SAMPLE_MS.
    void update();

    void snapshot(ImuSample &out);

    // Average IMU_CAL_SAMPLES readings and store the result as the gyro zero.
    // BLOCKS for roughly a second and the vehicle must be completely still.
    bool calibrateGyro();

    bool present() const { return _present; }
    void printToSerial();

private:
    bool  _present;
    float _accelScale;     // LSB per g
    float _gyroScale;      // LSB per deg/s

    float _gyroBias[3];

    ImuSample _s;
    SemaphoreHandle_t _mutex;
    uint32_t _lastSampleMs;
    bool     _tiltPrimed;

    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t *buf, size_t len);

    void loadBias();
    void saveBias();
};

extern ImuMpu6500 imu;

#endif /* IMU_MPU6500_H_ */

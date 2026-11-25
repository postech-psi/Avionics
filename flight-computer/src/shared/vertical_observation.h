#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <ukf.h>

namespace vertical {
constexpr uint32_t kImuFreshMs = 30;
constexpr uint32_t kBaroFreshMs = 60;
constexpr uint32_t kCadenceMaxIntervalUs = 35000;
constexpr uint32_t kMaxBridgeUs = kBaroFreshMs * 1000;
constexpr uint32_t kVelocityRecoveryUs = 500000;

struct BaroTiming {
    uint32_t readUs;
    uint32_t intervalUs;  // zero on the first accepted observation
    float pairMeanAltitude;
};

// Input is already converted to pad-relative altitude. The pressure range
// check belongs to the sensor driver caller, not this observation history.
class BaroHistory {
public:
    bool accept(uint32_t readUs, float altitude, BaroTiming& timing) {
        if (!std::isfinite(altitude)) return false;
        const uint32_t dt = havePrevious_ ? readUs - lastUs_ : 0u;
        if (havePrevious_ && (dt == 0 ||
            (dt <= 200000u && std::fabs(altitude - lastAltitude_) > 680.0f * (float)dt * 1e-6f))) {
            return false;
        }
        timing = {readUs, dt, havePrevious_ ? (lastAltitude_ + altitude) * 0.5f : altitude};
        lastUs_ = readUs;
        lastAltitude_ = altitude;
        havePrevious_ = true;
        return true;
    }

private:
    uint32_t lastUs_ = 0;
    float lastAltitude_ = 0.0f;
    bool havePrevious_ = false;
};

class StateEstimator {
public:
    void observeImu(uint32_t nowMs, float accelZG) {
        if (haveImu_ && nowMs - lastImuMs_ > kImuFreshMs) imuGap_ = true;
        accelZG_ = accelZG;
        lastImuMs_ = nowMs;
        haveImu_ = true;
    }

    void observeBaro(const BaroTiming& baro, uint32_t nowMs, float altitude) {
        const bool fresh = imuFresh(nowMs);
        const bool continuousImu = fresh && estimateImuFresh_ && !imuGap_;
        // Bridge a short BMP gap only while the IMU stream has stayed fresh.
        // Propagate once over actual elapsed time with the latest acceleration;
        // do not replay the returning barometer measurement as intermediate updates.
        const auto interval = baro.intervalUs;
        const bool anchor = interval == 0 || interval > kMaxBridgeUs ||
                            (interval > kCadenceMaxIntervalUs && !continuousImu);
        const float linearG = fresh ? accelZG_ : 0.0f;
        filter_.rtU.accel_z_in = linearG * 9.81f + 9.81f;  // UKF expects specific force
        filter_.rtU.baro_altitude_in = altitude;
        filter_.step(anchor ? 0.0 : interval / 1000000.0);

        if (anchor) recovering_ = true;
        if (recovering_) {
            if (anchor || !continuousImu) recoveryStartUs_ = baro.readUs;
            else if (baro.readUs - recoveryStartUs_ >= kVelocityRecoveryUs) recovering_ = false;
        }
        lastBaroMs_ = nowMs;
        estimateImuFresh_ = fresh;
        imuGap_ = false;
    }

    float altitude() const { return (float)filter_.rtY.kf_altitude_out; }

    float velocity(uint32_t nowMs) const {
        if (recovering_ || !estimateImuFresh_ || imuGap_ || !imuFresh(nowMs) ||
            nowMs - lastBaroMs_ > kBaroFreshMs) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return (float)filter_.rtY.kf_velocity_out;
    }

private:
    bool imuFresh(uint32_t nowMs) const {
        return haveImu_ && nowMs - lastImuMs_ <= kImuFreshMs;
    }

    kf filter_;
    uint32_t lastImuMs_ = 0;
    uint32_t lastBaroMs_ = 0;
    uint32_t recoveryStartUs_ = 0;
    float accelZG_ = 0.0f;
    bool haveImu_ = false;
    bool imuGap_ = false;
    bool estimateImuFresh_ = false;
    bool recovering_ = true;
};
}  // namespace vertical

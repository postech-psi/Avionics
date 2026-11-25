#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "tailfin_config.h"

namespace tailfin {
class Outputs {
public:
    enum class State { Disabled, AttachFailed, Neutral, Active };

    void begin() {
        if (!kEnableOutputs) return;
        mount_ = kImuFromModelBody;
        if (!normalize(mount_)) return;
        for (unsigned i = 0; i < 4; ++i) {
            const auto& fin = kFinByModelSlot[i];
            // The bundled Mbed Servo cannot store the custom 1000/2000 offsets
            // in its int8 fields. Keep its default attach range and clamp here.
            servos_[i].attach(kConnectorPins[static_cast<int>(fin.connector)]);
            if (!servos_[i].attached()) {
                for (unsigned j = 0; j < i; ++j) servos_[j].detach();
                state_ = State::AttachFailed;
                return;
            }
        }
        write(Result{});
        state_ = State::Neutral;
    }

    void update(const float euler[3], const float gyro[3], float altitude,
                float upwardMps, bool inputsUsable) {
        if (state_ != State::Neutral && state_ != State::Active) return;
        const uint32_t now = micros();
        const float dt = haveTime_ ? (now-lastUs_) * 1e-6f : TS;
        lastUs_ = now;
        haveTime_ = true;

        Result result;
        if (inputsUsable && upwardMps > 0) {
            const auto imu = eulerDegrees(euler[0],euler[1],euler[2]);
            float rates[3];
            gyroToSimulation(gyro,mount_,rates);
            result = controller_.step(imuToSimulation(imu,mount_),rates,
                verticalPressureEstimate(altitude,upwardMps),true,true,dt);
        } else {
            controller_.reset();
        }
        write(result);
        state_ = result.enabled ? State::Active : State::Neutral;
    }

    const char* status() const {
        switch (state_) {
            case State::AttachFailed: return "ATTACH FAILED";
            case State::Neutral: return "NEUTRAL";
            case State::Active: return "ACTIVE";
            default: return "DISABLED";
        }
    }

private:
    void write(const Result& result) {
        for (unsigned i = 0; i < 4; ++i) {
            const auto& fin = kFinByModelSlot[i];
            servos_[i].writeMicroseconds(commandToPulse(result.command[i],fin.neutralUs,
                fin.direction,fin.degreesPerUs,kMinUs,kMaxUs));
        }
    }

    Servo servos_[4];
    Controller controller_;
    Quaternion mount_;
    State state_ = State::Disabled;
    uint32_t lastUs_ = 0;
    bool haveTime_ = false;
};
} // namespace tailfin

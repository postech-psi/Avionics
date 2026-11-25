#pragma once

#include <limits>
#include "tailfin_controller.h"

namespace tailfin {

enum class Connector : int { Unassigned = -1, H7, H8, H9, H10 };

struct FinCalibration {
    Connector connector;
    int neutralUs;
    int direction;
    float degreesPerUs;
};

inline constexpr int kMinUs = 1000;
inline constexpr int kMaxUs = 2000;
inline constexpr bool kEnableOutputs = false;

// Electrical routing only. Model slots are assigned separately below.
inline constexpr int kConnectorPins[] = {6, 5, 4, 3};  // H7, H8, H9, H10
inline constexpr FinCalibration kFinByModelSlot[] = {
    {Connector::Unassigned, 0, 0, 0},  // model azimuth 0 deg
    {Connector::Unassigned, 0, 0, 0},  // model azimuth 90 deg
    {Connector::Unassigned, 0, 0, 0},  // model azimuth 180 deg
    {Connector::Unassigned, 0, 0, 0},  // model azimuth 270 deg
};
// Fill from the installed IMU mounting. Zero magnitude marks an unset rotation.
inline constexpr Quaternion kImuFromModelBody{0,0,0,0};

constexpr bool validOutputConfig() {
    unsigned seen = 0;
    for (const auto& fin : kFinByModelSlot) {
        const int channel = static_cast<int>(fin.connector);
        if (channel < 0 || channel >= 4 || (seen & (1u << channel))) return false;
        seen |= 1u << channel;
        if (fin.direction != -1 && fin.direction != 1) return false;
        if (!(fin.degreesPerUs > 0 && fin.degreesPerUs <= std::numeric_limits<float>::max())) return false;
        const float travelUs = (LIMIT / RAD) / fin.degreesPerUs;
        if (!(fin.neutralUs - travelUs >= kMinUs && fin.neutralUs + travelUs <= kMaxUs)) return false;
    }
    const auto& q = kImuFromModelBody;
    const float normSq = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    // Legacy recovery expects sensor +Z down when the nose is upright.
    // Require the model nose (+X) to map close to sensor -Z for that convention.
    return normSq >= 0.999f && normSq <= 1.001f && 2*(q.x*q.z-q.w*q.y) <= -0.95f;
}

static_assert(!kEnableOutputs || validOutputConfig(),
              "Confirm fin mapping, pulse calibration, and a mount consistent with parachute tilt");
} // namespace tailfin

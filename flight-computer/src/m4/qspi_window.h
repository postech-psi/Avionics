#pragma once

#include <cstdint>

namespace qspi {
enum class StopReason : uint8_t { None, Manual, Landing, Timeout, Limit };
constexpr uint32_t maxSessionMs = 6*60*1000;
constexpr uint32_t landingTailMs = 30*1000;

inline StopReason timedStop(uint32_t now, uint32_t started, bool landed, uint32_t landing) {
    if (uint32_t(now-started) >= maxSessionMs) return StopReason::Timeout;
    if (landed && uint32_t(now-landing) >= landingTailMs) return StopReason::Landing;
    return StopReason::None;
}
} // namespace qspi

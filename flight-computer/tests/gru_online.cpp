#include "../src/m7/apogee_gru.h"

#include <cassert>
#include <cmath>
#include <cstdio>

using namespace apogee_gru;

namespace {
Sample sample(uint32_t elapsedUs) {
    const float t = (float)elapsedUs * 1e-6f;
    return {120.0f + 10.0f * t, 119.0f + 10.0f * t, 12.0f + t,
            -1.02f + 0.01f * t, 1.5f + t, 5.0f - t, 130.0f + 10.0f * t};
}

void checkRegularWindows() {
    Online online;
    // First observation is 3 ms after each exact grid boundary. The matching
    // input must use the previous observation, never the just-arrived one.
    constexpr uint32_t origin = 0xfffffff0u;
    online.start(origin);
    unsigned predictions = 0;
    for (uint32_t t = 3000; t < 2700000; t += 10000) {
        online.observe(origin + t, sample(t), true, true);
        if (!online.advance()) continue;
        const auto result = online.latest();
        const uint32_t grid = result.gridUs - origin;
        assert(grid == 2180000u + predictions * 200000u);
        assert(t - grid == 123000u);
        Kernel expected;
        for (uint32_t u = grid - (kWindow - 1) * kStepUs; u <= grid; u += kStepUs) {
            float x[kFeatures];
            assert(normalize(sample(u - 7000u), x));
            expected.step(x);
        }
        const Sample last = sample(grid - 7000u);
        assert(result.quadraticAltitude == last.quadraticAltitude);
        assert(std::abs(result.ratio - expected.ratio()) < 1e-6f);
        assert(std::abs(result.altitude - energyHeight(last.altitude, last.velocity) * expected.ratio()) < .0001f);
        ++predictions;
    }
    assert(predictions == 2);
    assert(std::isnan(online.altitude(online.latest().gridUs + 400000u)));
}

void checkInvalidRecovery() {
    Online online;
    online.start(0);
    unsigned count = 0;
    for (uint32_t t = 0; t <= 5000000u; t += 10000) {
        const bool valid = t < 2200000u || t >= 2700000u;
        online.observe(t, sample(t), valid, true);
        const bool complete = online.advance();
        assert(!(t >= 2200000u && t < 4800000u && complete));
        if (!valid) assert(std::isnan(online.altitude(t)));
        if (complete) {
            const uint32_t expected = count == 0 ? 1980000u : 4780000u;
            assert(online.latest().gridUs == expected);
            ++count;
        }
    }
    assert(count == 2);
}

void checkSkippedWork() {
    Online online;
    online.start(0);
    for (uint32_t t = 0; t <= 2180000u; t += 10000) {
        online.observe(t, sample(t), true, true);
        // Starve the first job: the producer will overwrite its oldest slot.
        if (t >= 2000000u) assert(!online.advance());
    }
    // Skip several grid points. Cancel the running job and do not publish.
    online.observe(2300000u, sample(2300000u), true, true);
    assert(!online.busy());
    assert(std::isnan(online.altitude(2300000u)));
}
}  // namespace

int main() {
    checkRegularWindows();
    checkInvalidRecovery();
    checkSkippedWork();
    std::puts("GRU online: causal windows, time wrap, slicing, recovery mask and skipped work passed");
}

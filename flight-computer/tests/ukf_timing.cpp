#include "ukf.h"
#include "../src/shared/vertical_observation.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

namespace {
constexpr double kDescentSpeed = -25.0;

void expectNear(const char* label, double actual, double expected, double tolerance) {
    if (!(std::abs(actual - expected) <= tolerance)) {
        std::fprintf(stderr, "%s: expected %.6f, got %.6f\n", label, expected, actual);
        std::exit(EXIT_FAILURE);
    }
}

void expectUnavailable(const char* label, float velocity) {
    if (!std::isnan(velocity)) {
        std::fprintf(stderr, "%s: expected unavailable velocity, got %.6f\n", label, velocity);
        std::exit(EXIT_FAILURE);
    }
}

void observe(kf& filter, double altitude, double dt) {
    // Match the filter's drag at the reference speed to give a constant-speed
    // descent with a known barometric slope, regardless of the sample grid.
    const double drag = 0.6125 * 0.62 * 0.00725 / 4.25;
    filter.rtU = {9.81 + drag * kDescentSpeed * std::abs(kDescentSpeed), altitude};
    filter.step(dt);
}

void checkAnchor(kf& filter, double altitude) {
    observe(filter, altitude, 0.0);
    expectNear("anchor altitude", filter.rtY.kf_altitude_out, altitude, 0.0);
    if (!std::isnan(filter.rtY.kf_velocity_out)) {
        std::fprintf(stderr, "anchor reported a usable velocity\n");
        std::exit(EXIT_FAILURE);
    }
}

void checkDescent(std::initializer_list<double> intervals) {
    kf filter;
    double time = 0.0;
    double altitude = 500.0;
    checkAnchor(filter, altitude);
    while (time < 2.0) {
        for (double dt : intervals) {
            time += dt;
            altitude += kDescentSpeed * dt;
            observe(filter, altitude, dt);
        }
    }
    expectNear("descent velocity", filter.rtY.kf_velocity_out, kDescentSpeed, 0.2);

    // The sample after a gap is an anchor, not a 20 ms step across the gap.
    altitude += kDescentSpeed * 0.12;
    checkAnchor(filter, altitude);
    for (int i = 0; i < 25; ++i) {
        altitude += kDescentSpeed * 0.02;
        observe(filter, altitude, 0.02);
    }
    expectNear("velocity after 500 ms recovery", filter.rtY.kf_velocity_out, kDescentSpeed, 0.2);
}

void checkGapPolicy(uint32_t gapMs, bool continuousImu, bool expectBridge) {
    vertical::StateEstimator estimator;
    vertical::BaroHistory history;
    constexpr float speed = 40.0f;
    constexpr float drag = 0.6125f * 0.62f * 0.00725f / 4.25f;
    constexpr float matchingAccelG = drag * speed * speed / 9.81f;

    auto update = [&](uint32_t ms) {
        const float altitude = 100.0f + speed * (float)ms * 0.001f;
        vertical::BaroTiming observation;
        if (!history.accept(ms * 1000, altitude, observation)) std::exit(EXIT_FAILURE);
        estimator.observeBaro(observation, ms, altitude);
    };
    for (uint32_t ms = 0; ms <= 1000; ms += 20) {
        estimator.observeImu(ms, matchingAccelG);
        update(ms);
        if (ms < 500) expectUnavailable("startup recovery", estimator.velocity(ms));
    }
    expectNear("settled velocity before missing baro", estimator.velocity(1000), speed, 0.2);
    for (uint32_t ms = 1020; continuousImu && ms < 1000 + gapMs; ms += 20) {
        estimator.observeImu(ms, matchingAccelG);
    }
    const uint32_t resumed = 1000 + gapMs;
    estimator.observeImu(resumed, matchingAccelG);
    update(resumed);
    if (expectBridge) {
        expectNear("bridged velocity", estimator.velocity(resumed), speed, 0.2);
        expectNear("bridged altitude", estimator.altitude(), 100.0 + speed * resumed * 0.001, 0.1);
        return;
    }

    for (uint32_t elapsed = 0; elapsed < 500; elapsed += 20) {
        const uint32_t ms = resumed + elapsed;
        if (elapsed != 0) {
            estimator.observeImu(ms, matchingAccelG);
            update(ms);
        }
        expectUnavailable("velocity during re-anchor recovery", estimator.velocity(ms));
    }
    estimator.observeImu(resumed + 500, matchingAccelG);
    update(resumed + 500);
    expectNear("velocity after re-anchor recovery", estimator.velocity(resumed + 500), speed, 0.2);
}
}

int main() {
    checkDescent({0.02});
    checkDescent({0.01, 0.03});
    checkDescent({0.03});
    checkGapPolicy(40, true, true);    // one missed 50 Hz BMP observation
    checkGapPolicy(60, true, true);    // last supported interval
    checkGapPolicy(40, false, false);  // fresh at the end but IMU stream broke
    checkGapPolicy(61, true, false);   // exceeds the bridge horizon
    checkGapPolicy(120, true, false);
    std::puts("UKF timing: nominal, jitter, short gaps and invalid recovery passed");
}

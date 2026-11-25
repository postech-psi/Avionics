#include "apogee_gru.h"
#include "apogee_gru_weights.h"

#include <limits>

namespace apogee_gru {
namespace {
float dot(const float* w, const float* x, std::size_t n, float bias) {
    for (std::size_t i = 0; i < n; ++i) bias += w[i] * x[i];
    return bias;
}

float sigmoid(float x) {
    const float e = std::exp(-std::fabs(x));
    return x >= 0.0f ? 1.0f / (1.0f + e) : e / (1.0f + e);
}
}  // namespace

void normalizeFeatures(const float (&raw)[kFeatures], float (&out)[kFeatures]) {
    for (std::size_t i = 0; i < kFeatures; ++i) {
        out[i] = (raw[i] - weights::kMean[i]) / weights::kScale[i];
    }
}

bool normalize(const Sample& s, float (&out)[kFeatures]) {
    if (!std::isfinite(s.altitude) || !std::isfinite(s.velocity)) return false;
    const float raw[kFeatures] = {
        s.baroAltitude, s.velocity, s.accelZG * 9.80665f, -std::fabs(s.pitchDeg),
        s.gyroZDps * 0.017453292519943295f,
        0.6125f * std::exp(-0.00012f * std::fabs(s.altitude)) * s.velocity * s.velocity,
    };
    normalizeFeatures(raw, out);
    for (float f : out) if (!std::isfinite(f)) return false;
    return true;
}

void Kernel::reset() {
    for (float& h : hidden_) h = 0.0f;
}

void Kernel::step(const float (&x)[kFeatures]) {
    using namespace weights;
    float next[kHidden];
    for (std::size_t i = 0; i < kHidden; ++i) {
        const std::size_t r = i, z = i + kHidden, n = i + 2 * kHidden;
        const auto input = [&](std::size_t gate) {
            return dot(kInputWeight + gate * kFeatures, x, kFeatures, kInputBias[gate]);
        };
        const auto recurrent = [&](std::size_t gate) {
            return dot(kHiddenWeight + gate * kHidden, hidden_, kHidden, kHiddenBias[gate]);
        };
        // PyTorch gate order r, z, n. Its reset gate multiplies the recurrent
        // candidate including the recurrent bias, not the hidden vector.
        const float reset = sigmoid(input(r) + recurrent(r));
        const float update = sigmoid(input(z) + recurrent(z));
        const float candidate = std::tanh(input(n) + reset * recurrent(n));
        next[i] = candidate + update * (hidden_[i] - candidate);
    }
    for (std::size_t i = 0; i < kHidden; ++i) hidden_[i] = next[i];
}

float Kernel::ratio() const {
    using namespace weights;
    float y = kOutputBias[0];
    constexpr std::size_t kHead = sizeof(kHeadBias) / sizeof(float);
    for (std::size_t i = 0; i < kHead; ++i) {
        const float v = dot(kHeadWeight + i * kHidden, hidden_, kHidden, kHeadBias[i]);
        y += kOutputWeight[i] * (v > 0.0f ? v : 0.0f);
    }
    return 1.0f + 0.5f * std::tanh(y);
}

namespace {
constexpr uint32_t kMaxSnapshotAgeUs = 40000;
constexpr uint32_t kPeriodUs = kStepUs * kStride;
constexpr unsigned kStepsPerLoop = 8;
}

void Online::observe(uint32_t nowUs, const Sample& sample, bool valid, bool enabled) {
    if (!valid || !enabled) {
        resultValid_ = false;
        remaining_ = 0;
    }
    if (!valid) filled_ = 0;
    if (busy() && nowUs - jobGridUs_ >= kPeriodUs) remaining_ = 0;
    const uint32_t lag = nowUs - nextGridUs_;
    if (lag < 0x80000000u) {
        if (lag > kMaxSnapshotAgeUs) {
            // A late control loop cannot manufacture a valid history or
            // catch up unbounded work. Preserve phase, not the missed samples.
            const uint32_t missed = lag / kStepUs;
            phase_ = (phase_ + missed) % kStride;
            nextGridUs_ += missed * kStepUs;
            previousValid_ = false;
            filled_ = remaining_ = 0;
            resultValid_ = false;
        }
        while (nowUs - nextGridUs_ <= kMaxSnapshotAgeUs) {
            const bool coincident = nowUs == nextGridUs_;
            const bool heldValid = previousValid_ && nextGridUs_ - previousUs_ <= kMaxSnapshotAgeUs;
            push(nextGridUs_, coincident ? sample : previous_, valid && (coincident || heldValid), enabled);
            nextGridUs_ += kStepUs;
        }
    }
    previous_ = sample;
    previousUs_ = nowUs;
    previousValid_ = valid;
}

void Online::push(uint32_t gridUs, const Sample& sample, bool valid, bool enabled) {
    const bool due = ++phase_ == kStride;
    if (due) phase_ = 0;
    if (busy() && read_ == write_) remaining_ = 0;
    if (!valid || !normalize(sample, history_[write_])) {
        filled_ = remaining_ = 0;
        resultValid_ = false;
        return;
    }
    write_ = next(write_);
    if (filled_ < kWindow) ++filled_;
    if (!due || !enabled || filled_ != kWindow) return;
    const float energy = energyHeight(sample.altitude, sample.velocity);
    if (!(energy > 1e-9f) || !std::isfinite(energy)) {
        resultValid_ = false;
        return;
    }
    kernel_.reset();
    read_ = write_;
    remaining_ = kWindow;
    jobGridUs_ = gridUs;
    jobEnergyHeight_ = energy;
    jobQuadraticAltitude_ = sample.quadraticAltitude;
}

bool Online::advance() {
    if (!busy()) return false;
    for (unsigned i = 0; i < kStepsPerLoop && busy(); ++i) {
        kernel_.step(history_[read_]);
        read_ = next(read_);
        --remaining_;
    }
    if (busy()) return false;
    const float ratio = kernel_.ratio();
    result_ = {jobGridUs_, jobEnergyHeight_ * ratio, ratio, jobQuadraticAltitude_};
    resultValid_ = std::isfinite(result_.altitude);
    return resultValid_;
}

float Online::altitude(uint32_t nowUs) const {
    return resultValid_ && nowUs - result_.gridUs < 2 * kPeriodUs
        ? result_.altitude : std::numeric_limits<float>::quiet_NaN();
}
}  // namespace apogee_gru

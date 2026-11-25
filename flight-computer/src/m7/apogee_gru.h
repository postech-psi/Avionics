#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace apogee_gru {
constexpr std::size_t kFeatures = 6;
constexpr std::size_t kHidden = 32;
constexpr std::size_t kWindow = 100;
constexpr uint32_t kStepUs = 20000;
constexpr unsigned kStride = 10;

struct Sample {
    float baroAltitude;
    float altitude;
    float velocity;
    float accelZG;
    float pitchDeg;
    float gyroZDps;
    float quadraticAltitude = NAN;  // comparison metadata, never a model feature
};

bool normalize(const Sample& sample, float (&out)[kFeatures]);
void normalizeFeatures(const float (&raw)[kFeatures], float (&out)[kFeatures]);

class Kernel {
public:
    void reset();
    void step(const float (&x)[kFeatures]);
    float ratio() const;

private:
    float hidden_[kHidden]{};
};

struct Prediction {
    uint32_t gridUs;
    float altitude;
    float ratio;
    float quadraticAltitude;
};

// One producer on the flight loop. Inference consumes history faster than
// the 50 Hz grid overwrites it; a delayed slice is canceled before overwrite.
class Online {
public:
    void start(uint32_t originUs) { nextGridUs_ = originUs; }
    void observe(uint32_t nowUs, const Sample& sample, bool valid, bool enabled);
    bool busy() const { return remaining_ != 0; }
    uint32_t pendingGridUs() const { return jobGridUs_; }
    bool advance();
    const Prediction& latest() const { return result_; }
    float altitude(uint32_t nowUs) const;

private:
    void push(uint32_t gridUs, const Sample& sample, bool valid, bool enabled);
    static std::size_t next(std::size_t i) { return i + 1 == kWindow ? 0 : i + 1; }

    Kernel kernel_;
    float history_[kWindow][kFeatures]{};
    Sample previous_{};
    uint32_t previousUs_ = 0;
    uint32_t nextGridUs_ = 0;
    uint32_t jobGridUs_ = 0;
    Prediction result_{};
    float jobEnergyHeight_ = 0.0f;
    float jobQuadraticAltitude_ = NAN;
    std::size_t write_ = 0;
    std::size_t read_ = 0;
    std::size_t filled_ = 0;
    std::size_t remaining_ = 0;
    unsigned phase_ = 0;
    bool previousValid_ = false;
    bool resultValid_ = false;
};

inline float energyHeight(float altitude, float velocity) {
    return altitude + velocity * velocity / 19.62f;
}
}  // namespace apogee_gru

// Host-only parity driver: for each window, read h, v and 100 × 6 raw features.
#include "../src/m7/apogee_gru.h"
#include <iomanip>
#include <iostream>
#include <limits>

int main() {
    std::cout << std::setprecision(std::numeric_limits<float>::max_digits10);
    float h, v;
    while (std::cin >> h) {
        if (!(std::cin >> v)) return 1;
        apogee_gru::Kernel model;
        for (unsigned t = 0; t < apogee_gru::kWindow; ++t) {
            float raw[apogee_gru::kFeatures], x[apogee_gru::kFeatures];
            for (float& f : raw) if (!(std::cin >> f)) return 1;
            apogee_gru::normalizeFeatures(raw, x);
            model.step(x);
        }
        const float r = model.ratio();
        std::cout << r << ' ' << apogee_gru::energyHeight(h, v) * r << '\n';
    }
    return std::cin.eof() ? 0 : 1;
}

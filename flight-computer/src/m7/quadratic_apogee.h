#pragma once

#include <cmath>
#include <limits>

// Diagnostic baseline from main's generated quadratic regression. The input
// altitude comes from the new UKF so both predictors see the same estimator.
class QuadraticApogee {
public:
    void observe(double time, double altitude) {
        constexpr double forgetting = 0.965;
        const double x[3] = {time*time,time,1};
        double variance = 0;
        double fittedAltitude = 0;
        for (int i = 0; i < 3; ++i) {
            variance += ((covariance_[3*i+1]*time + covariance_[3*i]*x[0]) + covariance_[3*i+2])*x[i];
            fittedAltitude += x[i]*coefficients_[i];
        }
        double gain[3];
        for (int i = 0; i < 3; ++i) {
            gain[i] = ((covariance_[i+3]*time + covariance_[i]*x[0]) + covariance_[i+6])
                / (variance + forgetting);
            coefficients_[i] += gain[i]*(altitude-fittedAltitude);
        }
        for (int column = 0; column < 3; ++column) {
            const double* p = covariance_ + 3*column;
            double updated[3];
            for (int row = 0; row < 3; ++row) {
                updated[row] = (p[row] - ((p[1]*(gain[row]*time) + p[0]*(gain[row]*x[0]))
                    + p[2]*gain[row])) / forgetting;
            }
            for (int row = 0; row < 3; ++row) covariance_[3*column+row] = updated[row];
        }
    }

    float altitude() const {
        const double* c = coefficients_;
        if (c[0] < -1e-6) {
            const double prediction = c[2] - c[1]*c[1]/(4*c[0]);
            if (std::isfinite(prediction)) return static_cast<float>(prediction);
        }
        return std::numeric_limits<float>::quiet_NaN();
    }

private:
    double coefficients_[3] = {};
    double covariance_[9] = {1000,0,0, 0,1000,0, 0,0,1000};
};

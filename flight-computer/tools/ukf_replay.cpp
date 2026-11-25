#include "../src/shared/vertical_observation.h"

#include <charconv>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace {
constexpr char kInputHeader[] = "flight_id,time_us,accel_z_g,baro_alt";
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

bool parseUnsigned(const char* field, uint32_t& value) {
    const char* end = field + std::strlen(field);
    const auto parsed = std::from_chars(field, end, value);
    return parsed.ec == std::errc{} && parsed.ptr == end;
}

bool parseObservation(const char* field, float& value) {
    if (!*field) {
        value = kNaN;
        return true;
    }
    char* end;
    value = std::strtof(field, &end);
    return end != field && !*end && std::isfinite(value);
}

bool readLine(std::string& line) {
    if (!std::getline(std::cin, line)) return false;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    return true;
}

bool parseRow(std::string& row, uint32_t& id, uint32_t& time, float& accel, float& altitude) {
    char* fields[4] = {row.data()};
    for (int i = 0; i < 3; ++i) {
        char* comma = std::strchr(fields[i], ',');
        if (!comma) return false;
        *comma = '\0';
        fields[i + 1] = comma + 1;
    }
    return parseUnsigned(fields[0], id) && parseUnsigned(fields[1], time) &&
        parseObservation(fields[2], accel) && parseObservation(fields[3], altitude) &&
        (std::isfinite(accel) || std::isfinite(altitude));
}

void emitField(float value) {
    // An empty CSV cell denotes an unavailable value on every host CRT.
    if (std::isfinite(value)) std::cout << value;
    std::cout << ',';
}

struct Flight {
    explicit Flight(uint32_t flightId) : id(flightId) {}

    void emit(uint32_t time, float accel, float altitude) {
        lastTimeUs = time;
        const uint32_t nowMs = time / 1000;
        if (std::isfinite(accel)) estimator.observeImu(nowMs, accel);
        vertical::BaroTiming timing;
        if (std::isfinite(altitude) && history.accept(time, altitude, timing)) {
            baroAlt = altitude;
            estimator.observeBaro(timing, nowMs, altitude);
            kfAlt = estimator.altitude();
        }
        const float kfVz = estimator.velocity(nowMs);
        // Estimate/input validity, not the deployment cadence or arming verdict.
        const bool valid = std::isfinite(kfAlt) && std::isfinite(kfVz);
        std::cout << id << ',' << time << ',';
        emitField(baroAlt);
        emitField(kfAlt);
        emitField(kfVz);
        std::cout << (valid ? 1 : 0) << '\n';
    }

    uint32_t id;
    uint32_t lastTimeUs = 0;
    float baroAlt = kNaN;
    float kfAlt = kNaN;
    vertical::BaroHistory history;
    vertical::StateEstimator estimator;
};
}  // namespace

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::string line;
    if (!readLine(line) || line != kInputHeader) {
        std::cerr << "Expected header: " << kInputHeader << '\n';
        return EXIT_FAILURE;
    }

    std::cout << "flight_id,time_us,baro_alt,kf_alt,kf_vz,valid\n"
              << std::setprecision(std::numeric_limits<float>::max_digits10);
    std::optional<Flight> flight;
    size_t lineNumber = 1;
    while (readLine(line)) {
        ++lineNumber;
        uint32_t id, time;
        float accel, altitude;
        if (!parseRow(line, id, time, accel, altitude)) {
            std::cerr << "Invalid sensor row at line " << lineNumber << '\n';
            return EXIT_FAILURE;
        }
        if (flight && (id < flight->id || (id == flight->id && time <= flight->lastTimeUs))) {
            std::cerr << "Out-of-order flight or timestamp at line " << lineNumber << '\n';
            return EXIT_FAILURE;
        }
        if (!flight || flight->id != id) flight.emplace(id);
        flight->emit(time, accel, altitude);
    }
    return std::cin.eof() && std::cout.flush() ? EXIT_SUCCESS : EXIT_FAILURE;
}

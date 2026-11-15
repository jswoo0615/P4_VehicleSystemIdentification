
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace p4::io::detail {

// ============================================================
// 1. Trim
// ============================================================

inline std::string trim(const std::string& text) {
    const auto first =
        text.find_first_not_of(" \t\r\n");

    if (first == std::string::npos) {
        return "";
    }

    const auto last =
        text.find_last_not_of(" \t\r\n");

    return text.substr(first, last - first + 1);
}

// ============================================================
// 2. Split
// ============================================================

inline std::vector<std::string> split(
    const std::string& line
) {
    std::vector<std::string> tokens;
    std::istringstream stream(line);

    std::string token;

    while (stream >> token) {
        tokens.push_back(token);
    }

    return tokens;
}

// ============================================================
// 3. Parse Number
// ============================================================

inline double parseNumber(std::string value) {
    value = trim(value);

    if (value.empty()) {
        throw std::invalid_argument(
            "Empty VBO numeric value"
        );
    }

    std::replace(
        value.begin(),
        value.end(),
        ',', '.'
    );

    std::istringstream stream(value);
    stream.imbue(std::locale::classic());

    double result = 0.0;

    if (!(stream >> result)) {
        throw std::invalid_argument(
            "Invalid VBO number: " + value
        );
    }

    stream >> std::ws;

    if (stream.peek() !=
        std::char_traits<char>::eof()) {
        throw std::invalid_argument(
            "Trailing characters in VBO number: "
            + value
        );
    }

    if (!std::isfinite(result)) {
        throw std::invalid_argument(
            "Non-finite VBO number: " + value
        );
    }

    return result;
}

// ============================================================
// 4. Parse HHMMSS
// ============================================================

inline double parseTime(
    const std::string& value
) {
    const double raw = parseNumber(value);

    if (raw < 0.0 || raw >= 240000.0) {
        throw std::invalid_argument(
            "VBO timestamp out of range: " + value
        );
    }

    const int hours =
        static_cast<int>(raw / 10000.0);

    const int minutes =
        static_cast<int>(raw / 100.0) % 100;

    const double seconds =
        raw
        - static_cast<double>(hours) * 10000.0
        - static_cast<double>(minutes) * 100.0;

    if (hours >= 24 ||
        minutes >= 60 ||
        seconds < 0.0 ||
        seconds >= 60.0) {
        throw std::invalid_argument(
            "Invalid HHMMSS timestamp: " + value
        );
    }

    return hours * 3600.0
         + minutes * 60.0
         + seconds;
}

// ============================================================
// 5. Unique Column Names
// ============================================================

inline std::vector<std::string> makeUniqueColumns(
    const std::vector<std::string>& original
) {
    std::vector<std::string> unique;
    unique.reserve(original.size());

    std::unordered_set<std::string> used;

    const std::unordered_set<std::string> reserved(
        original.begin(),
        original.end()
    );

    for (const auto& name : original) {
        if (name.empty()) {
            throw std::invalid_argument(
                "Empty VBO column name"
            );
        }

        if (used.insert(name).second) {
            unique.push_back(name);
            continue;
        }

        std::size_t suffix = 2;
        std::string candidate;

        do {
            candidate =
                name + "__" + std::to_string(suffix++);

        } while (
            used.count(candidate) != 0 ||
            reserved.count(candidate) != 0
        );

        used.insert(candidate);
        unique.push_back(candidate);
    }

    return unique;
}

// ============================================================
// 6. Default VBO Columns (35 channels)
// ============================================================

inline std::vector<std::string> defaultVBOColumns() {
    return {
        "sats",
        "time",
        "time",
        "lat",
        "long",
        "velocity",
        "heading",
        "height",
        "vert-vel",
        "Longacc",
        "Latacc",
        "momento",
        "steer_ang2",
        "steer_vel",
        "Glonass_Sats",
        "Velocity_Quality",
        "event-1",
        "True_Head",
        "Slip_Angle",
        "Pitch_Ang.",
        "Lat._Vel.",
        "Yaw_Rate",
        "Roll_Angle",
        "Lng._Vel.",
        "Slip_COG",
        "pedalFreno",
        "suspRL",
        "suspRR",
        "YawRate",
        "X_Accel",
        "Y_Accel",
        "Temp",
        "PitchRate",
        "RollRate",
        "Z_Accel"
    };
}

} // namespace p4::io::detail

#include "p4/io/IONIQ5LoadSamples.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace p4::io {
namespace {
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}
}

std::vector<LoadPoint> readGuardedLoadSamples(const std::string& path,
                                               SteeringRateMode mode) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open " + path);
    std::string line;
    if (!std::getline(in, line)) throw std::runtime_error("Empty CSV: " + path);
    const auto header = split(line);
    std::unordered_map<std::string, std::size_t> col;
    for (std::size_t i = 0; i < header.size(); ++i)
        if (!col.emplace(header[i], i).second)
            throw std::runtime_error("Duplicate CSV column: " + header[i]);
    for (const char* name : {"time_s", "delta_stm_rad", "F_trl_N", "F_trr_N"})
        if (!col.count(name)) throw std::runtime_error(std::string("Missing column ") + name + " in " + path);
    std::vector<LoadPoint> changes;
    double last_t = -1, last_delta = 0;
    bool first = true;
    while (std::getline(in, line)) {
        const auto fields = split(line);
        if (fields.size() != header.size()) throw std::runtime_error("CSV width mismatch: " + path);
        auto get = [&](const char* name) {
            const auto& field = fields.at(col.at(name));
            std::size_t used = 0;
            const double value = std::stod(field, &used);
            if (used != field.size() || !std::isfinite(value))
                throw std::runtime_error("Invalid CSV number: " + path);
            return value;
        };
        LoadPoint p{get("time_s"), get("delta_stm_rad"), get("F_trl_N"), get("F_trr_N")};
        if (p.t <= last_t) throw std::runtime_error("Unordered time: " + path);
        last_t = p.t;
        if (first || p.delta != last_delta) changes.push_back(p);
        first = false;
        last_delta = p.delta;
    }
    std::vector<LoadPoint> samples;
    for (std::size_t i = 1; i + 1 < changes.size(); ++i) {
        const double before = changes[i].t - changes[i-1].t;
        const double after = changes[i+1].t - changes[i].t;
        if (before < .01 || after < .01 || before > .20 || after > .20) continue;
        auto p = changes[i];
        p.rate = mode == SteeringRateMode::PreviousUpdate
            ? (changes[i].delta - changes[i-1].delta) / before
            : (changes[i+1].delta - changes[i-1].delta) / (before + after);
        if (std::isfinite(p.rate) && std::abs(p.rate) <= 5.0) samples.push_back(p);
    }
    return samples;
}

} // namespace p4::io

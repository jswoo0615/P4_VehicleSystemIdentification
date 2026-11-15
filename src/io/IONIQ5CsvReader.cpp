#include "p4/io/IONIQ5CsvReader.hpp"

#include <cmath>
#include <fstream>
#include <limits>
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
    if (!line.empty() && line.back() == ',') fields.emplace_back();
    return fields;
}

double parse(const std::string& field, std::size_t line) {
    std::size_t used = 0;
    double value;
    try { value = std::stod(field, &used); }
    catch (const std::exception&) {
        throw std::runtime_error("Invalid CSV number at line " + std::to_string(line));
    }
    if (used != field.size() || !std::isfinite(value))
        throw std::runtime_error("Invalid CSV number at line " + std::to_string(line));
    return value;
}
} // namespace

ModelACsvSamples IONIQ5CsvReader::readModelA(
    const std::string& path, double start_s, double end_s) const {
    if (!std::isfinite(start_s) || !std::isfinite(end_s) ||
        start_s < 0 || end_s <= start_s)
        throw std::invalid_argument("Invalid time interval");
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open CSV: " + path);
    std::string line;
    while (std::getline(input, line) && (line.empty() || line.front() == '#')) {}
    if (line.empty() || line.front() == '#') throw std::runtime_error("Empty CSV: " + path);
    const auto headers = split(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < headers.size(); ++i)
        if (!columns.emplace(headers[i], i).second)
            throw std::runtime_error("Duplicate CSV column: " + headers[i]);
    const bool golf = map_ == CsvChannelMap::StanfordGolf;
    const char* time_col = golf ? "t_s" : "time_s";
    const char* steer_col = golf ? "roadWheelAngle_rad" : "delta_stm_rad";
    const char* yaw_col = golf ? "yawRate_radps" : "w_z_cor_radps";
    const char* speed_col = golf ? "Vx_mps" : "v_x_cor_mps";
    const char* ay_col = golf ? "ayCG_mps2" : "a_y_ra_mps2";
    const char* vy_col = golf ? "Vy_mps" : "v_y_cor_mps";
    for (const auto* name : {time_col, steer_col, yaw_col, speed_col})
        if (!columns.count(name)) throw std::runtime_error(std::string("Missing Model A column: ") + name);

    ModelACsvSamples out;
    double previous = -std::numeric_limits<double>::infinity();
    std::size_t line_number = 1;
    while (std::getline(input, line)) {
        ++line_number;
        const auto fields = split(line);
        if (fields.size() != headers.size())
            throw std::runtime_error("CSV column count mismatch at line " + std::to_string(line_number));
        const double time = parse(fields[columns.at(time_col)], line_number);
        if (time <= previous)
            throw std::runtime_error("Non-increasing time at line " + std::to_string(line_number));
        previous = time;
        if (time < start_s || time > end_s) continue;
        out.time_s.push_back(time);
        out.steering_rad.push_back(parse(fields[columns.at(steer_col)], line_number));
        out.yaw_rad_s.push_back(parse(fields[columns.at(yaw_col)], line_number));
        out.speed_m_s.push_back(parse(fields[columns.at(speed_col)], line_number));
        out.lateral_accel_m_s2.push_back(columns.count(ay_col)
            ? parse(fields[columns.at(ay_col)], line_number)
            : std::numeric_limits<double>::quiet_NaN());
        out.lateral_velocity_m_s.push_back(columns.count(vy_col)
            ? parse(fields[columns.at(vy_col)], line_number)
            : std::numeric_limits<double>::quiet_NaN());
        out.bank_rad.push_back(columns.count("bank_rad")
            ? parse(fields[columns.at("bank_rad")], line_number) : 0.0);
    }
    if (out.time_s.size() < 30)
        throw std::runtime_error("Fewer than 30 samples in Model A interval");
    return out;
}

std::vector<ModelACsvSamples> IONIQ5CsvReader::readModelASegments(
    const std::string& path, const ModelACsvOptions& options,
    double start_s, double end_s) const {
    if (!(options.sample_period_s > 0) || !(options.maximum_gap_s > options.sample_period_s) ||
        !(options.minimum_segment_s > 0) || !(options.glitch_guard_s >= 0) ||
        !(options.yaw_jump_rad_s > 0))
        throw std::invalid_argument("Invalid Model A sampling options");
    const auto raw = readModelA(path, start_s, end_s);
    std::vector<double> glitches;
    for (std::size_t i = 1; i < raw.time_s.size(); ++i) {
        if (raw.time_s[i] - raw.time_s[i-1] <= 0.03 &&
            std::abs(raw.yaw_rad_s[i] - raw.yaw_rad_s[i-1]) > options.yaw_jump_rad_s)
            glitches.push_back(raw.time_s[i]);
    }
    std::vector<ModelACsvSamples> segments;
    ModelACsvSamples current;
    double next_sample = raw.time_s.front();
    std::size_t glitch_index = 0;
    auto flush = [&]() {
        if (current.time_s.size() >= 30 &&
            current.time_s.back() - current.time_s.front() >= options.minimum_segment_s)
            segments.push_back(std::move(current));
        current = {};
    };
    for (std::size_t i = 0; i < raw.time_s.size(); ++i) {
        const double t = raw.time_s[i];
        while (glitch_index < glitches.size() &&
               glitches[glitch_index] < t - options.glitch_guard_s - 1e-9) ++glitch_index;
        const bool glitch = glitch_index < glitches.size() &&
            std::abs(t - glitches[glitch_index]) <= options.glitch_guard_s + 1e-9;
        if (raw.speed_m_s[i] < options.minimum_speed_m_s || glitch) {
            flush();
            next_sample = t + options.sample_period_s;
            continue;
        }
        // Stanford timestamps jitter around 5 ms; retain the IONIQ sampling rule.
        if (t + (map_ == CsvChannelMap::StanfordGolf ? 0.1 * options.sample_period_s : 1e-9)
                < next_sample) continue;
        if (!current.time_s.empty() && t - current.time_s.back() > options.maximum_gap_s)
            flush();
        current.time_s.push_back(t);
        current.steering_rad.push_back(raw.steering_rad[i]);
        current.yaw_rad_s.push_back(raw.yaw_rad_s[i]);
        current.speed_m_s.push_back(raw.speed_m_s[i]);
        current.lateral_accel_m_s2.push_back(raw.lateral_accel_m_s2[i]);
        current.lateral_velocity_m_s.push_back(raw.lateral_velocity_m_s[i]);
        current.bank_rad.push_back(raw.bank_rad[i]);
        if (map_ == CsvChannelMap::StanfordGolf) {
            next_sample += options.sample_period_s;
            if (next_sample < t + 0.5 * options.sample_period_s)
                next_sample = t + options.sample_period_s;
        } else {
            next_sample = t + options.sample_period_s;
        }
    }
    flush();
    return segments;
}
} // namespace p4::io

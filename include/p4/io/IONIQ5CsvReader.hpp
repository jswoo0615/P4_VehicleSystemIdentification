#ifndef P4_IO_IONIQ5_CSV_READER_HPP_
#define P4_IO_IONIQ5_CSV_READER_HPP_

#include <string>
#include <vector>

namespace p4::io {

enum class CsvChannelMap { IONIQ5, StanfordGolf };

struct ModelACsvSamples {
    std::vector<double> time_s;
    std::vector<double> steering_rad;
    std::vector<double> yaw_rad_s;
    std::vector<double> speed_m_s;
    std::vector<double> lateral_accel_m_s2;
    std::vector<double> lateral_velocity_m_s;
    std::vector<double> bank_rad;
};

struct ModelACsvOptions {
    double minimum_speed_m_s = 2.0;
    double sample_period_s = 0.02;
    double maximum_gap_s = 0.08;
    double yaw_jump_rad_s = 0.25;
    double glitch_guard_s = 0.10;
    double minimum_segment_s = 2.0;
};

class IONIQ5CsvReader {
public:
    explicit IONIQ5CsvReader(CsvChannelMap map = CsvChannelMap::IONIQ5) : map_(map) {}
    ModelACsvSamples readModelA(const std::string& path,
                               double start_s, double end_s) const;
    std::vector<ModelACsvSamples> readModelASegments(
        const std::string& path, const ModelACsvOptions& options = {},
        double start_s = 0, double end_s = 1e9) const;
private:
    CsvChannelMap map_;
};

} // namespace p4::io
#endif

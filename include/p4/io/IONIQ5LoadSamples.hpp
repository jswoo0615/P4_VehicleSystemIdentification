#ifndef P4_IO_IONIQ5_LOAD_SAMPLES_HPP_
#define P4_IO_IONIQ5_LOAD_SAMPLES_HPP_

#include <string>
#include <vector>

namespace p4::io {

struct LoadPoint {
    double t;
    double delta;
    double left;
    double right;
    double rate = 0;
};

enum class SteeringRateMode { Centered, PreviousUpdate };

// Select actual steering-value updates. Centered rate uses the next update
// and is for offline analysis; PreviousUpdate uses only past angle values.
std::vector<LoadPoint> readGuardedLoadSamples(
    const std::string& path,
    SteeringRateMode mode = SteeringRateMode::Centered);

} // namespace p4::io

#endif

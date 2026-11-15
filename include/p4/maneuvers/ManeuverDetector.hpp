
#ifndef MANEUVER_DETECTOR_HPP_
#define MANEUVER_DETECTOR_HPP_

#include "p4/io/VBOData.hpp"
#include "p4/maneuvers/Maneuver.hpp"

#include <string>
#include <vector>

namespace p4::maneuvers {

struct ManeuverDetectorConfig {
    double min_steering_change_deg = 3.0;
    double min_onset_change_deg = 0.2;
    double max_rise_time = 0.5;
    double min_hold_time = 0.3;
    double hold_tolerance_deg = 0.5;
    double max_sample_gap = 0.1;
};

class ManeuverDetector {
public:
    explicit ManeuverDetector(
        ManeuverDetectorConfig config = {}
    );

    std::vector<Maneuver> detectStepSteer(
        const p4::io::VBOData& data,
        const std::string& steering_column =
            "steer_ang2"
    ) const;

private:
    ManeuverDetectorConfig config_;
};

} // namespace p4::maneuvers
#endif // MANEUVER_DETECTOR_HPP_
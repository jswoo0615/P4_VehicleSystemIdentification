#ifndef MAJOR_EXCURSION_DETECTOR_HPP_
#define MAJOR_EXCURSION_DETECTOR_HPP_

#include "p4/io/VBOData.hpp"
#include "p4/maneuvers/ManeuverSegmenter.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace p4::maneuvers {

enum class ExcursionType {
    Entry,
    Recovery,
    Reversal,
    HoldCandidate,
    Neutral,
    Transition
};

struct MajorExcursion {
    std::size_t event_id = 0;
    std::size_t excursion_id = 0;

    // Inclusive indices in original VBOData
    std::size_t start_index = 0;
    std::size_t end_index = 0;

    double start_s = 0.0;
    double end_s = 0.0;

    double steer_start_deg = 0.0;
    double steer_end_deg = 0.0;
    double steering_delta_deg = 0.0;

    ExcursionType type = ExcursionType::Transition;
    bool verified = false;
};

struct MajorExcursionConfig {
    double smooth_seconds = 0.50;
    double min_excursion_deg = 4.0;
    double min_peak_gap_s = 0.50;
    double active_steer_deg = 3.0;
};

std::string toString(ExcursionType type);

class MajorExcursionDetector {
public:
    explicit MajorExcursionDetector(
        MajorExcursionConfig config = {}
    );

    std::vector<MajorExcursion> analyze(
        const p4::io::VBOData& data,
        const std::vector<ManeuverEvent>& events
    ) const;

private:
    MajorExcursionConfig config_;
};

} // namespace p4::maneuvers

#endif // MAJOR_EXCURSION_DETECTOR_HPP_
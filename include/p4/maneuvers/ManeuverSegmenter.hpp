#ifndef MANEUVER_SEGMENTER_HPP_
#define MANEUVER_SEGMENTER_HPP_

#include "p4/io/VBOData.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace p4::maneuvers {

enum class EventCandidate {
    Circle,
    JTurn,
    AlternatingSteering,
    SteeringReversal,
    SteeringUnknown,
    SustainedTurnUnknown,
    ShortTurnUnknown
};

struct ManeuverEvent {
    std::size_t event_id = 0;

    // [start_index, end_index)
    std::size_t start_index = 0;
    std::size_t end_index = 0;

    // Relative to beginning of VBO recording
    double start_s = 0.0;
    double end_s = 0.0;

    double duration_s = 0.0;

    double mean_speed_kmh = 0.0;
    double max_abs_steer_deg = 0.0;
    double max_abs_yaw_deg_s = 0.0;

    EventCandidate candidate =
        EventCandidate::SteeringUnknown;

    // Legacy classifications are provisional.
    bool verified = false;
};

struct ManeuverSegmenterConfig {
    double smooth_seconds = 0.15;

    double steer_on_deg = 3.0;
    double yaw_on_deg_s = 3.0;

    double padding_seconds = 0.35;
    double merge_gap_seconds = 1.0;
    double min_event_seconds = 0.40;

    double circle_steer_deg = 10.0;
    double circle_yaw_deg_s = 8.0;
    double circle_min_fraction = 0.70;
    double circle_min_duration = 8.0;
    double circle_direction_fraction = 0.95;

    double jturn_steer_deg = 20.0;
    double sustained_turn_seconds = 3.0;
};

std::string toString(EventCandidate candidate);

class ManeuverSegmenter {
public:
    explicit ManeuverSegmenter(
        ManeuverSegmenterConfig config = {}
    );

    std::vector<ManeuverEvent> analyze(
        const p4::io::VBOData& data
    ) const;

private:
    ManeuverSegmenterConfig config_;
};

} // namespace p4::maneuvers
#endif // MANEUVER_SEGMENTER_HPP_
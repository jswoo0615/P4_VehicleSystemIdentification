#ifndef MANEUVER_PHASE_ANALYZER_HPP_
#define MANEUVER_PHASE_ANALYZER_HPP_

#include "p4/io/VBOData.hpp"
#include "p4/maneuvers/ManeuverSegmenter.hpp"
#include "p4/maneuvers/MajorExcursionDetector.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace p4::maneuvers {

enum class PhaseType {
    Entry,
    Hold,
    Recovery,
    Reversal,
    Neutral,
    Transition,
    SustainedTurn,
    InitialBoundary
};

struct ManeuverPhase {
    std::size_t event_id = 0;
    std::size_t phase_id = 0;

    // Major excursions contributing to this phase.
    std::vector<std::size_t> excursion_ids;

    double start_s = 0.0;
    double end_s = 0.0;
    double duration_s = 0.0;

    double steer_start_deg = 0.0;
    double steer_end_deg = 0.0;
    double speed_mean_kmh = 0.0;
    double yaw_mean_deg_s = 0.0;

    PhaseType type = PhaseType::Transition;
    bool verified = false;
};

struct ManeuverPhaseConfig {
    double smooth_seconds = 0.50;
    double active_steer_deg = 3.0;
    double hold_rate_deg_s = 2.0;
    double hold_peak_tolerance_deg = 1.5;
    double min_hold_s = 0.25;
};

std::string toString(PhaseType type);

class ManeuverPhaseAnalyzer {
public:
    explicit ManeuverPhaseAnalyzer(
        ManeuverPhaseConfig config = {}
    );

    std::vector<ManeuverPhase> analyze(
        const p4::io::VBOData& data,
        const std::vector<ManeuverEvent>& events,
        const std::vector<MajorExcursion>& excursions
    ) const;

private:
    ManeuverPhaseConfig config_;
};

} // namespace p4::maneuvers
#endif // MANEUVER_PHASE_ANALYZER_HPP_
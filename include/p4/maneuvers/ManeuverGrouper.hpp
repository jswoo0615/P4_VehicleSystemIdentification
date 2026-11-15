#ifndef MANEUVER_GROUPER_HPP_
#define MANEUVER_GROUPER_HPP_

#include "p4/maneuvers/ManeuverSegmenter.hpp"

#include <cstddef>
#include <vector>

namespace p4::maneuvers {

struct ManeuverGroup {
    std::size_t group_id = 0;

    double start_s = 0.0;
    double end_s = 0.0;
    double duration_s = 0.0;

    std::vector<ManeuverEvent> events;
};

struct ManeuverGrouperConfig {
    double max_event_gap_s = 4.0;
};

class ManeuverGrouper {
public:
    explicit ManeuverGrouper(
        ManeuverGrouperConfig config = {}
    );

    std::vector<ManeuverGroup> group(
        const std::vector<ManeuverEvent>& events
    ) const;

private:
    ManeuverGrouperConfig config_;
};

} // namespace p4::maneuvers
#endif // MANEUVER_GROUPER_HPP_
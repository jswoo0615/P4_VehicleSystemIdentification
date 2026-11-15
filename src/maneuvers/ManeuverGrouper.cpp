
#include "p4/maneuvers/ManeuverGrouper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace p4::maneuvers {

namespace {

bool isCircle(const ManeuverEvent& event) {
    return event.candidate == EventCandidate::Circle;
}

ManeuverGroup makeGroup(
    std::size_t id,
    const ManeuverEvent& event
) {
    ManeuverGroup result;

    result.group_id = id;
    result.start_s = event.start_s;
    result.end_s = event.end_s;
    result.duration_s = event.end_s - event.start_s;
    result.events.push_back(event);

    return result;
}

} // anonymous namespace

ManeuverGrouper::ManeuverGrouper(
    ManeuverGrouperConfig config
) : config_(config) {
    if (!std::isfinite(config_.max_event_gap_s) ||
        config_.max_event_gap_s < 0.0) {
        throw std::invalid_argument(
            "Invalid max_event_gap_s"
        );
    }
}

std::vector<ManeuverGroup> ManeuverGrouper::group(
    const std::vector<ManeuverEvent>& events
) const {
    if (events.empty()) {
        return {};
    }

    // Sort without modifying the original input.
    std::vector<ManeuverEvent> sorted = events;

    std::stable_sort(
        sorted.begin(),
        sorted.end(),
        [](const ManeuverEvent& a,
           const ManeuverEvent& b) {
            return a.start_s < b.start_s;
        }
    );

    std::vector<ManeuverGroup> groups;

    for (const auto& event : sorted) {
        if (!std::isfinite(event.start_s) ||
            !std::isfinite(event.end_s) ||
            event.end_s < event.start_s) {
            throw std::invalid_argument(
                "Invalid event time interval"
            );
        }

        if (groups.empty()) {
            groups.push_back(makeGroup(1, event));
            continue;
        }

        auto& current = groups.back();

        const auto& previous = current.events.back();

        // Preserve Circle as a standalone group.
        const bool circleBoundary =
            isCircle(previous) || isCircle(event);

        const double gap =
            event.start_s - previous.end_s;

        if (!circleBoundary &&
            gap <= config_.max_event_gap_s) {

            current.events.push_back(event);
            current.end_s = event.end_s;
            current.duration_s =
                current.end_s - current.start_s;
        }
        else {
            groups.push_back(
                makeGroup(groups.size() + 1, event)
            );
        }
    }

    return groups;
}

} // namespace p4::maneuvers

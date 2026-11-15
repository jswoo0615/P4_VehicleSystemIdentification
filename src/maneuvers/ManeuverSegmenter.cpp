
#include "p4/maneuvers/ManeuverSegmenter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace p4::maneuvers {

namespace {

struct Signals {
    std::vector<double> t;
    std::vector<double> steer;
    std::vector<double> yaw;
    std::vector<double> speed;
};

struct Range {
    std::size_t begin;
    std::size_t end; // exclusive
};

Signals readSignals(const p4::io::VBOData& data) {
    for (const char* channel :
         {"steer_ang2", "YawRate", "velocity"}) {
        if (!data.hasColumn(channel)) {
            throw std::runtime_error(
                std::string("Missing channel: ") + channel
            );
        }
    }

    Signals s;

    const auto n = data.sampleCount();
    if (n < 3) {
        throw std::runtime_error(
            "Insufficient VBO samples"
        );
    }

    const double t0 = data.timeAt(0);

    s.t.reserve(n);
    s.steer.reserve(n);
    s.yaw.reserve(n);
    s.speed.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
        const double t = data.timeAt(i) - t0;
        const double steer = data.value(i, "steer_ang2");
        const double yaw = data.value(i, "YawRate");
        const double speed = data.value(i, "velocity");

        if (!std::isfinite(t) ||
            !std::isfinite(steer) ||
            !std::isfinite(yaw) ||
            !std::isfinite(speed)) {
            throw std::runtime_error(
                "Non-finite signal value"
            );
        }

        if (!s.t.empty() && t <= s.t.back()) {
            throw std::runtime_error(
                "Non-increasing timestamp"
            );
        }

        s.t.push_back(t);
        s.steer.push_back(steer);
        s.yaw.push_back(yaw);
        s.speed.push_back(speed);
    }

    return s;
}

double medianDt(const std::vector<double>& t) {
    std::vector<double> intervals;
    intervals.reserve(t.size() - 1);

    for (std::size_t i = 1; i < t.size(); ++i) {
        intervals.push_back(t[i] - t[i - 1]);
    }

    std::sort(intervals.begin(), intervals.end());

    const std::size_t n = intervals.size();

    if (n % 2 == 1) {
        return intervals[n / 2];
    }

    return 0.5 * (
        intervals[n / 2 - 1] + intervals[n / 2]
    );
}

std::vector<double> smooth(
    const std::vector<double>& values,
    const std::vector<double>& time,
    double window_seconds
) {
    const double dt = medianDt(time);

    int window = std::max(
        1, static_cast<int>(
            std::nearbyint(window_seconds / dt)
        )
    );

    if (window % 2 == 0) {
        ++window;
    }

    const int pad = window / 2;
    const int n = static_cast<int>(values.size());

    std::vector<double> output(values.size());

    for (int i = 0; i < n; ++i) {
        double sum = 0.0;

        for (int j = -pad; j <= pad; ++j) {
            const int k = std::clamp(
                i + j, 0, n - 1
            );
            sum += values[static_cast<std::size_t>(k)];
        }

        output[static_cast<std::size_t>(i)] =
            sum / static_cast<double>(window);
    }

    return output;
}

double medianOf(
    std::vector<double> values
) {
    std::sort(values.begin(), values.end());

    const auto n = values.size();

    if (n % 2 == 1) {
        return values[n / 2];
    }

    return 0.5 * (
        values[n / 2 - 1] + values[n / 2]
    );
}

bool isCircleCandidate(
    const Signals& s,
    const std::vector<double>& steer,
    const std::vector<double>& yaw,
    const ManeuverSegmenterConfig& c
) {
    const double duration = s.t.back() - s.t.front();

    if (duration < c.circle_min_duration) {
        return false;
    }

    std::vector<double> activeSteer;
    std::vector<double> activeYaw;

    for (std::size_t i = 0; i < s.t.size(); ++i) {
        if (std::abs(steer[i]) >= c.circle_steer_deg &&
            std::abs(yaw[i]) >= c.circle_yaw_deg_s) {
            activeSteer.push_back(steer[i]);
            activeYaw.push_back(yaw[i]);
        }
    }

    if (activeSteer.empty()) {
        return false;
    }

    const double activeFraction =
        static_cast<double>(activeSteer.size()) /
        static_cast<double>(s.t.size());

    const auto sign = [](double x) {
        return (x > 0.0) - (x < 0.0);
    };

    const int steerSign = sign(medianOf(activeSteer));
    const int yawSign = sign(medianOf(activeYaw));

    if (steerSign == 0 || yawSign == 0) {
        return false;
    }

    std::size_t matching = 0;

    for (std::size_t i = 0; i < activeSteer.size(); ++i) {
        if (sign(activeSteer[i]) == steerSign &&
            sign(activeYaw[i]) == yawSign) {
            ++matching;
        }
    }

    const double directionalFraction =
        static_cast<double>(matching) /
        static_cast<double>(activeSteer.size());

    return activeFraction >= c.circle_min_fraction &&
           directionalFraction >=
               c.circle_direction_fraction;
}

std::vector<Range> detectRanges(
    const Signals& s,
    const std::vector<double>& steer,
    const std::vector<double>& yaw,
    const ManeuverSegmenterConfig& c
) {
    const auto n = s.t.size();

    std::vector<std::uint8_t> expanded(n, 0);

    // Expand every active sample by +/- padding_seconds.
    for (std::size_t i = 0; i < n; ++i) {
        const bool active =
            std::abs(steer[i]) >= c.steer_on_deg ||
            std::abs(yaw[i]) >= c.yaw_on_deg_s;

        if (!active) {
            continue;
        }

        const auto left = std::lower_bound(
            s.t.begin(), s.t.end(),
            s.t[i] - c.padding_seconds
        );

        const auto right = std::upper_bound(
            s.t.begin(), s.t.end(),
            s.t[i] + c.padding_seconds
        );

        const auto lo = static_cast<std::size_t>(
            left - s.t.begin()
        );

        const auto hi = static_cast<std::size_t>(
            right - s.t.begin()
        );

        std::fill(
            expanded.begin() + lo,
            expanded.begin() + hi,
            static_cast<std::uint8_t>(1)
        );
    }

    std::vector<Range> groups;

    // Find contiguous active ranges.
    std::size_t i = 0;

    while (i < n) {
        if (!expanded[i]) {
            ++i;
            continue;
        }

        const std::size_t begin = i;

        while (i < n && expanded[i]) {
            ++i;
        }

        groups.push_back({begin, i});
    }

    if (groups.empty()) {
        return {};
    }

    // Merge nearby ranges.
    std::vector<Range> merged;
    merged.push_back(groups.front());

    for (std::size_t j = 1; j < groups.size(); ++j) {
        auto& previous = merged.back();
        const auto& current = groups[j];

        const double gap =
            s.t[current.begin] -
            s.t[previous.end - 1];

        if (gap <= c.merge_gap_seconds) {
            previous.end = current.end;
        }
        else {
            merged.push_back(current);
        }
    }

    // Reject short ranges.
    std::vector<Range> result;

    for (const auto& range : merged) {
        const double duration =
            s.t[range.end - 1] -
            s.t[range.begin];

        if (duration >= c.min_event_seconds) {
            result.push_back(range);
        }
    }

    return result;
}

EventCandidate classifyEvent(
    const Signals& s,
    const std::vector<double>& steer,
    const std::vector<double>& yaw,
    Range range,
    const ManeuverSegmenterConfig& c
) {
    double maxSteer = 0.0;
    double maxYaw = 0.0;

    int previousSign = 0;
    int signChanges = 0;

    for (std::size_t i = range.begin;
         i < range.end; ++i) {

        maxSteer = std::max(
            maxSteer, std::abs(steer[i])
        );

        maxYaw = std::max(
            maxYaw, std::abs(yaw[i])
        );

        if (std::abs(steer[i]) < c.steer_on_deg) {
            continue;
        }

        const int currentSign =
            steer[i] > 0.0 ? 1 : -1;

        if (previousSign != 0 &&
            currentSign != previousSign) {
            ++signChanges;
        }

        previousSign = currentSign;
    }

    if (maxSteer >= c.jturn_steer_deg &&
        signChanges == 0) {
        return EventCandidate::JTurn;
    }

    if (signChanges >= 2) {
        return EventCandidate::AlternatingSteering;
    }

    if (signChanges == 1) {
        return EventCandidate::SteeringReversal;
    }

    if (maxYaw < c.yaw_on_deg_s) {
        return EventCandidate::SteeringUnknown;
    }

    const double duration =
        s.t[range.end - 1] -
        s.t[range.begin];

    if (duration >= c.sustained_turn_seconds) {
        return EventCandidate::SustainedTurnUnknown;
    }

    return EventCandidate::ShortTurnUnknown;
}

ManeuverEvent makeEvent(
    const Signals& s,
    const std::vector<double>& steer,
    const std::vector<double>& yaw,
    Range range,
    std::size_t id,
    EventCandidate candidate
) {
    ManeuverEvent event;

    event.event_id = id;
    event.start_index = range.begin;
    event.end_index = range.end;

    event.start_s = s.t[range.begin];
    event.end_s = s.t[range.end - 1];

    event.duration_s =
        event.end_s - event.start_s;

    double speedSum = 0.0;

    for (std::size_t i = range.begin;
         i < range.end; ++i) {

        speedSum += s.speed[i];

        event.max_abs_steer_deg = std::max(
            event.max_abs_steer_deg,
            std::abs(steer[i])
        );

        event.max_abs_yaw_deg_s = std::max(
            event.max_abs_yaw_deg_s,
            std::abs(yaw[i])
        );
    }

    event.mean_speed_kmh =
        speedSum / static_cast<double>(
            range.end - range.begin
        );

    event.candidate = candidate;
    event.verified = false;

    return event;
}

} // anonymous namespace

std::string toString(EventCandidate candidate) {
    switch (candidate) {
        case EventCandidate::Circle:
            return "Circle candidate";

        case EventCandidate::JTurn:
            return "J-turn candidate";

        case EventCandidate::AlternatingSteering:
            return "Alternating steering / DLC candidate";

        case EventCandidate::SteeringReversal:
            return "Steering reversal / LC-DLC candidate";

        case EventCandidate::SteeringUnknown:
            return "Steering event / Unknown";

        case EventCandidate::SustainedTurnUnknown:
            return "Sustained turn / Unknown";

        case EventCandidate::ShortTurnUnknown:
            return "Short turn / Unknown";
    }

    return "Unknown";
}

ManeuverSegmenter::ManeuverSegmenter(
    ManeuverSegmenterConfig config
) : config_(config) {

    if (!(config_.smooth_seconds > 0.0) ||
        !(config_.steer_on_deg > 0.0) ||
        !(config_.yaw_on_deg_s > 0.0) ||
        !(config_.padding_seconds >= 0.0) ||
        !(config_.merge_gap_seconds >= 0.0) ||
        !(config_.min_event_seconds > 0.0) ||
        !(config_.circle_min_fraction > 0.0 &&
          config_.circle_min_fraction <= 1.0) ||
        !(config_.circle_direction_fraction > 0.0 &&
          config_.circle_direction_fraction <= 1.0) ||
        !(config_.circle_min_duration > 0.0) ||
        !(config_.jturn_steer_deg > 0.0)) {
        throw std::invalid_argument(
            "Invalid ManeuverSegmenter configuration"
        );
    }
}

std::vector<ManeuverEvent> ManeuverSegmenter::analyze(
    const p4::io::VBOData& data
) const {
    const Signals s = readSignals(data);

    const auto steer = smooth(
        s.steer, s.t, config_.smooth_seconds
    );

    const auto yaw = smooth(
        s.yaw, s.t, config_.smooth_seconds
    );

    // Circle is handled as one full-record event.
    if (isCircleCandidate(s, steer, yaw, config_)) {
        return {
            makeEvent(
                s, steer, yaw,
                {0, s.t.size()},
                1,
                EventCandidate::Circle
            )
        };
    }

    const auto ranges = detectRanges(
        s, steer, yaw, config_
    );

    std::vector<ManeuverEvent> events;
    events.reserve(ranges.size());

    for (const auto& range : ranges) {
        const auto candidate = classifyEvent(
            s, steer, yaw, range, config_
        );

        events.push_back(
            makeEvent(
                s, steer, yaw, range,
                events.size() + 1,
                candidate
            )
        );
    }

    return events;
}

} // namespace p4::maneuvers

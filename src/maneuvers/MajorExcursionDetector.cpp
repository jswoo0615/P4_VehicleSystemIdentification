
#include "p4/maneuvers/MajorExcursionDetector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace p4::maneuvers {

namespace {

double medianDt(const std::vector<double>& t) {
    std::vector<double> dt;
    dt.reserve(t.size() - 1);

    for (std::size_t i = 1; i < t.size(); ++i) {
        const double d = t[i] - t[i - 1];

        if (!std::isfinite(d) || d <= 0.0) {
            throw std::runtime_error(
                "Invalid sampling interval"
            );
        }

        dt.push_back(d);
    }

    std::sort(dt.begin(), dt.end());

    const auto n = dt.size();

    if (n % 2 == 1) {
        return dt[n / 2];
    }

    return 0.5 * (dt[n / 2 - 1] + dt[n / 2]);
}

std::vector<double> smoothSignal(
    const std::vector<double>& t,
    const std::vector<double>& x,
    double windowSeconds
) {
    const double dt = medianDt(t);

    int window = std::max(
        3,
        static_cast<int>(
            std::nearbyint(windowSeconds / dt)
        )
    );

    if (window % 2 == 0) {
        ++window;
    }

    const int pad = window / 2;
    const int n = static_cast<int>(x.size());

    std::vector<double> result(x.size());

    for (int i = 0; i < n; ++i) {
        double sum = 0.0;

        for (int j = -pad; j <= pad; ++j) {
            const int k = std::clamp(
                i + j, 0, n - 1
            );

            sum += x[static_cast<std::size_t>(k)];
        }

        result[static_cast<std::size_t>(i)] =
            sum / static_cast<double>(window);
    }

    return result;
}

// Return local indices of major steering extrema.
// First and last samples are always retained.
std::vector<std::size_t> turningPoints(
    const std::vector<double>& t,
    const std::vector<double>& y,
    const MajorExcursionConfig& c
) {
    const auto n = y.size();

    if (n < 3) {
        return {0, n - 1};
    }

    std::vector<std::size_t> anchors = {0};

    int direction = 0;

    std::size_t high = 0;
    std::size_t low = 0;

    for (std::size_t i = 1; i < n; ++i) {
        if (direction == 0) {
            if (y[i] > y[high]) {
                high = i;
            }

            if (y[i] < y[low]) {
                low = i;
            }

            if (y[i] - y[low] >= c.min_excursion_deg) {
                direction = 1;

                if (low > anchors.back()) {
                    anchors.push_back(low);
                }

                high = i;
            }
            else if (
                y[high] - y[i] >= c.min_excursion_deg
            ) {
                direction = -1;

                if (high > anchors.back()) {
                    anchors.push_back(high);
                }

                low = i;
            }
        }
        else if (direction == 1) {
            if (y[i] >= y[high]) {
                high = i;
            }
            else if (
                y[high] - y[i] >= c.min_excursion_deg
            ) {
                if (
                    t[high] - t[anchors.back()]
                    >= c.min_peak_gap_s
                ) {
                    anchors.push_back(high);
                }

                direction = -1;
                low = i;
            }
        }
        else {
            if (y[i] <= y[low]) {
                low = i;
            }
            else if (
                y[i] - y[low] >= c.min_excursion_deg
            ) {
                if (
                    t[low] - t[anchors.back()]
                    >= c.min_peak_gap_s
                ) {
                    anchors.push_back(low);
                }

                direction = 1;
                high = i;
            }
        }
    }

    // Keep the final candidate extremum.
    if (direction != 0) {
        const auto candidate =
            direction == 1 ? high : low;

        if (candidate > anchors.back()) {
            anchors.push_back(candidate);
        }
    }

    if (anchors.back() != n - 1) {
        anchors.push_back(n - 1);
    }

    std::sort(anchors.begin(), anchors.end());
    anchors.erase(
        std::unique(anchors.begin(), anchors.end()),
        anchors.end()
    );

    return anchors;
}

ExcursionType classifyInterval(
    const std::vector<double>& y,
    std::size_t a,
    std::size_t b,
    const MajorExcursionConfig& c
) {
    const double start = y[a];
    const double end = y[b];
    const double change = end - start;

    // Significant steering sign reversal.
    if (
        start * end < 0.0 &&
        std::min(std::abs(start), std::abs(end))
            >= c.active_steer_deg
    ) {
        return ExcursionType::Reversal;
    }

    if (std::abs(change) < c.min_excursion_deg) {
        if (
            std::max(std::abs(start), std::abs(end))
            >= c.active_steer_deg
        ) {
            return ExcursionType::HoldCandidate;
        }

        return ExcursionType::Neutral;
    }

    if (std::abs(end) > std::abs(start)) {
        return ExcursionType::Entry;
    }

    if (std::abs(end) < std::abs(start)) {
        return ExcursionType::Recovery;
    }

    return ExcursionType::Transition;
}

} // anonymous namespace

std::string toString(ExcursionType type) {
    switch (type) {
        case ExcursionType::Entry:
            return "ENTRY";

        case ExcursionType::Recovery:
            return "RECOVERY";

        case ExcursionType::Reversal:
            return "REVERSAL";

        case ExcursionType::HoldCandidate:
            return "HOLD_CANDIDATE";

        case ExcursionType::Neutral:
            return "NEUTRAL";

        case ExcursionType::Transition:
            return "TRANSITION";
    }

    return "UNKNOWN";
}

MajorExcursionDetector::MajorExcursionDetector(
    MajorExcursionConfig config
) : config_(config) {
    if (
        !std::isfinite(config_.smooth_seconds) ||
        config_.smooth_seconds <= 0.0 ||
        !std::isfinite(config_.min_excursion_deg) ||
        config_.min_excursion_deg <= 0.0 ||
        !std::isfinite(config_.min_peak_gap_s) ||
        config_.min_peak_gap_s < 0.0 ||
        !std::isfinite(config_.active_steer_deg) ||
        config_.active_steer_deg <= 0.0
    ) {
        throw std::invalid_argument(
            "Invalid MajorExcursionConfig"
        );
    }
}

std::vector<MajorExcursion>
MajorExcursionDetector::analyze(
    const p4::io::VBOData& data,
    const std::vector<ManeuverEvent>& events
) const {
    const auto n = data.sampleCount();

    if (n < 3) {
        throw std::runtime_error(
            "Insufficient samples"
        );
    }

    if (!data.hasColumn("steer_ang2")) {
        throw std::runtime_error(
            "Missing steer_ang2"
        );
    }

    std::vector<double> t(n);
    std::vector<double> steer(n);

    const double t0 = data.timeAt(0);

    for (std::size_t i = 0; i < n; ++i) {
        t[i] = data.timeAt(i) - t0;
        steer[i] = data.value(i, "steer_ang2");

        if (!std::isfinite(t[i]) ||
            !std::isfinite(steer[i])) {
            throw std::runtime_error(
                "Non-finite sample"
            );
        }
    }

    const auto filtered = smoothSignal(
        t, steer, config_.smooth_seconds
    );

    std::vector<MajorExcursion> result;

    for (const auto& event : events) {
        const auto lo = event.start_index;
        const auto hi = event.end_index;

        if (lo >= hi || hi > n) {
            throw std::invalid_argument(
                "Invalid event indices"
            );
        }

        if (hi - lo < 3) {
            continue;
        }

        // Smooth the full recording first, then
        // extract the event, as in the Python code.
        const std::vector<double> localT(
            t.begin() + lo,
            t.begin() + hi
        );

        const std::vector<double> localSteer(
            filtered.begin() + lo,
            filtered.begin() + hi
        );

        const auto anchors = turningPoints(
            localT, localSteer, config_
        );

        std::size_t excursionId = 0;

        for (std::size_t j = 1;
             j < anchors.size(); ++j) {

            const auto a = anchors[j - 1];
            const auto b = anchors[j];

            if (b <= a) {
                continue;
            }

            MajorExcursion excursion;

            excursion.event_id = event.event_id;
            excursion.excursion_id = ++excursionId;

            excursion.start_index = lo + a;
            excursion.end_index = lo + b;

            excursion.start_s = localT[a];
            excursion.end_s = localT[b];

            excursion.steer_start_deg =
                localSteer[a];

            excursion.steer_end_deg =
                localSteer[b];

            excursion.steering_delta_deg =
                localSteer[b] - localSteer[a];

            excursion.type = classifyInterval(
                localSteer, a, b, config_
            );

            result.push_back(excursion);
        }
    }

    return result;
}

} // namespace p4::maneuvers

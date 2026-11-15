
#include "p4/maneuvers/ManeuverDetector.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace p4::maneuvers {

ManeuverDetector::ManeuverDetector(
    ManeuverDetectorConfig config
) : config_(config) {
    if (config_.min_steering_change_deg <= 0.0 ||
        config_.min_onset_change_deg <= 0.0 ||
        config_.max_rise_time <= 0.0 ||
        config_.min_hold_time <= 0.0 ||
        config_.hold_tolerance_deg < 0.0 ||
        config_.max_sample_gap <= 0.0) {
        throw std::invalid_argument(
            "Invalid ManeuverDetector configuration"
        );
    }
}

std::vector<Maneuver> ManeuverDetector::detectStepSteer(
    const p4::io::VBOData& data,
    const std::string& steering_column
) const {
    std::vector<Maneuver> results;

    const std::size_t n = data.sampleCount();

    if (n < 3) {
        return results;
    }

    if (!data.hasColumn(steering_column)) {
        throw std::invalid_argument(
            "Steering column not found: " +
            steering_column
        );
    }

    std::vector<double> time(n);
    std::vector<double> steer(n);

    for (std::size_t k = 0; k < n; ++k) {
        time[k] = data.timeAt(k);
        steer[k] = data.value(k, steering_column);

        if (!std::isfinite(time[k]) ||
            !std::isfinite(steer[k])) {
            throw std::runtime_error(
                "Non-finite maneuver signal"
            );
        }

        if (k > 0 && time[k] <= time[k - 1]) {
            throw std::runtime_error(
                "Non-increasing maneuver timestamp"
            );
        }
    }

    std::size_t i = 0;

    while (i + 2 < n) {

        // Skip gaps and insignificant movements.
        const double first_dt = time[i + 1] - time[i];

        if (first_dt > config_.max_sample_gap ||
            std::abs(steer[i + 1] - steer[i]) <
                config_.min_onset_change_deg) {
            ++i;
            continue;
        }

        bool detected = false;

        // Search for the end of the rapid steering change.
        for (std::size_t j = i + 1; j < n; ++j) {
            const double rise = time[j] - time[i];

            if (rise > config_.max_rise_time) {
                break;
            }

            if (time[j] - time[j - 1] >
                config_.max_sample_gap) {
                break;
            }

            const double change = steer[j] - steer[i];

            if (std::abs(change) <
                config_.min_steering_change_deg) {
                continue;
            }

            // Check whether the new steering level is held.
            std::size_t k = j;

            while (k + 1 < n) {
                if (time[k + 1] - time[k] >
                    config_.max_sample_gap) {
                    break;
                }

                if (std::abs(steer[k + 1] - steer[j]) >
                    config_.hold_tolerance_deg) {
                    break;
                }

                ++k;
            }

            const double hold = time[k] - time[j];

            if (hold < config_.min_hold_time) {
                continue;
            }

            Maneuver maneuver;
            maneuver.type = ManeuverType::StepSteer;

            maneuver.start_index = i;
            maneuver.end_index = k + 1;

            maneuver.start_time = time[i];
            maneuver.end_time = time[k];

            maneuver.steering_delta_deg = change;
            maneuver.rise_time = rise;
            maneuver.hold_time = hold;

            results.push_back(maneuver);

            // Do not repeatedly detect the same event.
            i = k;
            detected = true;
            break;
        }

        if (!detected) {
            ++i;
        }
    }

    return results;
}

} // namespace p4::maneuvers


#include "p4/identification/IdentificationWindowSelector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace p4::identification {
namespace {

using Phase = p4::maneuvers::ManeuverPhase;
using PhaseType = p4::maneuvers::PhaseType;

struct Signals {
    std::vector<double> t;
    std::vector<double> steer;
    std::vector<double> yaw;
    std::vector<double> speed;
};

struct Statistics {
    std::size_t samples = 0;

    double mean_steer = 0.0;
    double mean_yaw = 0.0;
    double mean_speed = 0.0;

    double steer_range = 0.0;
    double yaw_range = 0.0;
    double speed_range = 0.0;

    bool same_steer_sign = true;
};

Signals readSignals(const p4::io::VBOData& data) {
    const auto n = data.sampleCount();

    if (n < 2) {
        throw std::runtime_error(
            "Insufficient VBO samples"
        );
    }

    for (const char* name :
         {"steer_ang2", "YawRate", "velocity"}) {
        if (!data.hasColumn(name)) {
            throw std::runtime_error(
                std::string("Missing column: ") + name
            );
        }
    }

    Signals result;

    result.t.reserve(n);
    result.steer.reserve(n);
    result.yaw.reserve(n);
    result.speed.reserve(n);

    const double t0 = data.timeAt(0);

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
                "Non-finite VBO signal"
            );
        }

        if (!result.t.empty() &&
            t <= result.t.back()) {
            throw std::runtime_error(
                "Non-increasing VBO time"
            );
        }

        result.t.push_back(t);
        result.steer.push_back(steer);
        result.yaw.push_back(yaw);
        result.speed.push_back(speed);
    }

    return result;
}

// [start, end) using original measurement samples.
Statistics calculateStats(
    const Signals& s,
    double start,
    double end
) {
    Statistics stats;

    const auto first = std::lower_bound(
        s.t.begin(), s.t.end(), start
    );

    const auto last = std::lower_bound(
        s.t.begin(), s.t.end(), end
    );

    const auto a = static_cast<std::size_t>(
        first - s.t.begin()
    );

    const auto b = static_cast<std::size_t>(
        last - s.t.begin()
    );

    if (a >= b) {
        return stats;
    }

    double minSteer = s.steer[a];
    double maxSteer = s.steer[a];

    double minYaw = s.yaw[a];
    double maxYaw = s.yaw[a];

    double minSpeed = s.speed[a];
    double maxSpeed = s.speed[a];

    bool positive = false;
    bool negative = false;

    for (std::size_t i = a; i < b; ++i) {
        const double steer = s.steer[i];
        const double yaw = s.yaw[i];
        const double speed = s.speed[i];

        stats.mean_steer += steer;
        stats.mean_yaw += yaw;
        stats.mean_speed += speed;

        minSteer = std::min(minSteer, steer);
        maxSteer = std::max(maxSteer, steer);

        minYaw = std::min(minYaw, yaw);
        maxYaw = std::max(maxYaw, yaw);

        minSpeed = std::min(minSpeed, speed);
        maxSpeed = std::max(maxSpeed, speed);

        positive |= steer > 0.0;
        negative |= steer < 0.0;
    }

    stats.samples = b - a;

    const double count =
        static_cast<double>(stats.samples);

    stats.mean_steer /= count;
    stats.mean_yaw /= count;
    stats.mean_speed /= count;

    stats.steer_range = maxSteer - minSteer;
    stats.yaw_range = maxYaw - minYaw;
    stats.speed_range = maxSpeed - minSpeed;

    stats.same_steer_sign = !(positive && negative);

    return stats;
}

bool isSteady(
    const Statistics& s,
    const IdentificationWindowConfig& c
) {
    return
        s.samples >= 2 &&
        s.same_steer_sign &&
        std::abs(s.mean_steer) >= c.min_abs_steer_deg &&
        s.steer_range <= c.max_steer_range_deg &&
        s.yaw_range <= c.max_yaw_range_deg_s &&
        s.speed_range <= c.max_speed_range_kmh &&
        s.mean_speed >= c.min_speed_kmh;
}

IdentificationWindow makeWindow(
    const Signals& signals,
    double start,
    double end,
    WindowType type,
    std::size_t eventId = 0,
    std::size_t phaseId = 0
) {
    const auto stats = calculateStats(
        signals, start, end
    );

    IdentificationWindow result;

    result.type = type;
    result.event_id = eventId;
    result.phase_id = phaseId;

    result.start_s = start;
    result.end_s = end;
    result.duration_s = end - start;

    result.mean_steer_deg = stats.mean_steer;
    result.mean_yaw_deg_s = stats.mean_yaw;
    result.mean_speed_kmh = stats.mean_speed;

    result.steer_range_deg = stats.steer_range;
    result.yaw_range_deg_s = stats.yaw_range;
    result.speed_range_kmh = stats.speed_range;

    return result;
}

bool isTransientPhase(PhaseType type) {
    return type == PhaseType::Entry ||
           type == PhaseType::Recovery ||
           type == PhaseType::Reversal;
}

} // anonymous namespace

std::string toString(WindowType type) {
    switch (type) {
        case WindowType::SteadyState:
            return "STEADY_STATE";

        case WindowType::Transient:
            return "TRANSIENT";
    }

    return "UNKNOWN";
}

IdentificationWindowSelector::IdentificationWindowSelector(
    IdentificationWindowConfig config
) : config_(config) {
    const double values[] = {
        config_.window_s,
        config_.min_steady_duration_s,
        config_.min_abs_steer_deg,
        config_.max_steer_range_deg,
        config_.max_yaw_range_deg_s,
        config_.max_speed_range_kmh,
        config_.min_speed_kmh,
        config_.min_transient_duration_s,
        config_.min_transient_steer_change_deg
    };

    for (double value : values) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument(
                "Invalid IdentificationWindowConfig"
            );
        }
    }

    if (config_.window_s <= 0.0 ||
        config_.min_steady_duration_s <= 0.0 ||
        config_.min_transient_duration_s <= 0.0) {
        throw std::invalid_argument(
            "Window durations must be positive"
        );
    }
}

IdentificationWindowResult
IdentificationWindowSelector::select(
    const p4::io::VBOData& data,
    const std::vector<Phase>& phases
) const {
    const auto signals = readSignals(data);

    IdentificationWindowResult result;

    const double duration = signals.t.back();
    const double step = config_.window_s;

    // =====================================================
    // 1. Steady-state windows
    // =====================================================

    bool inRun = false;
    double runStart = 0.0;
    double runEnd = 0.0;

    auto flushRun = [&]() {
        if (!inRun) {
            return;
        }

        const double runDuration = runEnd - runStart;

        if (runDuration + 1e-9 >=
            config_.min_steady_duration_s) {

            result.steady.push_back(
                makeWindow(
                    signals,
                    runStart,
                    runEnd,
                    WindowType::SteadyState
                )
            );
        }

        inRun = false;
    };

    // Integer indexing avoids floating-point
    // accumulation from repeated start += step.
    for (std::size_t k = 0;; ++k) {
        const double start = static_cast<double>(k) * step;
        const double end = start + step;

        // Reject the incomplete final window.
        if (end > duration + 1e-9) {
            break;
        }

        const auto stats =
            calculateStats(signals, start, end);

        if (isSteady(stats, config_)) {
            if (!inRun) {
                runStart = start;
                inRun = true;
            }

            runEnd = end;
        }
        else {
            flushRun();
        }
    }

    flushRun();

    // =====================================================
    // 2. Transient windows
    // =====================================================

    for (const auto& phase : phases) {
        if (!isTransientPhase(phase.type)) {
            continue;
        }

        if (!std::isfinite(phase.start_s) ||
            !std::isfinite(phase.end_s) ||
            phase.end_s <= phase.start_s ||
            phase.start_s < 0.0 ||
            phase.end_s > duration + 1e-6) {
            throw std::invalid_argument(
                "Invalid phase interval"
            );
        }

        const double phaseDuration =
            phase.end_s - phase.start_s;

        const double steerChange = std::abs(
            phase.steer_end_deg -
            phase.steer_start_deg
        );

        if (phaseDuration <
            config_.min_transient_duration_s) {
            continue;
        }

        if (steerChange <
            config_.min_transient_steer_change_deg) {
            continue;
        }

        const auto stats = calculateStats(
            signals,
            phase.start_s,
            phase.end_s
        );

        if (stats.samples < 2 ||
            stats.mean_speed < config_.min_speed_kmh) {
            continue;
        }

        result.transient.push_back(
            makeWindow(
                signals,
                phase.start_s,
                phase.end_s,
                WindowType::Transient,
                phase.event_id,
                phase.phase_id
            )
        );
    }

    return result;
}

} // namespace p4::identification

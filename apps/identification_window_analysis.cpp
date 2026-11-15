
#include "p4/io/VBOReader.hpp"
#include "p4/maneuvers/ManeuverSegmenter.hpp"
#include "p4/maneuvers/MajorExcursionDetector.hpp"
#include "p4/maneuvers/ManeuverPhaseAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Phase = p4::maneuvers::ManeuverPhase;
using PhaseType = p4::maneuvers::PhaseType;

// ============================================================
// 1. Configuration
// ============================================================

struct Limits {
    // Steady-state screening
    double window_s = 1.0;
    double min_steady_duration_s = 2.0;

    double min_abs_steer_deg = 3.0;
    double max_steer_range_deg = 1.5;
    double max_yaw_range_deg_s = 2.0;
    double max_speed_range_kmh = 2.0;
    double min_speed_kmh = 5.0;

    // Transient screening
    double min_transient_duration_s = 0.25;
    double min_transient_steer_change_deg = 3.0;
};

// ============================================================
// 2. Data Structures
// ============================================================

enum class WindowStatus {
    Qualified,
    Review
};

enum class WindowType {
    SteadyState,
    Transient
};

const char* toString(WindowStatus status) {
    switch (status) {
        case WindowStatus::Qualified:
            return "QUALIFIED";
        case WindowStatus::Review:
            return "REVIEW";
    }
    return "UNKNOWN";
}

struct WindowStats {
    double start_s = 0.0;
    double end_s = 0.0;

    double steer_mean = 0.0;
    double steer_min = 0.0;
    double steer_max = 0.0;

    double yaw_mean = 0.0;
    double yaw_min = 0.0;
    double yaw_max = 0.0;

    double speed_mean = 0.0;
    double speed_min = 0.0;
    double speed_max = 0.0;

    std::size_t samples = 0;

    bool same_steer_sign = true;
    bool stable_candidate = false;
};

struct IdentificationWindow {
    WindowType type = WindowType::SteadyState;
    WindowStatus status = WindowStatus::Review;

    std::size_t event_id = 0;
    std::size_t phase_id = 0;

    PhaseType phase_type = PhaseType::Transition;

    double start_s = 0.0;
    double end_s = 0.0;
    double duration_s = 0.0;

    std::size_t merged_windows = 0;

    double steer_change_deg = 0.0;

    WindowStats stats;
};

// ============================================================
// 3. Window Statistics
// ============================================================

WindowStats calculate(
    const p4::io::VBOData& data,
    const std::vector<double>& time,
    double start,
    double end,
    const Limits& limits
) {
    WindowStats w;
    w.start_s = start;
    w.end_s = end;

    if (!(end > start)) {
        return w;
    }

    w.steer_min = w.yaw_min = w.speed_min =
        std::numeric_limits<double>::infinity();

    w.steer_max = w.yaw_max = w.speed_max =
        -std::numeric_limits<double>::infinity();

    const auto first = std::lower_bound(
        time.begin(), time.end(), start
    );

    const auto last = std::lower_bound(
        time.begin(), time.end(), end
    );

    const auto a = static_cast<std::size_t>(
        first - time.begin()
    );

    const auto b = static_cast<std::size_t>(
        last - time.begin()
    );

    bool positive = false;
    bool negative = false;

    for (std::size_t i = a; i < b; ++i) {
        const double steer =
            data.value(i, "steer_ang2");
        const double yaw =
            data.value(i, "YawRate");
        const double speed =
            data.value(i, "velocity");

        // Do not silently omit invalid samples.
        if (!std::isfinite(steer) ||
            !std::isfinite(yaw) ||
            !std::isfinite(speed)) {
            throw std::runtime_error(
                "Non-finite measurement"
            );
        }

        ++w.samples;

        w.steer_mean += steer;
        w.yaw_mean += yaw;
        w.speed_mean += speed;

        w.steer_min = std::min(w.steer_min, steer);
        w.steer_max = std::max(w.steer_max, steer);

        w.yaw_min = std::min(w.yaw_min, yaw);
        w.yaw_max = std::max(w.yaw_max, yaw);

        w.speed_min = std::min(w.speed_min, speed);
        w.speed_max = std::max(w.speed_max, speed);

        positive |= steer > 0.0;
        negative |= steer < 0.0;
    }

    if (w.samples == 0) {
        return w;
    }

    const double n = static_cast<double>(w.samples);

    w.steer_mean /= n;
    w.yaw_mean /= n;
    w.speed_mean /= n;

    w.same_steer_sign = !(positive && negative);

    const double steer_range =
        w.steer_max - w.steer_min;

    const double yaw_range =
        w.yaw_max - w.yaw_min;

    const double speed_range =
        w.speed_max - w.speed_min;

    w.stable_candidate =
        w.samples >= 2 &&
        w.same_steer_sign &&
        std::abs(w.steer_mean) >=
            limits.min_abs_steer_deg &&
        steer_range <= limits.max_steer_range_deg &&
        yaw_range <= limits.max_yaw_range_deg_s &&
        speed_range <= limits.max_speed_range_kmh &&
        w.speed_mean >= limits.min_speed_kmh;

    return w;
}

// ============================================================
// 4. Basic Steady-State Windows
// ============================================================

std::vector<WindowStats> analyzeBasicWindows(
    const p4::io::VBOData& data,
    const std::vector<double>& time,
    const Limits& limits
) {
    std::vector<WindowStats> result;

    const double duration = time.back();

    for (std::size_t k = 0;; ++k) {
        const double start =
            static_cast<double>(k) * limits.window_s;

        const double end =
            start + limits.window_s;

        // Exclude incomplete final windows.
        if (end > duration + 1e-9) {
            break;
        }

        result.push_back(
            calculate(data, time, start, end, limits)
        );
    }

    return result;
}

// ============================================================
// 5. Merge Steady-State Windows
// ============================================================

std::vector<IdentificationWindow> mergeStableWindows(
    const p4::io::VBOData& data,
    const std::vector<double>& time,
    const std::vector<WindowStats>& basic,
    const Limits& limits
) {
    std::vector<IdentificationWindow> result;

    bool active = false;
    double run_start = 0.0;
    double run_end = 0.0;
    std::size_t run_count = 0;

    auto flushRun = [&]() {
        if (!active) {
            return;
        }

        const double duration = run_end - run_start;

        if (duration + 1e-9 >=
            limits.min_steady_duration_s) {

            IdentificationWindow window;

            window.type = WindowType::SteadyState;
            window.start_s = run_start;
            window.end_s = run_end;
            window.duration_s = duration;
            window.merged_windows = run_count;

            window.stats = calculate(
                data, time, run_start, run_end, limits
            );

            // Re-evaluate the entire merged interval.
            window.status =
                window.stats.stable_candidate
                    ? WindowStatus::Qualified
                    : WindowStatus::Review;

            result.push_back(window);
        }

        active = false;
        run_count = 0;
    };

    for (const auto& w : basic) {
        if (!w.stable_candidate) {
            flushRun();
            continue;
        }

        if (!active) {
            active = true;
            run_start = w.start_s;
            run_end = w.end_s;
            run_count = 1;
            continue;
        }

        if (std::abs(w.start_s - run_end) <= 1e-6) {
            run_end = w.end_s;
            ++run_count;
        }
        else {
            flushRun();

            active = true;
            run_start = w.start_s;
            run_end = w.end_s;
            run_count = 1;
        }
    }

    flushRun();
    return result;
}

// ============================================================
// 6. Transient Phase Screening
// ============================================================

bool isTransientPhase(PhaseType type) {
    return
        type == PhaseType::Entry ||
        type == PhaseType::Recovery ||
        type == PhaseType::Reversal;
}

std::vector<IdentificationWindow> selectTransientWindows(
    const p4::io::VBOData& data,
    const std::vector<double>& time,
    const std::vector<Phase>& phases,
    const Limits& limits
) {
    std::vector<IdentificationWindow> result;

    for (const auto& phase : phases) {
        if (!isTransientPhase(phase.type)) {
            continue;
        }

        const double start = phase.start_s;
        const double end = phase.end_s;
        const double duration = end - start;

        if (!std::isfinite(start) ||
            !std::isfinite(end) ||
            start < 0.0 ||
            end > time.back() + 1e-6 ||
            !(duration > 0.0)) {
            throw std::runtime_error(
                "Invalid transient phase interval"
            );
        }

        const double steer_change = std::abs(
            phase.steer_end_deg -
            phase.steer_start_deg
        );

        if (duration <
            limits.min_transient_duration_s) {
            continue;
        }

        if (steer_change <
            limits.min_transient_steer_change_deg) {
            continue;
        }

        const auto stats = calculate(
            data, time, start, end, limits
        );

        if (stats.samples < 2 ||
            stats.speed_mean < limits.min_speed_kmh) {
            continue;
        }

        IdentificationWindow window;

        window.type = WindowType::Transient;
        window.status = WindowStatus::Qualified;

        window.event_id = phase.event_id;
        window.phase_id = phase.phase_id;
        window.phase_type = phase.type;

        window.start_s = start;
        window.end_s = end;
        window.duration_s = duration;

        window.steer_change_deg = steer_change;
        window.stats = stats;

        result.push_back(window);
    }

    return result;
}

// ============================================================
// 7. Output
// ============================================================

void printBasicWindows(
    const std::vector<WindowStats>& windows
) {
    std::cout << "\n=== BASIC WINDOWS ===\n";

    for (const auto& w : windows) {
        std::cout
            << "[" << w.start_s << ", "
            << w.end_s << ") s"
            << " | N=" << w.samples;

        if (w.samples == 0) {
            std::cout << " | NO DATA\n";
            continue;
        }

        std::cout
            << " | Steer=" << w.steer_mean
            << " | Yaw=" << w.yaw_mean
            << " | Speed=" << w.speed_mean
            << " | "
            << (w.stable_candidate
                ? "STABLE_CANDIDATE"
                : "REVIEW")
            << '\n';
    }
}

void printSteady(
    const std::vector<IdentificationWindow>& windows
) {
    std::cout << "\n=== STEADY-STATE WINDOWS ===\n";

    std::size_t id = 0;

    for (const auto& w : windows) {
        std::cout
            << "#" << ++id
            << " [" << w.start_s
            << ", " << w.end_s << ") s"
            << " | Duration=" << w.duration_s
            << " | Merged=" << w.merged_windows
            << " | Steer=" << w.stats.steer_mean
            << " | Yaw=" << w.stats.yaw_mean
            << " | Speed=" << w.stats.speed_mean
            << " | " << toString(w.status)
            << '\n';
    }
}

void printTransient(
    const std::vector<IdentificationWindow>& windows
) {
    std::cout << "\n=== TRANSIENT WINDOWS ===\n";

    std::size_t id = 0;

    for (const auto& w : windows) {
        std::cout
            << "#" << ++id
            << " Event=" << w.event_id
            << " Phase=" << w.phase_id
            << " ("
            << p4::maneuvers::toString(w.phase_type)
            << ")"
            << " [" << w.start_s
            << ", " << w.end_s << ") s"
            << " | Duration=" << w.duration_s
            << " | DeltaSteer=" << w.steer_change_deg
            << " deg"
            << " | Speed=" << w.stats.speed_mean
            << " km/h"
            << " | " << toString(w.status)
            << '\n';
    }
}

} // anonymous namespace

// ============================================================
// 8. Main
// ============================================================

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr
            << "Usage: identification_window_analysis "
            << "<file.vbo>\n";
        return 1;
    }

    try {
        p4::io::VBOReader reader;
        p4::maneuvers::ManeuverSegmenter segmenter;
        p4::maneuvers::MajorExcursionDetector excursionDetector;
        p4::maneuvers::ManeuverPhaseAnalyzer phaseAnalyzer;

        const auto data = reader.read(argv[1]);

        for (const char* name :
             {"steer_ang2", "YawRate", "velocity"}) {
            if (!data.hasColumn(name)) {
                throw std::runtime_error(
                    std::string("Missing column: ") + name
                );
            }
        }

        const std::size_t n = data.sampleCount();

        if (n < 3) {
            throw std::runtime_error(
                "Insufficient samples"
            );
        }

        std::vector<double> time(n);
        const double t0 = data.timeAt(0);

        for (std::size_t i = 0; i < n; ++i) {
            time[i] = data.timeAt(i) - t0;

            if (!std::isfinite(time[i]) ||
                (i > 0 && time[i] <= time[i - 1])) {
                throw std::runtime_error(
                    "Invalid VBO timestamps"
                );
            }
        }

        const Limits limits;

        // Existing maneuver pipeline
        const auto events = segmenter.analyze(data);

        const auto excursions =
            excursionDetector.analyze(data, events);

        const auto phases = phaseAnalyzer.analyze(
            data, events, excursions
        );

        // Steady-state selection
        const auto basic = analyzeBasicWindows(
            data, time, limits
        );

        const auto steady = mergeStableWindows(
            data, time, basic, limits
        );

        // Transient selection
        const auto transient = selectTransientWindows(
            data, time, phases, limits
        );

        std::cout
            << std::fixed << std::setprecision(3)
            << "\n=== IDENTIFICATION WINDOW ANALYSIS ===\n"
            << "File       : " << argv[1] << '\n'
            << "Samples    : " << n << '\n'
            << "Events     : " << events.size() << '\n'
            << "Excursions : " << excursions.size() << '\n'
            << "Phases     : " << phases.size() << '\n';

        printBasicWindows(basic);
        printSteady(steady);
        printTransient(transient);

        std::size_t qualifiedSteady = 0;
        std::size_t reviewSteady = 0;

        for (const auto& w : steady) {
            if (w.status == WindowStatus::Qualified) {
                ++qualifiedSteady;
            }
            else {
                ++reviewSteady;
            }
        }

        std::cout
            << "\n=== SUMMARY ===\n"
            << "Basic windows      : " << basic.size()
            << '\n'
            << "Steady qualified   : " << qualifiedSteady
            << '\n'
            << "Steady review      : " << reviewSteady
            << '\n'
            << "Transient selected : " << transient.size()
            << '\n';

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr
            << "[ERROR] " << e.what() << '\n';
        return 1;
    }
}

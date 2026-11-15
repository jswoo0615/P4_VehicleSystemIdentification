#ifndef IDENTIFICATION_WINDOW_SELECTOR_HPP_
#define IDENTIFICATION_WINDOW_SELECTOR_HPP_

#include "p4/io/VBOData.hpp"
#include "p4/maneuvers/ManeuverPhaseAnalyzer.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace p4::identification {

enum class WindowType {
    SteadyState,
    Transient
};

struct IdentificationWindow {
    WindowType type = WindowType::SteadyState;

    std::size_t event_id = 0;
    std::size_t phase_id = 0;

    double start_s = 0.0;
    double end_s = 0.0;
    double duration_s = 0.0;

    double mean_steer_deg = 0.0;
    double mean_yaw_deg_s = 0.0;
    double mean_speed_kmh = 0.0;

    double steer_range_deg = 0.0;
    double yaw_range_deg_s = 0.0;
    double speed_range_kmh = 0.0;
};

struct IdentificationWindowConfig {
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

struct IdentificationWindowResult {
    std::vector<IdentificationWindow> steady;
    std::vector<IdentificationWindow> transient;
};

std::string toString(WindowType type);

class IdentificationWindowSelector {
public:
    explicit IdentificationWindowSelector(
        IdentificationWindowConfig config = {}
    );

    IdentificationWindowResult select(
        const p4::io::VBOData& data,
        const std::vector<
            p4::maneuvers::ManeuverPhase
        >& phases
    ) const;

private:
    IdentificationWindowConfig config_;
};

} // namespace p4::identification
#endif // IDENTIFICATION_WINDOW_SELECTOR_HPP_
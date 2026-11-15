#ifndef MANEUVER_HPP_
#define MANEUVER_HPP_

#include <cstddef>

namespace p4::maneuvers {
    enum class ManeuverType {
        Unknown,
        StraightDriving,
        StepSteer,
        RampSteer,
        SineSteer,
        SineWithDwell,
        DoubleLaneChange,
        StepAcceleration,
        StepBraking
    };

    struct Maneuver {
        ManeuverType type = ManeuverType::Unknown;

        // [start_index, end_index]
        std::size_t start_index = 0;
        std::size_t end_index = 0;

        double start_time = 0.0;
        double end_time = 0.0;

        // Change in measured steering angle
        double steering_delta_deg = 0.0;

        // Time taken to reach the new steering level
        double rise_time = 0.0;

        // Stable time after the steering change
        double hold_time = 0.0;
    };
} // namespace p4::maneuvers

#endif // MANEUVER_HPP_
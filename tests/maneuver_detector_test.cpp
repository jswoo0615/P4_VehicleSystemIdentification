
#include "p4/maneuvers/ManeuverDetector.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

p4::io::VBOData makeTestData(bool step) {
    p4::io::VBOData data;

    data.columns = {"steer_ang2"};
    data.original_columns = data.columns;

    for (int i = 0; i < 120; ++i) {
        const double t = i * 0.01;
        double steering = 0.0;

        if (step) {
            if (i >= 30 && i < 35) {
                steering = (i - 29) * 1.0;
            }
            else if (i >= 35) {
                steering = 5.0;
            }
        }
        else {
            // Slow steering ramp
            steering = i * 0.02;
        }

        p4::io::VBORawSample sample;
        sample.time = t;
        sample.values = {steering};
        sample.source_line =
            static_cast<std::size_t>(i + 1);

        data.samples.push_back(sample);
    }

    return data;
}

} // namespace

int main() {
    try {
        p4::maneuvers::ManeuverDetector detector;

        const auto step_data = makeTestData(true);
        const auto step_results =
            detector.detectStepSteer(step_data);

        require(
            step_results.size() == 1,
            "Expected exactly one Step Steer"
        );

        const auto& m = step_results.front();

        require(
            m.type ==
                p4::maneuvers::ManeuverType::StepSteer,
            "Incorrect maneuver type"
        );

        require(
            std::abs(m.steering_delta_deg - 5.0)
                < 1e-9,
            "Incorrect steering change"
        );

        require(
            m.start_index == 29,
            "Incorrect onset index"
        );

        std::cout
            << "[PASS] Step Steer Detected\n"
            << "  Start : " << m.start_time << " s\n"
            << "  End   : " << m.end_time << " s\n"
            << "  Delta : "
            << m.steering_delta_deg << " deg\n"
            << "  Rise  : " << m.rise_time << " s\n"
            << "  Hold  : " << m.hold_time << " s\n";

        const auto ramp_data = makeTestData(false);
        const auto ramp_results =
            detector.detectStepSteer(ramp_data);

        require(
            ramp_results.empty(),
            "Slow ramp must not be detected as Step Steer"
        );

        std::cout
            << "[PASS] Slow Ramp Rejected\n";

        std::cout
            << "ALL MANEUVER DETECTOR TESTS PASSED\n";

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr
            << "[FAIL] " << e.what() << '\n';

        return 1;
    }
}

#ifndef STEERING_ACTUATOR_MODEL_HPP_
#define STEERING_ACTUATOR_MODEL_HPP_

#include <array>
#include <cstddef>

namespace p4::models {
    struct SteeringParameters {
        double gain = 1.0;
        double tau = 0.2;
        double delay = 0.03;
        double bias = 0.0;
    };

    class SteeringActuatorModel {
        public:
            static constexpr std::size_t MaxHistory = 1024;
            SteeringActuatorModel(const SteeringParameters& params, double sample_time);
            void reset(double initial_angle, double initial_command=0.0);
            double step(double steering_command);
            double steeringAngle() const noexcept;
            const SteeringParameters& parameters() const noexcept;

        private:
            SteeringParameters params_;
            double dt_;
            double a_;

            std::size_t delay_steps_;
            std::size_t write_index_ = 0;

            std::array<double, MaxHistory> history_{};

            double steering_angle_ = 0.0;

    };
} // namespace p4::models

#endif // STEERING_ACTUATOR_MODEL_HPP_
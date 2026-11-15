#include "p4/models/FirstOrderYawModel.hpp"

#include <cmath>
#include <stdexcept>

namespace p4::models {
    FirstOrderYawModel::FirstOrderYawModel(const YawParameters& params) : params_(params) {
        if (!std::isfinite(params_.gain) ||
            !std::isfinite(params_.tau) ||
            !std::isfinite(params_.bias) ||
            params_.tau <= 0.0) {
            throw std::invalid_argument("Invalid yaw model parameters");
        }
    }

    void FirstOrderYawModel::reset(double yaw_rate) {
        if (!std::isfinite(yaw_rate)) {
            throw std::invalid_argument("Invalid yaw rate");
        }

        yaw_rate_ = yaw_rate;
    }

    double FirstOrderYawModel::step(double steering_angle, double dt) {
        if (!std::isfinite(steering_angle) ||
            !std::isfinite(dt) || dt <= 0.0) {
            throw std::invalid_argument("Ivalid steering angle or dt");
        }

        const double a = std::exp(-dt / params_.tau);

        const double r_ss = params_.gain * steering_angle + params_.bias;

        yaw_rate_ = a * yaw_rate_ + (1.0 - a) * r_ss;

        return yaw_rate_;
    }

    double FirstOrderYawModel::yawRate() const noexcept {
        return yaw_rate_;
    }

    const YawParameters& FirstOrderYawModel::parameters() const noexcept {
        return params_;
    }
} // namespace p4::models

#ifndef P4_MODELS_MODEL_A_ROLLOUT_HPP_
#define P4_MODELS_MODEL_A_ROLLOUT_HPP_

#include <cmath>
#include <stdexcept>
#include <vector>

namespace p4::models {

// Model A: zero-order-held steering input and exact first-order update.
inline std::vector<double> simulateModelA(
    const std::vector<double>& time,
    const std::vector<double>& steering,
    double initial_yaw_rate,
    double tau,
    double gain
) {
    if (time.empty() || time.size() != steering.size() ||
        !std::isfinite(initial_yaw_rate) || !std::isfinite(tau) ||
        !std::isfinite(gain) || tau <= 0.0) {
        throw std::invalid_argument("Invalid Model A rollout inputs");
    }

    std::vector<double> predicted(time.size());
    predicted[0] = initial_yaw_rate;
    for (std::size_t k = 1; k < time.size(); ++k) {
        const double dt = time[k] - time[k - 1];
        if (!std::isfinite(dt) || dt <= 0.0 ||
            !std::isfinite(steering[k - 1])) {
            throw std::invalid_argument("Invalid Model A sample");
        }
        const double a = std::exp(-dt / tau);
        predicted[k] = a * predicted[k - 1] +
            (1.0 - a) * gain * steering[k - 1];
    }
    return predicted;
}

} // namespace p4::models

#endif

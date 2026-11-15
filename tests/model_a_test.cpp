#include "p4/models/FirstOrderYawModel.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    p4::models::FirstOrderYawModel model;

    const double dt = 0.01;
    const double steering = -2.0;

    model.reset(0.0);

    const auto& p = model.parameters();
    const double target = p.gain * steering + p.bias;

    std::cout << std::fixed << std::setprecision(6);

    std::cout << "Target yaw rate : " << target << " deg/s\n\n";

    for (int k = 1; k <= 100; ++k) {
        const double r = model.step(steering, dt);

        if (k % 10 == 0) {
            std::cout << "t = " << k * dt << " s, r = " << r << " deg/s\n";
        }
    }

    const double expected = target * (1.0 - std::exp(-1.0 / p.tau));

    const double error = std::abs(model.yawRate() - expected);

    std::cout << "\nFinal Error : " << error << "\n";

    return error < 1e-10 ? 0 : 1;

}
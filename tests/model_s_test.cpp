
#include "p4/models/SteeringActuatorModel.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <algorithm>

namespace {

constexpr double pi = 3.14159265358979323846;

double degToRad(double deg) {
    return deg * pi / 180.0;
}

double radToDeg(double rad) {
    return rad * 180.0 / pi;
}

} // namespace

int main() {
    using p4::models::SteeringActuatorModel;
    using p4::models::SteeringParameters;

    SteeringParameters params;
    params.gain = 1.0;
    params.tau = 0.2;
    params.delay = 0.03;
    params.bias = 0.0;

    constexpr double dt = 0.01;
    constexpr int steps = 100;

    const double command = degToRad(10.0);

    SteeringActuatorModel model(params, dt);
    model.reset(0.0, 0.0);

    std::cout << std::fixed
              << std::setprecision(9);

    for (int k = 0; k < steps; ++k) {
        const double angle = model.step(command);

        // step() 호출 후의 실제 시뮬레이션 시간
        const double t = (k + 1) * dt;

        if ((k + 1) % 10 == 0) {
            std::cout
                << "t = " << t
                << " s, angle = "
                << radToDeg(angle)
                << " deg\n";
        }
    }

    // 반드시 시뮬레이션 종료 시각과 동일하게 계산
    const double final_time = steps * dt;

    const double elapsed =
        std::max(0.0, final_time - params.delay);

    const double expected =
        command * params.gain *
        (1.0 - std::exp(-elapsed / params.tau))
        + params.bias *
        (1.0 - std::exp(-final_time / params.tau));

    const double actual = model.steeringAngle();
    const double error = std::abs(actual - expected);

    std::cout << "\nExpected : "
              << radToDeg(expected) << " deg\n";

    std::cout << "Actual   : "
              << radToDeg(actual) << " deg\n";

    std::cout << "Error    : "
              << error << " rad\n";

    return error < 1e-10 ? 0 : 1;
}

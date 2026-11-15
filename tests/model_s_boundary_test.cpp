
#include "p4/models/SteeringActuatorModel.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using p4::models::SteeringActuatorModel;
using p4::models::SteeringParameters;

constexpr double dt = 0.01;
constexpr double tolerance = 1e-10;

bool near(double actual, double expected) {
    return std::abs(actual - expected) < tolerance;
}

bool report(const std::string& name, bool pass) {
    std::cout
        << (pass ? "[PASS] " : "[FAIL] ")
        << name << '\n';

    return pass;
}

// S1: Zero Delay
bool testZeroDelay() {
    SteeringParameters p;
    p.gain = 1.0;
    p.tau = 0.2;
    p.delay = 0.0;
    p.bias = 0.0;

    SteeringActuatorModel model(p, dt);
    model.reset(0.0, 0.0);

    const double command = 0.1;
    const double a = std::exp(-dt / p.tau);

    const double actual = model.step(command);
    const double expected = (1.0 - a) * command;

    return report("S1 Zero Delay",
                  near(actual, expected));
}

// S2: Exactly three samples of delay
bool testDelayBoundary() {
    SteeringParameters p;
    p.gain = 1.0;
    p.tau = 0.2;
    p.delay = 0.03;
    p.bias = 0.0;

    SteeringActuatorModel model(p, dt);
    model.reset(0.0, 0.0);

    const double command = 0.1;
    const double a = std::exp(-dt / p.tau);

    bool pass = true;

    // k=0,1,2: 아직 지연된 명령은 0
    for (int k = 0; k < 3; ++k) {
        pass &= near(model.step(command), 0.0);
    }

    // k=3: 첫 유효 명령이 반영됨
    const double actual = model.step(command);
    const double expected = (1.0 - a) * command;

    pass &= near(actual, expected);

    return report("S2 Delay Boundary", pass);
}

// S3: Reset clears state and command history
bool testReset() {
    SteeringParameters p;
    p.gain = 1.0;
    p.tau = 0.2;
    p.delay = 0.03;
    p.bias = 0.0;

    SteeringActuatorModel model(p, dt);

    // 기존 명령으로 충분히 구동
    for (int k = 0; k < 100; ++k) {
        model.step(0.2);
    }

    // 모든 상태와 명령 이력을 초기화
    model.reset(0.0, 0.0);

    bool pass = near(model.steeringAngle(), 0.0);

    // 초기화 후 0 명령에 응답이 남지 않아야 함
    for (int k = 0; k < 10; ++k) {
        pass &= near(model.step(0.0), 0.0);
    }

    return report("S3 Reset", pass);
}

// S4: Nonzero steering bias
bool testBias() {
    SteeringParameters p;
    p.gain = 1.0;
    p.tau = 0.2;
    p.delay = 0.0;
    p.bias = 0.02;

    SteeringActuatorModel model(p, dt);
    model.reset(0.0, 0.0);

    constexpr int steps = 100;

    for (int k = 0; k < steps; ++k) {
        model.step(0.0);
    }

    const double expected =
        p.bias * (1.0 - std::exp(
            -(steps * dt) / p.tau
        ));

    return report(
        "S4 Steering Bias",
        near(model.steeringAngle(), expected)
    );
}

// S5: Reject unsupported fractional delay
bool testInvalidDelay() {
    SteeringParameters p;
    p.delay = 0.025;  // 2.5 samples

    try {
        SteeringActuatorModel model(p, dt);
        return report("S5 Invalid Delay", false);
    }
    catch (const std::invalid_argument&) {
        return report("S5 Invalid Delay", true);
    }
}

} // anonymous namespace

int main() {
    std::cout
        << "=== Model S Boundary Tests ===\n";

    bool pass = true;

    pass &= testZeroDelay();
    pass &= testDelayBoundary();
    pass &= testReset();
    pass &= testBias();
    pass &= testInvalidDelay();

    std::cout
        << (pass ? "\nALL TESTS PASSED\n"
                 : "\nTEST FAILURE\n");

    return pass ? 0 : 1;
}

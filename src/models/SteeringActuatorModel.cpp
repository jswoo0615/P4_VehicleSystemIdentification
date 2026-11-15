#include "p4/models/SteeringActuatorModel.hpp"

#include <cmath>
#include <stdexcept>

namespace p4::models {
    SteeringActuatorModel::SteeringActuatorModel(const SteeringParameters& params, double sample_time) : params_(params), dt_(sample_time), a_(0.0), delay_steps_(0) {
        if (!std::isfinite(params_.gain) ||
            !std::isfinite(params_.tau) ||
            !std::isfinite(params_.delay) ||
            !std::isfinite(params_.bias) ||
            !std::isfinite(dt_) ||
            params_.tau <= 0.0 || params_.delay < 0.0 || dt_ <= 0.0) {
            throw std::invalid_argument("Invalid steering parameters");
        }

        a_ = std::exp(-dt_ / params_.tau);

        const double steps = params_.delay / dt_;

        // 이번 구현은 정수 샘플 지연만 지원
        const double rounded = std::round(steps);

        if (!std::isfinite(steps) ||
            rounded < 0.0 ||
            rounded >= static_cast<double>(MaxHistory) ||
            std::abs(steps - rounded) > 1e-8) {
            throw std::invalid_argument(
                "Delay must be a supported integer "
                "multiple of sample time"
            );
        }

        delay_steps_ = static_cast<std::size_t>(rounded);

        reset(0.0, 0.0);
    }

    void SteeringActuatorModel::reset(double initial_angle, double initial_command) {
        if (!std::isfinite(initial_angle) ||
            !std::isfinite(initial_command)) {
            throw std::invalid_argument("Invalid initial state");
        }

        steering_angle_ = initial_angle;

        // 시뮬레이션 시작 전 명령 이력 초기화
        history_.fill(initial_command);
        write_index_ = 0;
    }

    double SteeringActuatorModel::step(double steering_command) {
        if (!std::isfinite(steering_command)) {
            throw std::invalid_argument("Invalid steering command");
        }

        // 현재 시각 k의 명령을 기록
        history_[write_index_] = steering_command;

        // u[k - m]에 해당하는 명령 조회
        const std::size_t read_index = (write_index_ + MaxHistory - delay_steps_) % MaxHistory;
        const double delayed_command = history_[read_index];

        // 다음 샘플에 사용할 버퍼 위치
        write_index_ = (write_index_ + 1) % MaxHistory;

        // 현재 지연 명령에서의 정상상태 목표값
        const double target_angle = params_.gain * delayed_command + params_.bias;

        // Exact Zero Order Holds discretization
        steering_angle_ = a_ * steering_angle_ + (1.0 - a_) * target_angle;

        return steering_angle_;
    }

    double SteeringActuatorModel::steeringAngle() const noexcept {
        return steering_angle_;
    }

    const SteeringParameters& SteeringActuatorModel::parameters() const noexcept {
        return params_;
    }
} // namespace p4::models

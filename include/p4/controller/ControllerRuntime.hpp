#ifndef P4_CONTROLLER_RUNTIME_HPP_
#define P4_CONTROLLER_RUNTIME_HPP_

#include "p4/io/ControllerModel.hpp"
#include "p4/models/FirstOrderYawModel.hpp"
#include "p4/models/LinearBicycleModel.hpp"

namespace p4::controller {

struct ControlInput {
    double steering_rad;
    double vx_m_s;
    double bank_rad;
    double dt_s;
};

struct ControlOutput {
    double model_a_yaw_rad_s;
    double model_a_speed_yaw_rad_s;
    double model_b0_vy_m_s;
    double model_b0_yaw_rad_s;
};

class ControllerRuntime {
public:
    explicit ControllerRuntime(const io::ControllerModel& parameters);
    void reset(double yaw_rad_s, double vy_m_s);
    ControlOutput step(const ControlInput& input);
private:
    io::ControllerModel parameters_;
    models::FirstOrderYawModel model_a_;
    models::LinearBicycleModel model_b0_;
    double a_speed_yaw_=0;
};

} // namespace p4::controller
#endif

#include "p4/models/LinearBicycleModel.hpp"

#include <cmath>
#include <stdexcept>

int main() {
    p4::models::LinearBicycleModel model;
    auto s = model.step(0.0, 10.0, 0.01);
    if (s.lateral_velocity_m_s != 0.0 || s.yaw_rate_rad_s != 0.0) return 1;

    s = model.step(0.01, 10.0, 0.01);
    if (std::abs(s.lateral_velocity_m_s - 0.005333333333333333) > 1e-12 ||
        std::abs(s.yaw_rate_rad_s - 0.00384) > 1e-12) return 2;

    const auto next = model.step(0.0, 10.0, 0.01);
    if (!(next.lateral_velocity_m_s < s.lateral_velocity_m_s &&
          next.yaw_rate_rad_s < s.yaw_rate_rad_s)) return 5;

    model.reset({0.2, 0.1});
    if (std::abs(model.state().lateral_velocity_m_s - 0.2) > 1e-12 ||
        std::abs(model.state().yaw_rate_rad_s - 0.1) > 1e-12) return 3;

    p4::models::BicycleParameters kit{2254.0,2193.0,1.563,1.437,234008.0,234008.0};
    p4::models::LinearBicycleModel stable(kit),reference(kit);
    stable.reset({0.3,0.2}); reference.reset({0.3,0.2});
    const auto implicit=stable.stepImplicitMidpoint(0.08,3.0,0.02,0.002);
    for(int i=0;i<1000;++i) reference.step(0.08,3.0,0.00002);
    if(std::abs(implicit.lateral_velocity_m_s-reference.state().lateral_velocity_m_s)>1e-3 ||
       std::abs(implicit.yaw_rate_rad_s-reference.state().yaw_rate_rad_s)>1e-3) return 6;

    p4::models::LinearBicycleModel banked({1000.0,1000.0,1.0,1.0,10000.0,10000.0});
    const auto banked_state=banked.stepImplicitMidpoint(0.0,10.0,0.01,0.005,1.0);
    if(!(banked_state.lateral_velocity_m_s>0.0) ||
       std::abs(banked_state.yaw_rate_rad_s)>1e-12) return 7;

    try {
        model.step(0.0, 0.0, 0.01);
        return 4;
    } catch (const std::invalid_argument&) {
        return 0;
    }
}

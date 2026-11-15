#include "p4/controller/ControllerRuntime.hpp"

#include <cmath>
#include <stdexcept>

namespace p4::controller {

ControllerRuntime::ControllerRuntime(const io::ControllerModel& p)
    : parameters_(p),
      model_a_({p.a.gain,p.a.tau,p.a.bias}),
      model_b0_({1.0,p.b0.iz_over_m,p.b0.lf,p.b0.lr,p.b0.cf_over_m,p.b0.cr_over_m}) {}

void ControllerRuntime::reset(double yaw_rad_s,double vy_m_s) {
    if(!std::isfinite(yaw_rad_s)||!std::isfinite(vy_m_s))
        throw std::invalid_argument("Invalid controller initial state");
    model_a_.reset(yaw_rad_s);
    model_b0_.reset({vy_m_s,yaw_rad_s});
    a_speed_yaw_=yaw_rad_s;
}

ControlOutput ControllerRuntime::step(const ControlInput& in) {
    if(!std::isfinite(in.steering_rad)||!std::isfinite(in.vx_m_s)||
       !std::isfinite(in.bank_rad)||!std::isfinite(in.dt_s)||
       in.vx_m_s<=0||in.dt_s<=0)
        throw std::invalid_argument("Invalid controller input");
    const auto& a=parameters_.a_speed;
    const double alpha=std::exp(-in.dt_s/a.tau);
    a_speed_yaw_=alpha*a_speed_yaw_+(1-alpha)*(
        in.vx_m_s*in.steering_rad/(a.wheelbase+a.kv*in.vx_m_s*in.vx_m_s)+a.bias);
    const double yaw_a=model_a_.step(in.steering_rad,in.dt_s);
    const auto b=model_b0_.stepImplicitMidpoint(in.steering_rad,in.vx_m_s,
        in.dt_s,.02,9.80665*std::sin(in.bank_rad));
    return {yaw_a,a_speed_yaw_,b.lateral_velocity_m_s,b.yaw_rate_rad_s};
}
} // namespace p4::controller

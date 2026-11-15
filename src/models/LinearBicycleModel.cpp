#include "p4/models/LinearBicycleModel.hpp"

#include <cmath>
#include <stdexcept>

namespace p4::models {

LinearBicycleModel::LinearBicycleModel(const BicycleParameters& parameters)
    : parameters_(parameters) {
    const auto& p = parameters_;
    if (!std::isfinite(p.mass_kg) || p.mass_kg <= 0.0 ||
        !std::isfinite(p.yaw_inertia_kg_m2) || p.yaw_inertia_kg_m2 <= 0.0 ||
        !std::isfinite(p.front_axle_m) || p.front_axle_m <= 0.0 ||
        !std::isfinite(p.rear_axle_m) || p.rear_axle_m <= 0.0 ||
        !std::isfinite(p.front_cornering_stiffness_n_rad) ||
        p.front_cornering_stiffness_n_rad <= 0.0 ||
        !std::isfinite(p.rear_cornering_stiffness_n_rad) ||
        p.rear_cornering_stiffness_n_rad <= 0.0) {
        throw std::invalid_argument("Invalid Model B parameters");
    }
}

void LinearBicycleModel::reset(const BicycleState& initial) {
    if (!std::isfinite(initial.lateral_velocity_m_s) ||
        !std::isfinite(initial.yaw_rate_rad_s)) {
        throw std::invalid_argument("Invalid Model B initial state");
    }
    state_ = initial;
}

BicycleState LinearBicycleModel::step(double front_steering_rad,
                                     double longitudinal_speed_m_s,
                                     double dt_s) {
    if (!std::isfinite(front_steering_rad) ||
        !std::isfinite(longitudinal_speed_m_s) ||
        longitudinal_speed_m_s <= 0.0 ||
        !std::isfinite(dt_s) || dt_s <= 0.0) {
        throw std::invalid_argument("Invalid Model B input or timestep");
    }

    const auto& p = parameters_;
    const double vx = longitudinal_speed_m_s;
    const double vy = state_.lateral_velocity_m_s;
    const double r = state_.yaw_rate_rad_s;
    const double cf = p.front_cornering_stiffness_n_rad;
    const double cr = p.rear_cornering_stiffness_n_rad;
    const double lf = p.front_axle_m;
    const double lr = p.rear_axle_m;

    // Linear tire forces: Fyf = Cf*(delta - (vy + lf*r)/vx),
    // Fyr = Cr*(-(vy - lr*r)/vx).
    const double front_force = cf * (front_steering_rad - (vy + lf * r) / vx);
    const double rear_force = cr * (-(vy - lr * r) / vx);
    const double vy_dot = (front_force + rear_force) / p.mass_kg - vx * r;
    const double r_dot = (lf * front_force - lr * rear_force) /
                         p.yaw_inertia_kg_m2;

    // Explicit Euler; caller controls the integration timestep.
    state_.lateral_velocity_m_s += dt_s * vy_dot;
    state_.yaw_rate_rad_s += dt_s * r_dot;
    if (!std::isfinite(state_.lateral_velocity_m_s) ||
        !std::isfinite(state_.yaw_rate_rad_s)) {
        throw std::overflow_error("Model B state diverged");
    }
    return state_;
}

BicycleState LinearBicycleModel::stepImplicitMidpoint(double delta, double vx,
                                                      double dt, double max_substep,
                                                      double lateral_external_accel) {
    if (!std::isfinite(delta) || !std::isfinite(vx) || vx <= 0.0 ||
        !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(lateral_external_accel) ||
        !std::isfinite(max_substep) ||
        max_substep <= 0.0)
        throw std::invalid_argument("Invalid Model B input or timestep");
    const auto& p = parameters_;
    const double cf=p.front_cornering_stiffness_n_rad;
    const double cr=p.rear_cornering_stiffness_n_rad;
    const double lf=p.front_axle_m, lr=p.rear_axle_m;
    const double a11=-(cf+cr)/(p.mass_kg*vx);
    const double a12=(-lf*cf+lr*cr)/(p.mass_kg*vx)-vx;
    const double a21=(-lf*cf+lr*cr)/(p.yaw_inertia_kg_m2*vx);
    const double a22=-(lf*lf*cf+lr*lr*cr)/(p.yaw_inertia_kg_m2*vx);
    const double b1=cf*delta/p.mass_kg+lateral_external_accel;
    const double b2=lf*cf*delta/p.yaw_inertia_kg_m2;
    if(dt/max_substep>1e6)
        throw std::invalid_argument("Model B integration requires too many substeps");
    const auto steps=static_cast<unsigned>(std::ceil(dt/max_substep));
    const double h=dt/steps;
    const double m11=1-h*a11/2, m12=-h*a12/2;
    const double m21=-h*a21/2, m22=1-h*a22/2;
    const double det=m11*m22-m12*m21;
    if (!std::isfinite(det) || std::abs(det)<1e-15)
        throw std::overflow_error("Model B integration matrix is singular");
    for(unsigned i=0;i<steps;++i) {
        const double y1=(1+h*a11/2)*state_.lateral_velocity_m_s+
                        (h*a12/2)*state_.yaw_rate_rad_s+h*b1;
        const double y2=(h*a21/2)*state_.lateral_velocity_m_s+
                        (1+h*a22/2)*state_.yaw_rate_rad_s+h*b2;
        state_={(y1*m22-m12*y2)/det,(m11*y2-m21*y1)/det};
        if (!std::isfinite(state_.lateral_velocity_m_s) ||
            !std::isfinite(state_.yaw_rate_rad_s))
            throw std::overflow_error("Model B state diverged");
    }
    return state_;
}

const BicycleState& LinearBicycleModel::state() const noexcept { return state_; }
const BicycleParameters& LinearBicycleModel::parameters() const noexcept {
    return parameters_;
}

} // namespace p4::models

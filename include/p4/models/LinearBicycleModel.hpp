#ifndef P4_MODELS_LINEAR_BICYCLE_MODEL_HPP_
#define P4_MODELS_LINEAR_BICYCLE_MODEL_HPP_

namespace p4::models {

// Model B: 2DOF linear bicycle model. SI units and radians throughout.
struct BicycleParameters {
    double mass_kg = 1500.0;
    double yaw_inertia_kg_m2 = 2500.0;
    double front_axle_m = 1.2;
    double rear_axle_m = 1.6;
    double front_cornering_stiffness_n_rad = 80000.0;
    double rear_cornering_stiffness_n_rad = 80000.0;
};

struct BicycleState {
    double lateral_velocity_m_s = 0.0;
    double yaw_rate_rad_s = 0.0;
};

class LinearBicycleModel {
public:
    explicit LinearBicycleModel(const BicycleParameters& parameters = {});
    void reset(const BicycleState& initial = {});
    BicycleState step(double front_steering_rad,
                      double longitudinal_speed_m_s, double dt_s);
    BicycleState stepImplicitMidpoint(double front_steering_rad,
                                     double longitudinal_speed_m_s,
                                     double dt_s,
                                     double max_substep_s = 0.005,
                                     double lateral_external_accel_m_s2 = 0.0);
    const BicycleState& state() const noexcept;
    const BicycleParameters& parameters() const noexcept;

private:
    BicycleParameters parameters_;
    BicycleState state_;
};

} // namespace p4::models

#endif

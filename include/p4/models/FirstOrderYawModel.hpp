#ifndef FIRST_ORDER_YAW_MODEL_HPP_
#define FIRST_ORDER_YAW_MODEL_HPP_

namespace p4::models {
    struct YawParameters {
        double gain = -0.85496948;
        double tau  = 0.104;
        double bias = 0.22854073;
    };

    class FirstOrderYawModel {
        public:
            explicit FirstOrderYawModel(
                const YawParameters& params = {}
            );
            void reset(double yaw_rate);
            double step(double steering_angle, double dt);
            double yawRate() const noexcept;
            const YawParameters& parameters() const noexcept;

            private:
                YawParameters params_;
                double yaw_rate_ = 0.0;
    };
} // namespace p4::models

#endif // FIRST_ORDER_YAW_MODEL_HPP_

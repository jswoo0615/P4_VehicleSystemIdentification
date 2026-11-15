#ifndef P4_IO_CONTROLLER_MODEL_HPP_
#define P4_IO_CONTROLLER_MODEL_HPP_

#include <string>

namespace p4::io {

struct ControllerModel {
    struct A { double gain, tau, bias; } a;
    struct ASpeed { double wheelbase, kv, tau, bias; } a_speed;
    struct B0 { double cf_over_m, cr_over_m, iz_over_m, lf, lr; } b0;
};

ControllerModel loadControllerModel(const std::string& path);
ControllerModel loadControllerModelText(const std::string& json);
// Reads the frozen fit values for an independent export parity comparison.
ControllerModel loadGolfBaselineForParity(const std::string& path);

} // namespace p4::io

#endif

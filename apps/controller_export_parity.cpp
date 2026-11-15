#include "p4/io/ControllerModel.hpp"
#include "p4/controller/ControllerRuntime.hpp"
#include "p4/models/FirstOrderYawModel.hpp"
#include "p4/models/LinearBicycleModel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace {
using p4::io::ControllerModel;
constexpr double G=9.80665;

double relative(double loaded,double reference) {
    return std::abs(loaded-reference)/std::max(std::abs(reference),1e-300);
}
double parameterError(const ControllerModel& loaded,const ControllerModel& reference) {
    const std::array<double,12> x={loaded.a.gain,loaded.a.tau,loaded.a.bias,
        loaded.a_speed.wheelbase,loaded.a_speed.kv,loaded.a_speed.tau,loaded.a_speed.bias,
        loaded.b0.cf_over_m,loaded.b0.cr_over_m,loaded.b0.iz_over_m,loaded.b0.lf,loaded.b0.lr};
    const std::array<double,12> y={reference.a.gain,reference.a.tau,reference.a.bias,
        reference.a_speed.wheelbase,reference.a_speed.kv,reference.a_speed.tau,reference.a_speed.bias,
        reference.b0.cf_over_m,reference.b0.cr_over_m,reference.b0.iz_over_m,reference.b0.lf,reference.b0.lr};
    double worst=0;
    for(std::size_t i=0;i<x.size();++i) worst=std::max(worst,relative(x[i],y[i]));
    return worst;
}
p4::models::BicycleParameters bicycle(const ControllerModel::B0& b) {
    return {1.0,b.iz_over_m,b.lf,b.lr,b.cf_over_m,b.cr_over_m};
}
double speedStep(double previous,const ControllerModel::ASpeed& p,
                 double steering,double speed,double dt) {
    const double a=std::exp(-dt/p.tau);
    return a*previous+(1-a)*(speed*steering/(p.wheelbase+p.kv*speed*speed)+p.bias);
}
std::string readText(const std::string& path) {
    std::ifstream in(path);if(!in) throw std::runtime_error("Cannot open "+path);
    return {std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
}
void replaceOnce(std::string& s,const std::string& before,const std::string& after) {
    const auto pos=s.find(before);
    if(pos==std::string::npos) throw std::runtime_error("Parity fixture key missing");
    s.replace(pos,before.size(),after);
}
void expectReject(const std::string& json) {
    try { (void)p4::io::loadControllerModelText(json); }
    catch(const std::exception&) { return; }
    throw std::runtime_error("Invalid controller export was accepted");
}
void negativeCases(const std::string& original) {
    auto bad=original;replaceOnce(bad,"\"schema\"","\"missing_schema\"");expectReject(bad);
    bad=original;replaceOnce(bad,"p4.controller_model.v1","p4.controller_model.v999");expectReject(bad);
    bad=original;replaceOnce(bad,"\"K_r_per_s\"","\"missing_K_r_per_s\"");expectReject(bad);
    const auto key=original.find("\"K_r_per_s\"");
    if(key==std::string::npos) throw std::runtime_error("Parity fixture coefficient missing");
    const auto colon=original.find(':',key),end=original.find(',',colon);
    if(colon==std::string::npos||end==std::string::npos) throw std::runtime_error("Parity fixture malformed");
    for(const std::string invalid:{" NaN"," Infinity"," 1e999"}) {
        bad=original;bad.replace(colon+1,end-colon-1,invalid);expectReject(bad);
    }
}
void parameterMismatchCase(const std::string& original,const ControllerModel& reference) {
    const auto key=original.find("\"K_r_per_s\"");
    if(key==std::string::npos) throw std::runtime_error("Parity fixture coefficient missing");
    const auto colon=original.find(':',key),end=original.find(',',colon);
    if(colon==std::string::npos||end==std::string::npos) throw std::runtime_error("Parity fixture malformed");
    auto changed=original;changed.replace(colon+1,end-colon-1," 99.0");
    const auto loaded=p4::io::loadControllerModelText(changed);
    if(parameterError(loaded,reference)<=1e-12)
        throw std::runtime_error("Altered coefficient was not detected");
}
}

int main(int argc,char** argv) {
    try {
        if(argc!=4) throw std::runtime_error("Usage: controller_export_parity controller.json baseline.json result.json");
        const auto loaded=p4::io::loadControllerModel(argv[1]);
        const auto reference=p4::io::loadGolfBaselineForParity(argv[2]);
        const double param_error=parameterError(loaded,reference);
        if(param_error>1e-12) throw std::runtime_error("Reloaded coefficients differ from baseline");
        const auto source=readText(argv[1]);
        negativeCases(source);
        parameterMismatchCase(source,reference);

        p4::models::FirstOrderYawModel a_loaded({loaded.a.gain,loaded.a.tau,loaded.a.bias});
        p4::models::FirstOrderYawModel a_reference({reference.a.gain,reference.a.tau,reference.a.bias});
        a_loaded.reset(.07);a_reference.reset(.07);
        p4::models::LinearBicycleModel b_loaded(bicycle(loaded.b0)),b_reference(bicycle(reference.b0));
        b_loaded.reset({.15,.07});b_reference.reset({.15,.07});
        p4::controller::ControllerRuntime runtime(loaded);
        runtime.reset(.07,.15);
        double speed_loaded=.07,speed_reference=.07;
        double first_error=0,max_error=0;
        for(int i=0;i<1000;++i) {
            const double t=i*.02,dt=.02;
            const double steering=.07*std::sin(.8*t)+.025*std::cos(.23*t);
            const double vx=17.0+5.0*std::sin(.13*t);
            const double bank=.02*std::sin(.17*t);
            const double ar=a_loaded.step(steering,dt);
            const double ar_ref=a_reference.step(steering,dt);
            speed_loaded=speedStep(speed_loaded,loaded.a_speed,steering,vx,dt);
            speed_reference=speedStep(speed_reference,reference.a_speed,steering,vx,dt);
            const auto br=b_loaded.stepImplicitMidpoint(steering,vx,dt,.02,G*std::sin(bank));
            const auto br_ref=b_reference.stepImplicitMidpoint(steering,vx,dt,.02,G*std::sin(bank));
            const auto live=runtime.step({steering,vx,bank,dt});
            const double error=std::max({std::abs(ar-ar_ref),std::abs(speed_loaded-speed_reference),
                std::abs(br.yaw_rate_rad_s-br_ref.yaw_rate_rad_s),
                std::abs(br.lateral_velocity_m_s-br_ref.lateral_velocity_m_s),
                std::abs(live.model_a_yaw_rad_s-ar_ref),
                std::abs(live.model_a_speed_yaw_rad_s-speed_reference),
                std::abs(live.model_b0_yaw_rad_s-br_ref.yaw_rate_rad_s),
                std::abs(live.model_b0_vy_m_s-br_ref.lateral_velocity_m_s)});
            if(i==0) first_error=error;
            max_error=std::max(max_error,error);
        }
        if(first_error>1e-10||max_error>1e-8)
            throw std::runtime_error("Prediction parity tolerance exceeded");
        std::ofstream out(argv[3]);if(!out) throw std::runtime_error("Cannot write parity result");
        out<<std::setprecision(17)
           <<"{\n  \"experiment\": \"P4-EXPORT-PARITY-001\",\n"
           <<"  \"status\": \"passed\",\n  \"steps\": 1000,\n"
           <<"  \"max_parameter_relative_error\": "<<param_error<<",\n"
           <<"  \"first_step_max_absolute_error\": "<<first_error<<",\n"
           <<"  \"all_steps_max_absolute_error\": "<<max_error<<",\n"
           <<"  \"negative_loader_cases\": [\"missing_schema\", \"wrong_schema\", "
           <<"\"missing_parameter\", \"NaN\", \"Infinity\", \"number_overflow\", "
           <<"\"parameter_mismatch\"]\n}\n";
        std::cout<<"P4-EXPORT-PARITY-001 passed: parameter relative error="<<param_error
                 <<" first step="<<first_error<<" 1000-step max="<<max_error<<'\n';
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}

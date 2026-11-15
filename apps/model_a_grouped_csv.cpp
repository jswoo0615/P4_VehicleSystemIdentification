#include "p4/io/IONIQ5CsvReader.hpp"
#include "p4/models/FirstOrderYawModel.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Segment = p4::io::ModelACsvSamples;
struct Fit { double gain=0, tau=0, bias=0, rmse=std::numeric_limits<double>::infinity(); };

double rmse(const std::vector<Segment>& segments, const Fit& fit) {
    double error2=0; std::size_t count=0;
    for (const auto& s : segments) {
        p4::models::FirstOrderYawModel model({fit.gain, fit.tau, fit.bias});
        model.reset(s.yaw_rad_s.front());
        for (std::size_t k=1; k<s.time_s.size(); ++k) {
            const double predicted=model.step(s.steering_rad[k-1],s.time_s[k]-s.time_s[k-1]);
            error2+=std::pow(s.yaw_rad_s[k]-predicted,2); ++count;
        }
    }
    return count ? std::sqrt(error2/count) : std::numeric_limits<double>::infinity();
}

Fit fitModel(const std::vector<Segment>& segments) {
    Fit best;
    for (int j=0; j<=250; ++j) {
        const double tau=0.03*std::pow(100.0,j/250.0);
        double qq=0,qh=0,hh=0,qy=0,hy=0;
        for (const auto& s : segments) {
            double p=s.yaw_rad_s.front(), q=0, h=0;
            for (std::size_t k=1; k<s.time_s.size(); ++k) {
                const double a=std::exp(-(s.time_s[k]-s.time_s[k-1])/tau);
                p*=a; q=a*q+(1-a)*s.steering_rad[k-1]; h=a*h+(1-a);
                const double y=s.yaw_rad_s[k]-p;
                qq+=q*q; qh+=q*h; hh+=h*h; qy+=q*y; hy+=h*y;
            }
        }
        const double determinant=qq*hh-qh*qh;
        if (!(determinant>1e-12*qq*hh)) continue;
        Fit candidate{(qy*hh-hy*qh)/determinant,tau,(hy*qq-qy*qh)/determinant,0};
        candidate.rmse=rmse(segments,candidate);
        if (candidate.rmse<best.rmse) best=candidate;
    }
    if (!std::isfinite(best.rmse)) throw std::runtime_error("Insufficient steering variation");
    return best;
}

std::size_t points(const std::vector<Segment>& segments) {
    std::size_t n=0; for (const auto& s:segments) n+=s.time_s.size(); return n;
}

void exportTest(const std::string& path, const std::vector<Segment>& segments,
                const Fit& fit, const std::filesystem::path& directory) {
    std::filesystem::create_directories(directory);
    const auto output=directory/(std::filesystem::path(path).stem().string()+"_model_a.csv");
    std::ofstream out(output);
    if (!out) throw std::runtime_error("Cannot write "+output.string());
    out << "segment,sample,time_s,delta_rad,yaw_measured_radps,yaw_predicted_radps,speed_mps,ay_mps2\n"
        << std::setprecision(17);
    for (std::size_t j=0;j<segments.size();++j) {
        const auto& s=segments[j];
        p4::models::FirstOrderYawModel model({fit.gain,fit.tau,fit.bias});
        model.reset(s.yaw_rad_s.front());
        for (std::size_t k=0;k<s.time_s.size();++k) {
            const double predicted=k ? model.step(s.steering_rad[k-1],s.time_s[k]-s.time_s[k-1])
                                     : model.yawRate();
            out << j+1 << ',' << k << ',' << s.time_s[k] << ',' << s.steering_rad[k]
                << ',' << s.yaw_rad_s[k] << ',' << predicted << ',' << s.speed_m_s[k]
                << ',' << s.lateral_accel_m_s2[k] << '\n';
        }
    }
    if (!out) throw std::runtime_error("Write failed: "+output.string());
    std::cout << "Wrote " << output << '\n';
}
}

int main(int argc,char** argv) {
    if (argc<4) { std::cerr<<"Usage: model_a_grouped_csv train.csv [train.csv ...] --test test.csv [test.csv ...] [--report-dir directory]\n"; return 1; }
    try {
        p4::io::IONIQ5CsvReader reader;
        std::vector<Segment> train,test;
        std::vector<std::string> train_paths,test_paths;
        bool testing=false;
        std::filesystem::path report_dir;
        for (int i=1;i<argc;++i) {
            if (std::string(argv[i])=="--test") { testing=true; continue; }
            if (std::string(argv[i])=="--report-dir") {
                if (++i==argc) throw std::runtime_error("Missing --report-dir value");
                report_dir=argv[i]; continue;
            }
            const auto canonical=std::filesystem::canonical(argv[i]).string();
            auto& paths=testing?test_paths:train_paths;
            paths.push_back(canonical);
            auto segments=reader.readModelASegments(canonical);
            std::cout<<(testing?"Test ":"Train ")<<std::filesystem::path(canonical).filename().string()
                     <<": "<<segments.size()<<" segments, "<<points(segments)<<" samples\n";
            auto& target=testing?test:train;
            for (auto& s:segments) target.push_back(std::move(s));
        }
        if (train_paths.empty()||test_paths.empty()||train.empty()||test.empty())
            throw std::runtime_error("Need nonempty train and test files with usable driving segments");
        for (const auto& path:train_paths)
            for (const auto& heldout:test_paths)
                if (path==heldout) throw std::runtime_error("Train/test file overlap: "+path);
        const auto fit=fitModel(train);
        std::cout<<std::setprecision(8)<<"K="<<fit.gain<<" 1/s tau="<<fit.tau
                 <<" s b="<<fit.bias<<" rad/s\nTrain RMSE="<<fit.rmse
                 <<" rad/s Test RMSE="<<rmse(test,fit)<<" rad/s\n";
        if (fit.tau<0.0301||fit.tau>2.999) std::cout<<"Warning: tau at search boundary\n";
        if (!report_dir.empty()) {
            std::size_t offset=0;
            for (const auto& path:test_paths) {
                const auto n=reader.readModelASegments(path).size();
                exportTest(path,{test.begin()+offset,test.begin()+offset+n},fit,report_dir);
                offset+=n;
            }
        }
        return 0;
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
}

#include "p4/controller/ControllerRuntime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
thread_local bool count_allocations=false;
thread_local std::size_t step_allocations=0;
}
void* operator new(std::size_t size) {
    if(count_allocations) ++step_allocations;
    if(void* p=std::malloc(size)) return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
    if(count_allocations) ++step_allocations;
    if(void* p=std::malloc(size)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept {std::free(p);}
void operator delete[](void* p) noexcept {std::free(p);}
void operator delete(void* p,std::size_t) noexcept {std::free(p);}
void operator delete[](void* p,std::size_t) noexcept {std::free(p);}

namespace {
struct Sample {double t,steer,vx,vy,yaw,bank;};
struct Scores {double a=0,as=0,b_yaw=0,b_vy=0;std::size_t n=0;};

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields;std::stringstream stream(line);std::string value;
    while(std::getline(stream,value,',')) fields.push_back(value);
    return fields;
}
double field(const std::vector<std::string>& values,const std::map<std::string,std::size_t>& columns,
             const char* name) {
    const auto it=columns.find(name);
    if(it==columns.end()||it->second>=values.size()) throw std::runtime_error(std::string("Missing CSV channel: ")+name);
    std::size_t used=0;double value;
    try {value=std::stod(values[it->second],&used);}
    catch(const std::exception&) {throw std::runtime_error(std::string("Invalid CSV value: ")+name);}
    if(used!=values[it->second].size()||!std::isfinite(value))
        throw std::runtime_error(std::string("Invalid CSV value: ")+name);
    return value;
}
std::vector<Sample> readSamples(const std::string& path,double start,double end) {
    std::ifstream in(path);if(!in) throw std::runtime_error("Cannot open CSV: "+path);
    std::string line;
    while(std::getline(in,line)&&(line.empty()||line.front()=='#')) {}
    if(line.empty()||line.front()=='#') throw std::runtime_error("CSV header missing");
    const auto names=split(line);std::map<std::string,std::size_t> columns;
    for(std::size_t i=0;i<names.size();++i) columns.emplace(names[i],i);
    for(const char* name:{"t_s","roadWheelAngle_rad","Vx_mps","Vy_mps","yawRate_radps","bank_rad"})
        if(!columns.count(name)) throw std::runtime_error(std::string("Missing CSV channel: ")+name);
    std::vector<Sample> samples;
    double previous=-INFINITY;
    while(std::getline(in,line)) {
        if(line.empty()) continue;
        const auto values=split(line);
        const double t=field(values,columns,"t_s");
        if(t<=previous) throw std::runtime_error("Nonincreasing CSV time");
        previous=t;if(t<start||t>end) continue;
        samples.push_back({t,field(values,columns,"roadWheelAngle_rad"),
            field(values,columns,"Vx_mps"),field(values,columns,"Vy_mps"),
            field(values,columns,"yawRate_radps"),field(values,columns,"bank_rad")});
    }
    if(samples.size()<2) throw std::runtime_error("Too few samples in interval");
    return samples;
}
double quantile(const std::vector<long long>& sorted,double p) {
    const double at=p*(sorted.size()-1);
    const auto lo=static_cast<std::size_t>(at);
    const auto hi=std::min(lo+1,sorted.size()-1);
    return sorted[lo]+(at-lo)*(sorted[hi]-sorted[lo]);
}
}

int main(int argc,char** argv) {
    try {
        if(argc!=7) throw std::runtime_error("Usage: controller_integration model.json drive.csv start_s end_s report.json predictions.csv");
        const double start=std::stod(argv[3]),end=std::stod(argv[4]);
        if(!std::isfinite(start)||!std::isfinite(end)||end<=start)
            throw std::runtime_error("Invalid time interval");
        const auto model=p4::io::loadControllerModel(argv[1]);
        const auto samples=readSamples(argv[2],start,end);
        p4::controller::ControllerRuntime runtime(model);
        std::ofstream predictions(argv[6]);if(!predictions) throw std::runtime_error("Cannot write predictions");
        predictions<<"t_s,measured_yaw_radps,model_a_yaw_radps,model_a_speed_yaw_radps,model_b0_yaw_radps,measured_vy_mps,model_b0_vy_mps\n"
                   <<std::setprecision(17);
        Scores scores;std::vector<long long> latency;latency.reserve(samples.size());
        std::size_t resets=0,skipped_speed=0;bool active=false;Sample previous{};
        for(const auto& s:samples) {
            if(s.vx<5) {active=false;++skipped_speed;continue;}
            if(!active||s.t-previous.t>.08) {
                runtime.reset(s.yaw,s.vy);previous=s;active=true;++resets;continue;
            }
            const p4::controller::ControlInput input{previous.steer,previous.vx,previous.bank,s.t-previous.t};
            const auto before=std::chrono::steady_clock::now();
            count_allocations=true;
            const auto prediction=runtime.step(input);
            count_allocations=false;
            const auto after=std::chrono::steady_clock::now();
            latency.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(after-before).count());
            const double ea=s.yaw-prediction.model_a_yaw_rad_s;
            const double eas=s.yaw-prediction.model_a_speed_yaw_rad_s;
            const double eb=s.yaw-prediction.model_b0_yaw_rad_s;
            const double ev=s.vy-prediction.model_b0_vy_m_s;
            scores.a+=ea*ea;scores.as+=eas*eas;scores.b_yaw+=eb*eb;scores.b_vy+=ev*ev;++scores.n;
            predictions<<s.t<<','<<s.yaw<<','<<prediction.model_a_yaw_rad_s<<','
                       <<prediction.model_a_speed_yaw_rad_s<<','<<prediction.model_b0_yaw_rad_s<<','
                       <<s.vy<<','<<prediction.model_b0_vy_m_s<<'\n';
            previous=s;
        }
        if(scores.n==0) throw std::runtime_error("No valid prediction steps");
        std::sort(latency.begin(),latency.end());
        const auto over_deadline=std::count_if(latency.begin(),latency.end(),
            [](long long ns){return ns>5000000;});
        std::ofstream report(argv[5]);if(!report) throw std::runtime_error("Cannot write report");
        report<<std::setprecision(17)
              <<"{\n  \"experiment\": \"P4-CONTROLLER-INTEGRATION-001\",\n"
              <<"  \"evaluation_kind\": \"in_sample_diagnostic_full_data_refit\",\n"
              <<"  \"controller_model\": \""<<argv[1]<<"\",\n"
              <<"  \"drive_file\": \""<<argv[2]<<"\",\n"
              <<"  \"interval_s\": ["<<start<<", "<<end<<"],\n"
              <<"  \"scored_samples\": "<<scores.n<<", \"segment_resets\": "<<resets
              <<", \"skipped_low_speed_rows\": "<<skipped_speed<<",\n"
              <<"  \"rmse\": {\"A_yaw_radps\": "<<std::sqrt(scores.a/scores.n)
              <<", \"A_Speed_yaw_radps\": "<<std::sqrt(scores.as/scores.n)
              <<", \"B0_yaw_radps\": "<<std::sqrt(scores.b_yaw/scores.n)
              <<", \"B0_vy_mps\": "<<std::sqrt(scores.b_vy/scores.n)<<"},\n"
              <<"  \"step_latency_ns\": {\"p50\": "<<quantile(latency,.5)
              <<", \"p95\": "<<quantile(latency,.95)<<", \"p99\": "<<quantile(latency,.99)
              <<", \"max\": "<<latency.back()<<", \"over_5ms_count\": "<<over_deadline<<"},\n"
              <<"  \"step_cpp_new_calls\": "<<step_allocations<<"\n}\n";
        std::cout<<"P4-CONTROLLER-INTEGRATION-001 samples="<<scores.n
                 <<" B0 yaw RMSE="<<std::sqrt(scores.b_yaw/scores.n)
                 <<" B0 vy RMSE="<<std::sqrt(scores.b_vy/scores.n)
                 <<" p99="<<quantile(latency,.99)<<" ns C++ new calls="<<step_allocations<<'\n';
    } catch(const std::exception& e) {count_allocations=false;std::cerr<<e.what()<<'\n';return 2;}
}

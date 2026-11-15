#include "p4/controller/ControllerRuntime.hpp"
#include "p4/identification/GolfDataset.hpp"
#include "p4/io/ControllerModel.hpp"
#include "p4/io/IONIQ5CsvReader.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Segment=p4::io::ModelACsvSamples;
using File=p4::identification::GolfFile;
using Params=p4::io::ControllerModel;
constexpr std::array<int,5> horizons{{0,5,10,25,50}}; // zero denotes the original full free run

std::vector<std::string> split(const std::string& line,char delimiter) {
    std::vector<std::string> result;
    std::stringstream stream(line);
    std::string field;
    while(std::getline(stream,field,delimiter)) result.push_back(field);
    return result;
}

struct FoldParams {Params params;std::vector<std::string> held;};
std::array<FoldParams,5> readFoldParams(const std::string& path) {
    std::ifstream input(path);
    if(!input) throw std::runtime_error("Cannot open fold parameters: "+path);
    std::string line;
    if(!std::getline(input,line)) throw std::runtime_error("Empty fold parameter file");
    std::array<FoldParams,5> out{};
    std::array<bool,5> seen{};
    while(std::getline(input,line)) {
        const auto fields=split(line,',');
        if(fields.size()!=14) throw std::runtime_error("Invalid fold parameter row");
        const int fold=std::stoi(fields[0]);
        if(fold<1||fold>5||seen[fold-1]) throw std::runtime_error("Invalid or repeated fold ID");
        seen[fold-1]=true;
        auto& row=out[fold-1];
        row.held=split(fields[1],'|');
        auto number=[&](std::size_t i) {
            const double v=std::stod(fields[i]);
            if(!std::isfinite(v)) throw std::runtime_error("Nonfinite fold coefficient");
            return v;
        };
        row.params={{number(2),number(3),number(4)},
                    {number(5),number(6),number(7),number(8)},
                    {number(9),number(10),number(11),number(12),number(13)}};
    }
    for(bool ok:seen) if(!ok) throw std::runtime_error("Missing fold parameter row");
    return out;
}

struct Score {
    double a2=0,as2=0,by2=0,bv2=0;
    std::size_t count=0;
    void add(const p4::controller::ControlOutput& pred,const Segment& s,std::size_t target) {
        const double a=pred.model_a_yaw_rad_s-s.yaw_rad_s[target];
        const double as=pred.model_a_speed_yaw_rad_s-s.yaw_rad_s[target];
        const double by=pred.model_b0_yaw_rad_s-s.yaw_rad_s[target];
        const double bv=pred.model_b0_vy_m_s-s.lateral_velocity_m_s[target];
        a2+=a*a;as2+=as*as;by2+=by*by;bv2+=bv*bv;++count;
    }
    void combine(const Score& other) {
        a2+=other.a2;as2+=other.as2;by2+=other.by2;bv2+=other.bv2;count+=other.count;
    }
};

Score scoreSegment(const Segment& s,const Params& params,int horizon,
                   double target_start,double target_end) {
    p4::controller::ControllerRuntime runtime(params);
    Score score;
    if(horizon==0) {
        runtime.reset(s.yaw_rad_s.front(),s.lateral_velocity_m_s.front());
        for(std::size_t j=1;j<s.time_s.size();++j) {
            const p4::controller::ControlInput in{s.steering_rad[j-1],s.speed_m_s[j-1],
                                                 s.bank_rad[j-1],s.time_s[j]-s.time_s[j-1]};
            const auto pred=runtime.step(in);
            if(s.time_s[j]>=target_start&&s.time_s[j]<=target_end) score.add(pred,s,j);
        }
    } else {
        const auto h=static_cast<std::size_t>(horizon);
        for(std::size_t origin=0;origin+h<s.time_s.size();++origin) {
            const std::size_t target=origin+h;
            if(s.time_s[target]<target_start||s.time_s[target]>target_end) continue;
            runtime.reset(s.yaw_rad_s[origin],s.lateral_velocity_m_s[origin]);
            p4::controller::ControlOutput pred{};
            for(std::size_t j=origin+1;j<=target;++j) {
                const p4::controller::ControlInput in{s.steering_rad[j-1],s.speed_m_s[j-1],
                                                     s.bank_rad[j-1],s.time_s[j]-s.time_s[j-1]};
                pred=runtime.step(in);
            }
            score.add(pred,s,target);
        }
    }
    return score;
}

Score scoreFile(const File& file,const Params& params,int horizon,
                double target_start,double target_end) {
    Score result;
    for(const auto& segment:file)
        result.combine(scoreSegment(segment,params,horizon,target_start,target_end));
    return result;
}

void writeRow(std::ofstream& out,const std::string& mode,int fold,
              const std::string& name,int horizon,const Score& score) {
    out<<mode<<','<<fold<<','<<name<<','<<horizon<<','<<horizon*.02<<','<<score.count<<','
       <<score.a2<<','<<score.as2<<','<<score.by2<<','<<score.bv2<<'\n';
}

void writeHeader(std::ofstream& out) {
    out<<std::setprecision(17)
       <<"mode,fold,file,horizon_steps,horizon_nominal_s,samples,a_yaw_sse,as_yaw_sse,"
         "b_yaw_sse,b_vy_sse\n";
}
} // namespace

int main(int argc,char** argv) {
    try {
        if(argc<2) throw std::runtime_error("Expected --cv or --diagnostic");
        const std::string mode=argv[1];
        if(mode=="--cv") {
            if(argc!=5) throw std::runtime_error("Usage: --cv dataset_dir fold_params.csv result.csv");
            const auto dataset=p4::identification::GolfDataset::load(argv[2]);
            const auto params=readFoldParams(argv[3]);
            std::ofstream out(argv[4]);
            if(!out) throw std::runtime_error("Cannot write result CSV");
            writeHeader(out);
            for(std::size_t f=0;f<5;++f) {
                const auto fold=dataset.fold(f);
                if(params[f].held!=fold.held_names)
                    throw std::runtime_error("Held-out file list differs from the existing CV fold");
                for(std::size_t i=0;i<fold.validation.size();++i)
                    for(int horizon:horizons)
                        writeRow(out,"file_cv",static_cast<int>(f+1),fold.held_names[i],horizon,
                                 scoreFile(fold.validation[i],params[f].params,horizon,
                                           -std::numeric_limits<double>::infinity(),
                                           std::numeric_limits<double>::infinity()));
            }
        } else if(mode=="--diagnostic") {
            if(argc!=9) throw std::runtime_error(
                "Usage: --diagnostic csv baseline.json start_s end_s target_start_s target_end_s result.csv");
            const auto params=p4::io::loadGolfBaselineForParity(argv[3]);
            const double start=std::stod(argv[4]),end=std::stod(argv[5]);
            const double target_start=std::stod(argv[6]),target_end=std::stod(argv[7]);
            if(!(start<=target_start&&target_start<target_end&&target_end<=end))
                throw std::runtime_error("Invalid diagnostic time interval");
            p4::io::IONIQ5CsvReader reader(p4::io::CsvChannelMap::StanfordGolf);
            p4::io::ModelACsvOptions options;options.minimum_speed_m_s=5;options.sample_period_s=.02;
            const auto file=reader.readModelASegments(argv[2],options,start,end);
            std::ofstream out(argv[8]);
            if(!out) throw std::runtime_error("Cannot write result CSV");
            writeHeader(out);
            for(int horizon:horizons)
                writeRow(out,"full_fit_diagnostic",0,"2023-11_1.csv",horizon,
                         scoreFile(file,params,horizon,target_start,target_end));
        } else throw std::runtime_error("Expected --cv or --diagnostic");
    } catch(const std::exception& error) {
        std::cerr<<"golf_prediction_horizon: "<<error.what()<<'\n';return 1;
    }
}

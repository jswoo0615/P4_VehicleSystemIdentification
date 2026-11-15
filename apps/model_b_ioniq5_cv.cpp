#include "p4/io/IONIQ5CsvReader.hpp"
#include "p4/models/LinearBicycleModel.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {
using Segment=p4::io::ModelACsvSamples;
using File=std::vector<Segment>;
struct Score {double yaw_sse=0,vy_sse=0; std::size_t n=0;};
struct Scale {double yaw=0,vy=0;};
struct Fit {double cf=0,cr=0,objective=INFINITY;};
using DomainValues=std::array<std::vector<double>,3>;

// parameter.m: trvec_cor_ra=[0.958,-0.159,0], trvec_ra_cog=[1.437,0,0].
// ISO 8855 (x forward, y left, z up): v_cog=v_cor+omega cross r_cor_to_cog.
double vyCog(const Segment& s,std::size_t i) {
    return s.lateral_velocity_m_s[i]+2.395*s.yaw_rad_s[i];
}
double vxCog(const Segment& s,std::size_t i) {
    return s.speed_m_s[i]+0.159*s.yaw_rad_s[i];
}

DomainValues domainValues(const std::vector<File>& files) {
    DomainValues values;
    for(const auto& file:files) for(const auto& s:file)
        for(std::size_t i=1;i<s.time_s.size();++i) {
            values[0].push_back(vxCog(s,i));
            values[1].push_back(std::abs(s.steering_rad[i]));
            values[2].push_back(std::abs(s.lateral_accel_m_s2[i]));
        }
    for(auto& v:values) {
        if(v.empty()) throw std::runtime_error("No operating-domain samples");
        if(!std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);}))
            throw std::runtime_error("Nonfinite operating-domain signal");
        std::sort(v.begin(),v.end());
    }
    return values;
}

double quantile(const std::vector<double>& values,double p) {
    const double pos=p*(values.size()-1);
    const auto lo=static_cast<std::size_t>(pos);
    const auto hi=std::min(lo+1,values.size()-1);
    return values[lo]+(pos-lo)*(values[hi]-values[lo]);
}

void writeDomainReport(const std::filesystem::path& path,const std::vector<File>& train,
                       const std::vector<File>& tests,const std::vector<std::string>& names) {
    if(!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if(!out) throw std::runtime_error("Cannot write domain report: "+path.string());
    out<<"group,signal,unit,n,min,p05,median,p95,max,outside_train_minmax_fraction,outside_train_p05_p95_fraction\n"
       <<std::setprecision(12);
    const auto reference=domainValues(train);
    const std::array<const char*,3> labels={"vx_cog","abs_delta_stm","abs_ay_rear_axle"};
    const std::array<const char*,3> units={"m/s","rad","m/s2"};
    auto write=[&](const std::string& group,const DomainValues& values) {
        for(std::size_t j=0;j<values.size();++j) {
            const auto& v=values[j];const auto& r=reference[j];
            const double lo=quantile(r,.05),hi=quantile(r,.95);
            const auto outside=[&](double low,double high) {
                return static_cast<double>(std::count_if(v.begin(),v.end(),
                    [&](double x){return x<low||x>high;}))/v.size();
            };
            out<<group<<','<<labels[j]<<','<<units[j]<<','<<v.size()<<','
               <<v.front()<<','<<quantile(v,.05)<<','<<quantile(v,.5)<<','
               <<quantile(v,.95)<<','<<v.back()<<','
               <<outside(r.front(),r.back())<<','<<outside(lo,hi)<<'\n';
        }
    };
    write("train_asphalt_a",reference);
    for(std::size_t i=0;i<tests.size();++i)
        write(std::filesystem::path(names[i]).filename().string(),domainValues({tests[i]}));
    if(!out) throw std::runtime_error("Domain report write failed: "+path.string());
    std::cout<<"Wrote operating-domain report "<<path<<'\n';
}

Score evaluate(const std::vector<File>& files,double cf,double cr) {
    const p4::models::BicycleParameters parameters{2254.0,2193.0,1.563,1.437,cf,cr};
    Score out;
    for(const auto& file:files) for(const auto& s:file) {
        p4::models::LinearBicycleModel model(parameters);
        model.reset({vyCog(s,0),s.yaw_rad_s.front()});
        for(std::size_t i=1;i<s.time_s.size();++i) {
            const auto prediction=model.stepImplicitMidpoint(
                s.steering_rad[i-1],vxCog(s,i-1),s.time_s[i]-s.time_s[i-1]);
            const double ey=s.yaw_rad_s[i]-prediction.yaw_rate_rad_s;
            const double ev=vyCog(s,i)-prediction.lateral_velocity_m_s;
            out.yaw_sse+=ey*ey; out.vy_sse+=ev*ev; ++out.n;
        }
    }
    return out;
}

Scale trainingScale(const std::vector<File>& files) {
    double sy=0,sv=0,yy=0,vv=0; std::size_t n=0;
    for(const auto& file:files) for(const auto& s:file)
        for(std::size_t i=1;i<s.time_s.size();++i) {
            const double y=s.yaw_rad_s[i],v=vyCog(s,i);
            sy+=y;sv+=v;yy+=y*y;vv+=v*v;++n;
        }
    if(n==0) throw std::runtime_error("No Model B training samples");
    const double yaw=std::sqrt(std::max(0.0,yy/n-(sy/n)*(sy/n)));
    const double vy=std::sqrt(std::max(0.0,vv/n-(sv/n)*(sv/n)));
    if(yaw<.01||vy<.01) throw std::runtime_error("Too little yaw or lateral-velocity variation");
    return {yaw,vy};
}

double objective(const std::vector<File>& files,double cf,double cr,const Scale& scale) {
    if(cf<10000||cf>500000||cr<10000||cr>500000) return INFINITY;
    const auto s=evaluate(files,cf,cr);
    return (s.yaw_sse/(scale.yaw*scale.yaw)+s.vy_sse/(scale.vy*scale.vy))/s.n;
}

Fit fit(const std::vector<File>& files,const Scale& scale) {
    Fit best;
    for(double cf:{40000.0,100000.0,234008.0,400000.0})
        for(double cr:{40000.0,100000.0,234008.0,400000.0}) {
            const double value=objective(files,cf,cr,scale);
            if(value<best.objective) best={cf,cr,value};
        }
    // Deterministic log-space pattern search with multiple coarse starts.
    for(double step:{.4,.2,.1,.05,.025,.0125}) {
        for(int iteration=0;iteration<30;++iteration) {
            Fit candidate=best;
            for(int dc=-1;dc<=1;++dc) for(int dr=-1;dr<=1;++dr) {
                if(dc==0&&dr==0) continue;
                const double cf=best.cf*std::exp(dc*step);
                const double cr=best.cr*std::exp(dr*step);
                const double value=objective(files,cf,cr,scale);
                if(value<candidate.objective) candidate={cf,cr,value};
            }
            if(candidate.objective>=best.objective-1e-12) break;
            best=candidate;
        }
    }
    if(!std::isfinite(best.objective)) throw std::runtime_error("No valid Model B fit");
    return best;
}

std::vector<File> excluding(const std::vector<File>& files,std::size_t held) {
    std::vector<File> result;
    for(std::size_t i=0;i<files.size();++i) if(i!=held) result.push_back(files[i]);
    return result;
}

void printScore(const std::string& prefix,const Score& s) {
    std::cout<<prefix<<" yaw_RMSE="<<std::sqrt(s.yaw_sse/s.n)
             <<" rad/s vy_RMSE="<<std::sqrt(s.vy_sse/s.n)
             <<" m/s n="<<s.n<<'\n';
}

void printFoldSpread(const std::vector<std::array<double,2>>& values) {
    double cf=0,cr=0;
    for(const auto& v:values) {cf+=v[0];cr+=v[1];}
    cf/=values.size();cr/=values.size();
    double cf2=0,cr2=0,cross=0;
    for(const auto& v:values) {
        const double x=v[0]-cf,y=v[1]-cr;
        cf2+=x*x;cr2+=y*y;cross+=x*y;
    }
    const double denom=values.size()-1;
    std::cout<<"fold stiffness summary mean_Cf="<<cf<<" sd_Cf="<<std::sqrt(cf2/denom)
             <<" mean_Cr="<<cr<<" sd_Cr="<<std::sqrt(cr2/denom)
             <<" correlation="<<cross/std::sqrt(cf2*cr2)<<'\n';
}

void printCurvature(const std::vector<File>& files,const Scale& scale,const Fit& f) {
    const double h=.02;
    auto loss=[&](double x,double y) {
        return objective(files,f.cf*std::exp(x),f.cr*std::exp(y),scale);
    };
    const double f0=loss(0,0),xp=loss(h,0),xm=loss(-h,0);
    const double yp=loss(0,h),ym=loss(0,-h);
    const double hxx=(xp-2*f0+xm)/(h*h);
    const double hyy=(yp-2*f0+ym)/(h*h);
    const double hxy=(loss(h,h)-loss(h,-h)-loss(-h,h)+loss(-h,-h))/(4*h*h);
    const double disc=std::sqrt((hxx-hyy)*(hxx-hyy)+4*hxy*hxy);
    const double low=(hxx+hyy-disc)/2,high=(hxx+hyy+disc)/2;
    std::cout<<"local log-stiffness curvature hxx="<<hxx<<" hxy="<<hxy
             <<" hyy="<<hyy<<" eigen_low="<<low<<" eigen_high="<<high
             <<" condition="<<(low>0?high/low:INFINITY)
             <<" grad_Cf="<<(xp-xm)/(2*h)<<" grad_Cr="<<(yp-ym)/(2*h)<<'\n';
}

void writeLossMap(const std::filesystem::path& path,const std::vector<File>& files,
                  const Scale& scale) {
    if(!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if(!out) throw std::runtime_error("Cannot write loss map: "+path.string());
    out<<"cf_N_per_rad,cr_N_per_rad,objective,yaw_rmse_radps,vy_rmse_mps\n"
       <<std::setprecision(17);
    for(int ci=10;ci<=70;++ci) for(int ri=10;ri<=70;++ri) {
        const double cf=ci*4000.0,cr=ri*4000.0;
        const auto s=evaluate(files,cf,cr);
        out<<cf<<','<<cr<<','
           <<(s.yaw_sse/(scale.yaw*scale.yaw)+s.vy_sse/(scale.vy*scale.vy))/s.n
           <<','<<std::sqrt(s.yaw_sse/s.n)<<','<<std::sqrt(s.vy_sse/s.n)<<'\n';
    }
    if(!out) throw std::runtime_error("Loss-map write failed: "+path.string());
    std::cout<<"Wrote loss map "<<path<<" (61 x 61 grid)\n";
}
}

int main(int argc,char** argv) {
    try {
        if(argc<4) throw std::runtime_error("Usage: model_b_ioniq5_cv train1.csv train2.csv ... [--test test.csv ...]");
        p4::io::IONIQ5CsvReader reader;
        std::vector<std::string> names,test_names; std::vector<File> files,tests;
        std::filesystem::path loss_map_path,domain_report_path;
        std::unordered_set<std::string> seen;
        bool testing=false;
        for(int i=1;i<argc;++i) {
            if(std::string(argv[i])=="--test") {testing=true;continue;}
            if(std::string(argv[i])=="--loss-map") {
                if(++i==argc) throw std::runtime_error("Missing --loss-map path");
                loss_map_path=argv[i];continue;
            }
            if(std::string(argv[i])=="--domain-report") {
                if(++i==argc) throw std::runtime_error("Missing --domain-report path");
                domain_report_path=argv[i];continue;
            }
            const auto path=std::filesystem::canonical(argv[i]).string();
            if(!seen.insert(path).second) throw std::runtime_error("Duplicate or train/test overlap: "+path);
            auto segments=reader.readModelASegments(path);
            if(segments.empty()) throw std::runtime_error("No usable segments: "+path);
            for(const auto& s:segments) for(double v:s.lateral_velocity_m_s)
                if(!std::isfinite(v)) throw std::runtime_error("Missing lateral velocity: "+path);
            (testing?test_names:names).push_back(path);
            (testing?tests:files).push_back(std::move(segments));
        }
        if(files.size()<3) throw std::runtime_error("Need at least three training files");
        std::cout<<std::fixed<<std::setprecision(6)
                 <<"Model B file-group CV: "<<files.size()<<" folds, fixed m=2254 Iz=2193 lf=1.563 lr=1.437\n"
                 <<"CorrevIT to CoG velocity correction: vx_cog=vx_cor+0.159*r, vy_cog=vy_cor+2.395*r\n";
        Score pooled,reference;
        std::vector<std::array<double,2>> fold_parameters;
        for(std::size_t i=0;i<files.size();++i) {
            const auto train=excluding(files,i);
            const auto scale=trainingScale(train);
            const auto fit_result=fit(train,scale);
            fold_parameters.push_back({fit_result.cf,fit_result.cr});
            const auto test=evaluate({files[i]},fit_result.cf,fit_result.cr);
            const auto ref=evaluate({files[i]},234008.0,234008.0);
            pooled.yaw_sse+=test.yaw_sse;pooled.vy_sse+=test.vy_sse;pooled.n+=test.n;
            reference.yaw_sse+=ref.yaw_sse;reference.vy_sse+=ref.vy_sse;reference.n+=ref.n;
            const auto name=std::filesystem::path(names[i]).filename().string();
            std::cout<<"fold "<<name<<" Cf="<<fit_result.cf<<" Cr="<<fit_result.cr
                     <<" objective="<<fit_result.objective
                     <<" train_yaw_scale="<<scale.yaw<<" train_vy_scale="<<scale.vy<<'\n';
            printScore("fit "+name,test);
            printScore("reference "+name,ref);
        }
        printScore("pooled fit",pooled);
        printScore("pooled reference",reference);
        printFoldSpread(fold_parameters);
        const auto full_scale=trainingScale(files);
        const auto full=fit(files,full_scale);
        std::cout<<"all-file fit Cf="<<full.cf<<" Cr="<<full.cr
                 <<" objective="<<full.objective<<'\n';
        printScore("all-file training",evaluate(files,full.cf,full.cr));
        printCurvature(files,full_scale,full);
        const double understeer=2254.0/3.0*(1.437/full.cf-1.563/full.cr);
        const double reference_understeer=2254.0/3.0*(1.437/234008.0-1.563/234008.0);
        std::cout<<"steady-state denominator kv_B="<<understeer
                 <<" s^2/m reference_kv_B="<<reference_understeer<<" s^2/m\n";
        if(!loss_map_path.empty()) writeLossMap(loss_map_path,files,full_scale);
        if(!domain_report_path.empty()) writeDomainReport(domain_report_path,files,tests,test_names);
        for(std::size_t i=0;i<tests.size();++i) {
            const auto name=std::filesystem::path(test_names[i]).filename().string();
            printScore("new test fit "+name,evaluate({tests[i]},full.cf,full.cr));
            printScore("new test reference "+name,evaluate({tests[i]},234008.0,234008.0));
        }
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}

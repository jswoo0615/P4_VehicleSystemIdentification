#include "p4/io/IONIQ5LoadSamples.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Point=p4::io::LoadPoint;
using Coeff=std::array<double,3>;
Coeff solve(const std::vector<Point>& samples,bool left) {
    double a[3][4]={};
    for(const auto& p:samples) {
        const double x[3]={1,p.delta,p.rate}; const double y=left?p.left:p.right;
        for(int i=0;i<3;++i) { for(int j=0;j<3;++j) a[i][j]+=x[i]*x[j]; a[i][3]+=x[i]*y; }
    }
    for(int i=0;i<3;++i) {
        int pivot=i; for(int j=i+1;j<3;++j) if(std::abs(a[j][i])>std::abs(a[pivot][i])) pivot=j;
        if(std::abs(a[pivot][i])<1e-10) throw std::runtime_error("S-Load design is rank deficient");
        for(int j=0;j<4;++j) std::swap(a[i][j],a[pivot][j]);
        const double scale=a[i][i]; for(int j=i;j<4;++j) a[i][j]/=scale;
        for(int k=0;k<3;++k) if(k!=i) { const double m=a[k][i]; for(int j=i;j<4;++j) a[k][j]-=m*a[i][j]; }
    }
    return {a[0][3],a[1][3],a[2][3]};
}
double score(const std::vector<Point>& v,const Coeff& c,bool left) {
    double e=0; for(const auto& p:v) { const double d=(left?p.left:p.right)-(c[0]+c[1]*p.delta+c[2]*p.rate); e+=d*d; }
    return std::sqrt(e/v.size());
}
double baseline(const std::vector<Point>& train,const std::vector<Point>& test,bool left) {
    double mean=0,e=0; for(const auto& p:train) mean+=left?p.left:p.right; mean/=train.size();
    for(const auto& p:test) {double d=(left?p.left:p.right)-mean;e+=d*d;} return std::sqrt(e/test.size());
}
void exportTest(const std::string& path,const std::vector<Point>& samples,
                const Coeff& l,const Coeff& r,const std::filesystem::path& directory) {
    std::filesystem::create_directories(directory);
    const auto output=directory/(std::filesystem::path(path).stem().string()+"_s_load.csv");
    std::ofstream out(output);
    if(!out) throw std::runtime_error("Cannot write "+output.string());
    out<<"time_s,delta_rad,delta_dot_radps,left_measured_N,left_predicted_N,right_measured_N,right_predicted_N\n"
       <<std::setprecision(17);
    for(const auto& p:samples)
        out<<p.t<<','<<p.delta<<','<<p.rate<<','<<p.left<<','
           <<l[0]+l[1]*p.delta+l[2]*p.rate<<','<<p.right<<','
           <<r[0]+r[1]*p.delta+r[2]*p.rate<<'\n';
    if(!out) throw std::runtime_error("Write failed: "+output.string());
    std::cout<<"Wrote "<<output<<'\n';
}
}
int main(int argc,char** argv) {
    if(argc<4) {std::cerr<<"Usage: model_s_load_csv train.csv [train.csv ...] --test test.csv [test.csv ...] [--report-dir directory]\n";return 1;}
    try {
        std::vector<Point> train,test; std::vector<std::string> train_paths,test_paths; bool testing=false;
        std::filesystem::path report_dir;
        for(int i=1;i<argc;++i) {
            if(std::string(argv[i])=="--test") {testing=true;continue;}
            if(std::string(argv[i])=="--report-dir") {
                if(++i==argc) throw std::runtime_error("Missing --report-dir value");
                report_dir=argv[i]; continue;
            }
            const auto path=std::filesystem::canonical(argv[i]).string();
            (testing?test_paths:train_paths).push_back(path);
            auto v=p4::io::readGuardedLoadSamples(path);
            std::cout<<(testing?"Test ":"Train ")<<std::filesystem::path(path).filename().string()<<": "<<v.size()<<" guarded steering updates\n";
            (testing?test:train).insert((testing?test:train).end(),v.begin(),v.end());
        }
        if(train_paths.empty()||test_paths.empty()||train.size()<30||test.size()<30) throw std::runtime_error("Need separate train/test files with >=30 updates");
        for(const auto& x:train_paths) for(const auto& y:test_paths) if(x==y) throw std::runtime_error("Train/test file overlap");
        const auto l=solve(train,true),r=solve(train,false);
        std::cout<<std::setprecision(8);
        for(bool left:{true,false}) {
            const auto& c=left?l:r;
            std::cout<<(left?"F_trl_N":"F_trr_N")<<" = "<<c[0]<<" + "<<c[1]<<"*delta_rad + "<<c[2]<<"*delta_dot_radps\n"
                     <<"Train RMSE="<<score(train,c,left)<<" N Test RMSE="<<score(test,c,left)
                     <<" N Test mean-only baseline="<<baseline(train,test,left)<<" N\n";
        }
        if(!report_dir.empty()) for(const auto& path:test_paths)
            exportTest(path,p4::io::readGuardedLoadSamples(path),l,r,report_dir);
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 2;}
}
